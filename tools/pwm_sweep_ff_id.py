#!/usr/bin/env python3
"""
PWM Sweep Feedforward Identification.

Mengirim PWM langsung via encoding RPM=pwm+601, lalu mengukur actual RPM
untuk fitting linear PWM = a + b * RPM. Hanya motor tertentu yang diuji;
motor lain dikirim 0.

Safety: selalu kirim stop (RPM=0) di akhir, termasuk saat Ctrl+C.
"""
import argparse
import csv
import serial
import struct
import time
from typing import List, Tuple

# ---------------------------------------------------------------------------
# Protokol serial (identik dengan firmware STM32 & ESP32)
# ---------------------------------------------------------------------------
PACKET_HEADER = 0xA5
CMD_SIZE = 8        # header(1) + 3×int16(6) + xor(1)
FEEDBACK_SIZE = 42  # header(1) + ticks(12) + gyro(6) + accel(6) + ultrasonic(16) + xor(1)

TICKS_PER_REV = 380.0  # Harus sama dengan CONTROL_TICKS_PER_REV di firmware

# Maximum plausible tick delta per 50Hz feedback (600 RPM * 380 t/r / 60 / 50 = 76).
# Anything beyond this is clearly corrupt data (parser misalignment, noise, etc.).
MAX_PLAUSIBLE_TICK_DELTA = 2000  # generous margin above theoretical 76

CMD_INTERVAL = 0.02  # Kirim command ~50Hz (sinkron dengan feedback rate)
STEP_DURATION = 2.5
STEADY_WINDOW = 1.5
GAP_DURATION = 0.5

PWM_STEPS: List[int] = [
    0, 10, 15, 20, 25, 30, 40, 50,
    60, 70, 80, 100, 120, 150, 180,
    200, 220, 255,
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def xor_checksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def build_command(rpm1: int, rpm2: int, rpm3: int) -> bytes:
    """Bangun 8-byte command packet tanpa clamp ±600 (dipakai untuk PWM direct)."""
    body = struct.pack('>Bhhh', PACKET_HEADER, int(rpm1), int(rpm2), int(rpm3))
    return body + bytes([xor_checksum(body)])


def parse_feedback(packet: bytes):
    """Parse 42-byte feedback → (t1, t2, t3, gx, gy, gz, ax, ay, az) or None."""
    if len(packet) != FEEDBACK_SIZE:
        return None
    if packet[0] != PACKET_HEADER:
        return None
    if xor_checksum(packet[:-1]) != packet[-1]:
        return None
    if packet[1] == PACKET_HEADER:
        return None
    ticks = struct.unpack('>iii', packet[1:13])
    gyro = struct.unpack('>hhh', packet[13:19])
    accel = struct.unpack('>hhh', packet[19:25])
    for t in ticks:
        if abs(t) > MAX_PLAUSIBLE_TICK_DELTA:
            return None
    return ticks[0], ticks[1], ticks[2], gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]


def extract_packets(buf: bytearray) -> List[bytes]:
    """Extract semua feedback packet valid dari buffer."""
    packets: List[bytes] = []
    header = bytes([PACKET_HEADER])
    while True:
        idx = buf.find(header)
        if idx < 0:
            buf.clear()
            break
        if idx > 0:
            del buf[:idx]
        if len(buf) < FEEDBACK_SIZE:
            break
        candidate = bytes(buf[:FEEDBACK_SIZE])
        if xor_checksum(candidate[:-1]) == candidate[-1]:
            packets.append(candidate)
            del buf[:FEEDBACK_SIZE]
        else:
            del buf[:1]
    return packets


def ticks_to_rpm(delta_ticks: int, dt: float) -> float:
    if dt <= 0.0:
        return 0.0
    return delta_ticks * 60.0 / (TICKS_PER_REV * dt)


def send_stop(ser):
    """Kirim RPM=0 ke semua motor (best-effort, abaikan error)."""
    try:
        for _ in range(5):
            ser.write(build_command(0, 0, 0))
            time.sleep(0.02)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='PWM sweep feedforward identification (PWM -> RPM mapping).')
    ap.add_argument('--port', default='/dev/ttyACM0', help='Serial port (default: /dev/ttyACM0)')
    ap.add_argument('--baud', type=int, default=115200, help='Baud rate')
    ap.add_argument('--motor', required=True, type=int, choices=[1, 2, 3], help='Motor yang diuji: 1/2/3')
    args = ap.parse_args()

    motor_idx = args.motor - 1

    print(f'Port      : {args.port} @ {args.baud}')
    print(f'Motor     : M{args.motor}')
    print(f'PWM steps : {PWM_STEPS}')
    print()

    ser = serial.Serial(
        args.port, args.baud, timeout=0.02,
        xonxoff=False, rtscts=False, dsrdtr=False,
    )
    ser.dtr = False
    ser.rts = False

    time.sleep(0.2)
    ser.reset_input_buffer()

    print('Warm-up: kirim RPM=0 selama 0.5s...')
    warmup_end = time.time() + 0.5
    while time.time() < warmup_end:
        ser.write(build_command(0, 0, 0))
        time.sleep(CMD_INTERVAL)
    ser.reset_input_buffer()

    buf = bytearray()
    results: List[Tuple[int, float]] = []

    try:
        for pwm in PWM_STEPS:
            cmd_rpm = [0, 0, 0]
            cmd_rpm[motor_idx] = pwm + 601  # direct PWM encoding

            print(f'PWM {pwm:3d} → send RPM={cmd_rpm[motor_idx]} for {STEP_DURATION}s...')
            step_start = time.monotonic()
            t_last_cmd = 0.0
            t_last_fb = None
            samples: List[Tuple[float, float]] = []  # (t_rel, rpm)

            while True:
                t_now = time.monotonic()
                if t_now - step_start >= STEP_DURATION:
                    break

                if (t_now - t_last_cmd) >= CMD_INTERVAL:
                    ser.write(build_command(*cmd_rpm))
                    t_last_cmd = t_now

                waiting = ser.in_waiting
                if waiting > 0:
                    buf.extend(ser.read(waiting))
                else:
                    buf.extend(ser.read(1))

                for pkt in extract_packets(buf):
                    result = parse_feedback(pkt)
                    if result is None:
                        continue
                    d1, d2, d3, _gx, _gy, _gz, _ax, _ay, _az = result
                    t_fb = time.monotonic()
                    if t_last_fb is None:
                        t_last_fb = t_fb
                        continue
                    dt = t_fb - t_last_fb
                    t_last_fb = t_fb

                    rpm1 = ticks_to_rpm(d1, dt)
                    rpm2 = ticks_to_rpm(d2, dt)
                    rpm3 = ticks_to_rpm(d3, dt)
                    rpm_sel = (rpm1, rpm2, rpm3)[motor_idx]
                    samples.append((t_fb - step_start, rpm_sel))

            steady_samples = [rpm for t, rpm in samples if t >= (STEP_DURATION - STEADY_WINDOW)]
            if steady_samples:
                avg_rpm = sum(steady_samples) / len(steady_samples)
            elif samples:
                avg_rpm = sum(r for _, r in samples) / len(samples)
            else:
                avg_rpm = 0.0

            results.append((pwm, avg_rpm))
            print(f'  -> avg_rpm (last {STEADY_WINDOW:.1f}s) = {avg_rpm:.2f} ({len(samples)} samples)')

            gap_end = time.monotonic() + GAP_DURATION
            while time.monotonic() < gap_end:
                ser.write(build_command(0, 0, 0))
                time.sleep(CMD_INTERVAL)
            ser.reset_input_buffer()

    except KeyboardInterrupt:
        print('\n[Ctrl+C] Sweep dihentikan oleh user...')
    finally:
        print('\nMengirim STOP (RPM=0)...')
        send_stop(ser)
        ser.close()

    if not results:
        print('Tidak ada data hasil.')
        return

    filtered = [(pwm, rpm) for pwm, rpm in results if rpm > 8.0]

    print('\nHasil sweep (raw):')
    print(f"{'PWM':>5}  {'avg_rpm':>10}  {'used':>5}")
    for pwm, rpm in results:
        mark = 'yes' if rpm > 8.0 else 'no'
        print(f'{pwm:5d}  {rpm:10.2f}  {mark:>5}')

    if len(filtered) >= 2:
        xs = [rpm for _, rpm in filtered]
        ys = [pwm for pwm, _ in filtered]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom == 0.0:
            slope = 0.0
        else:
            slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
        offset = mean_y - slope * mean_x

        print('\nLinear regression (PWM = a + b * RPM):')
        print(f'  FF_OFFSET (a) = {offset:.3f}')
        print(f'  FF_SLOPE  (b) = {slope:.5f}')
    else:
        offset = 0.0
        slope = 0.0
        print('\nTidak cukup data untuk regresi (butuh ≥2 titik dengan avg_rpm > 8).')

    csv_path = f'pwm_sweep_M{args.motor}.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['pwm', 'avg_rpm'])
        writer.writerows(results)
    print(f'CSV tersimpan: {csv_path}')


if __name__ == '__main__':
    main()
