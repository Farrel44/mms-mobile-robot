#!/usr/bin/env python3
"""
RPM Step Response Test — Kirim step RPM ke STM32, rekam respons encoder.

Mengirim target RPM konstan, merekam actual RPM dari feedback encoder.
Data disimpan ke CSV untuk analisis step response / PID tuning di MATLAB.

Contoh:
  python3 test_rpm_step.py --rpm 150 --duration 3
  python3 test_rpm_step.py --rpm 100 --duration 5 --motor 2
  python3 test_rpm_step.py --rpm 200 --duration 4 --motor all --csv data.csv

Output CSV columns:
  time_s, cmd_rpm1, cmd_rpm2, cmd_rpm3, actual_rpm1, actual_rpm2, actual_rpm3

Safety:
  - Ctrl+C atau durasi habis → otomatis kirim RPM=0 (stop).
  - Watchdog di STM32 juga akan stop motor jika script mati mendadak.
"""
import argparse
import csv
import os
import serial
import struct
import time

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


def xor_checksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def build_command(rpm1: int, rpm2: int, rpm3: int) -> bytes:
    """Bangun 8-byte command packet: [0xA5, RPM1(2), RPM2(2), RPM3(2), XOR]."""
    rpm1 = max(-600, min(600, int(rpm1)))
    rpm2 = max(-600, min(600, int(rpm2)))
    rpm3 = max(-600, min(600, int(rpm3)))
    body = struct.pack('>Bhhh', PACKET_HEADER, rpm1, rpm2, rpm3)
    return body + bytes([xor_checksum(body)])


def parse_feedback(packet: bytes):
    """Parse 42-byte feedback → (t1, t2, t3, gx, gy, gz, ax, ay, az) or None."""
    if len(packet) != FEEDBACK_SIZE:
        return None
    if packet[0] != PACKET_HEADER:
        return None
    if xor_checksum(packet[:-1]) != packet[-1]:
        return None
    # Reject shifted-parse artifacts: if byte 1 == PACKET_HEADER, the parser
    # locked onto a checksum byte instead of the real header.  This occurs
    # when all data fields are zero and XOR checksum == 0xA5 == header byte.
    if packet[1] == PACKET_HEADER:
        return None
    ticks = struct.unpack('>iii', packet[1:13])
    gyro = struct.unpack('>hhh', packet[13:19])
    accel = struct.unpack('>hhh', packet[19:25])
    # Plausibility gate — physically impossible deltas indicate corruption.
    for t in ticks:
        if abs(t) > MAX_PLAUSIBLE_TICK_DELTA:
            return None
    return ticks[0], ticks[1], ticks[2], gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]


def extract_packets(buf: bytearray) -> list:
    """Extract semua feedback packet valid dari buffer."""
    packets = []
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
    """Konversi delta ticks dalam interval dt (detik) ke RPM.

    Rumus: RPM = delta_ticks × 60 / (TICKS_PER_REV × dt)
    Sama persis dengan yang dipakai di firmware (PID loop).
    """
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
    ap = argparse.ArgumentParser(
        description='RPM step response test — rekam data untuk PID tuning MATLAB')
    ap.add_argument('--port', default='/dev/ttyACM0',
                    help='Serial port (default: /dev/ttyACM0)')
    ap.add_argument('--baud', type=int, default=115200, help='Baud rate')
    ap.add_argument('--rpm', type=int, required=True,
                    help='Target RPM (positif = forward, negatif = reverse)')
    ap.add_argument('--duration', type=float, default=3.0,
                    help='Durasi test dalam detik (default: 3)')
    ap.add_argument('--motor', default='all',
                    help='Motor yang ditest: 1, 2, 3, atau all (default: all)')
    ap.add_argument('--csv', default='',
                    help='Nama file CSV output (default: auto-generated)')
    args = ap.parse_args()

    # Tentukan RPM per motor
    if args.motor == 'all':
        cmd_rpm = [args.rpm, args.rpm, args.rpm]
        motor_label = 'all'
    elif args.motor in ('1', '2', '3'):
        idx = int(args.motor) - 1
        cmd_rpm = [0, 0, 0]
        cmd_rpm[idx] = args.rpm
        motor_label = f'M{args.motor}'
    else:
        print(f'Error: --motor harus 1, 2, 3, atau all (dapat: {args.motor})')
        return

    # Nama CSV otomatis jika tidak di-specify
    csv_path = args.csv
    if not csv_path:
        csv_path = f'step_rpm{args.rpm}_{motor_label}_{int(args.duration)}s.csv'

    print(f'Port      : {args.port} @ {args.baud}')
    print(f'Command   : RPM [{cmd_rpm[0]}, {cmd_rpm[1]}, {cmd_rpm[2]}]')
    print(f'Durasi    : {args.duration}s')
    print(f'Output CSV: {csv_path}')
    print()

    ser = serial.Serial(
        args.port, args.baud, timeout=0.02,
        xonxoff=False, rtscts=False, dsrdtr=False,
    )
    ser.dtr = False
    ser.rts = False

    # Buang data lama, beri waktu STM32 settle
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Kirim RPM=0 dulu agar STM32 watchdog reset dan comm_healthy = true
    print('Warm-up: kirim RPM=0 selama 0.5s...')
    warmup_end = time.time() + 0.5
    while time.time() < warmup_end:
        ser.write(build_command(0, 0, 0))
        time.sleep(0.02)
    ser.reset_input_buffer()

    buf = bytearray()
    rows = []
    pkt_count = 0

    t_start = time.monotonic()
    t_last_cmd = 0.0
    t_last_fb = t_start  # timestamp feedback terakhir (untuk hitung dt)

    CMD_INTERVAL = 0.02  # Kirim command ~50Hz (sinkron dengan feedback rate)

    print(f'\n{"time_s":>7}  {"cmd1":>5}  {"cmd2":>5}  {"cmd3":>5}  '
          f'{"rpm1":>8}  {"rpm2":>8}  {"rpm3":>8}')
    print('-' * 60)

    try:
        while True:
            t_now = time.monotonic()
            elapsed = t_now - t_start

            if elapsed >= args.duration:
                break

            # Kirim command pada interval tetap
            if (t_now - t_last_cmd) >= CMD_INTERVAL:
                ser.write(build_command(*cmd_rpm))
                t_last_cmd = t_now

            # Baca feedback
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
                dt = t_fb - t_last_fb
                t_last_fb = t_fb

                # Hitung actual RPM dari delta ticks
                rpm1 = ticks_to_rpm(d1, dt)
                rpm2 = ticks_to_rpm(d2, dt)
                rpm3 = ticks_to_rpm(d3, dt)

                t_rel = t_fb - t_start
                rows.append([
                    round(t_rel, 4),
                    cmd_rpm[0], cmd_rpm[1], cmd_rpm[2],
                    round(rpm1, 2), round(rpm2, 2), round(rpm3, 2),
                ])
                pkt_count += 1

                # Print setiap ~10 paket (~5x/detik)
                if pkt_count % 10 == 0:
                    print(f'{t_rel:7.3f}  {cmd_rpm[0]:5d}  {cmd_rpm[1]:5d}  '
                          f'{cmd_rpm[2]:5d}  {rpm1:8.1f}  {rpm2:8.1f}  {rpm3:8.1f}')

    except KeyboardInterrupt:
        print('\n[Ctrl+C] Menghentikan test...')
    finally:
        # Selalu stop motor di akhir
        print('\nMengirim STOP (RPM=0)...')
        send_stop(ser)
        ser.close()

    # Simpan ke CSV
    if rows:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_s',
                'cmd_rpm1', 'cmd_rpm2', 'cmd_rpm3',
                'actual_rpm1', 'actual_rpm2', 'actual_rpm3',
            ])
            writer.writerows(rows)
        print(f'\nData tersimpan: {csv_path} ({len(rows)} samples)')
    else:
        print('\nTidak ada data yang terekam.')

    print(f'Total feedback packets: {pkt_count}')

    # Tips MATLAB
    print(f"""
=== Load di MATLAB ===
  T = readtable('{csv_path}');
  figure;
  plot(T.time_s, T.actual_rpm1, 'b', T.time_s, T.cmd_rpm1, 'r--');
  xlabel('Time (s)'); ylabel('RPM');
  legend('Actual', 'Command'); title('Step Response Motor 1');
  grid on;
""")


if __name__ == '__main__':
    main()
