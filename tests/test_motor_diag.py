#!/usr/bin/env python3
"""
Motor Hardware Diagnostic — Isolate why motors don't spin.

Performs 3 tests in sequence:
  TEST 1: Serial link check  (is STM32 responding?)
  TEST 2: Encoder check      (spin wheel by hand → ticks visible?)
  TEST 3: Motor drive check   (send high RPM → does motor spin?)

Run:
  python3 test_motor_diag.py
  python3 test_motor_diag.py --port /dev/ttyACM0

After test, prints a diagnosis with next-step recommendations.
"""
import argparse
import serial
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Protocol constants (must match STM32 firmware)
# ---------------------------------------------------------------------------
PACKET_HEADER = 0xA5
CMD_SIZE = 8
FEEDBACK_SIZE = 42  # Updated: 26 (original) + 16 (8×uint16 ultrasonic)
TICKS_PER_REV = 380.0
KEEPALIVE_INTERVAL_S = 0.05
MAX_KEEPALIVE_GAP_S = 0.20


def xor_checksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def build_command(rpm1: int, rpm2: int, rpm3: int) -> bytes:
    rpm1 = max(-600, min(600, int(rpm1)))
    rpm2 = max(-600, min(600, int(rpm2)))
    rpm3 = max(-600, min(600, int(rpm3)))
    body = struct.pack('>Bhhh', PACKET_HEADER, rpm1, rpm2, rpm3)
    return body + bytes([xor_checksum(body)])


# Maximum plausible tick delta per 50Hz feedback (600 RPM * 380 t/r / 60 / 50 = 76).
# Anything beyond this is clearly corrupt data (parser misalignment, noise, etc.).
MAX_PLAUSIBLE_TICK_DELTA = 2000  # generous margin above theoretical 76


def parse_feedback(packet: bytes):
    if len(packet) != FEEDBACK_SIZE:
        return None
    if packet[0] != PACKET_HEADER:
        return None
    if xor_checksum(packet[:-1]) != packet[-1]:
        return None
    # Reject shifted-parse artifacts: if byte 1 == PACKET_HEADER, the parser
    # locked onto a checksum byte instead of the real header (happens when
    # all data fields are zero and checksum == 0xA5 == header).  See safety
    # note below for full explanation.
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


def read_feedback_for(ser, duration_s, send_rpm=None, cmd_interval=0.02):
    """Read feedback for 'duration_s'.  Optionally send a command at cmd_interval.
    Returns list of (time_rel, d1, d2, d3, gx, gy, gz, ax, ay, az) tuples."""
    buf = bytearray()
    results = []
    t0 = time.monotonic()
    t_last_cmd = 0.0

    while True:
        t_now = time.monotonic()
        elapsed = t_now - t0
        if elapsed >= duration_s:
            break

        # Send command if requested
        if send_rpm is not None and (t_now - t_last_cmd) >= cmd_interval:
            ser.write(build_command(*send_rpm))
            t_last_cmd = t_now

        # Read bytes
        waiting = ser.in_waiting
        if waiting > 0:
            buf.extend(ser.read(waiting))
        else:
            buf.extend(ser.read(1))

        for pkt in extract_packets(buf):
            result = parse_feedback(pkt)
            if result:
                results.append((time.monotonic() - t0,) + result)

    return results


def _phase_for_elapsed(elapsed_s: float, phase_plan: list[tuple[str, int, float]]) -> tuple[str, int]:
    """Map elapsed time to active phase label and RPM target.

    Args:
        elapsed_s: elapsed time from phase sequence start (seconds).
        phase_plan: list of (label, rpm_target, duration_s)

    Returns:
        Tuple (label, rpm_target) for current elapsed time.
    """
    acc_s = 0.0
    for label, rpm_target, dur_s in phase_plan:
        acc_s += dur_s
        if elapsed_s < acc_s:
            return label, rpm_target
    last_label, last_rpm, _ = phase_plan[-1]
    return last_label, last_rpm


def run_motor_drive_phases(
    ser,
    phase_plan: list[tuple[str, int, float]],
    cmd_interval_s: float = KEEPALIVE_INTERVAL_S,
):
    """Run motor-drive phases with continuous keepalive and feedback capture.

    Args:
        ser: open serial handle.
        phase_plan: list of (label, rpm_target, duration_s).
        cmd_interval_s: interval command keepalive in seconds.

    Returns:
        Tuple:
          - dict label -> list samples parsed in that phase.
          - dict keepalive stats {'sent', 'max_gap_s', 'max_gap_ms'}.
    """
    samples_by_phase = {label: [] for label, _, _ in phase_plan}
    total_duration_s = sum(dur_s for _, _, dur_s in phase_plan)

    buf = bytearray()
    t0 = time.monotonic()
    t_last_cmd = time.monotonic() - cmd_interval_s
    cmd_send_times: list[float] = []

    while True:
        t_now = time.monotonic()
        elapsed_s = t_now - t0
        if elapsed_s >= total_duration_s:
            break

        label, rpm_target = _phase_for_elapsed(elapsed_s, phase_plan)

        if (t_now - t_last_cmd) >= cmd_interval_s:
            ser.write(build_command(rpm_target, 0, 0))
            cmd_send_times.append(t_now)
            t_last_cmd = t_now

        waiting = ser.in_waiting
        if waiting > 0:
            buf.extend(ser.read(waiting))
        else:
            buf.extend(ser.read(1))

        for pkt in extract_packets(buf):
            parsed = parse_feedback(pkt)
            if parsed is None:
                continue
            sample_elapsed_s = time.monotonic() - t0
            sample_label, _ = _phase_for_elapsed(sample_elapsed_s, phase_plan)
            samples_by_phase[sample_label].append((sample_elapsed_s,) + parsed)

    max_gap_s = 0.0
    if len(cmd_send_times) >= 2:
        for i in range(1, len(cmd_send_times)):
            gap = cmd_send_times[i] - cmd_send_times[i - 1]
            if gap > max_gap_s:
                max_gap_s = gap

    keepalive_stats = {
        'sent': len(cmd_send_times),
        'max_gap_s': max_gap_s,
        'max_gap_ms': max_gap_s * 1000.0,
    }
    return samples_by_phase, keepalive_stats


def send_stop(ser):
    try:
        for _ in range(10):
            ser.write(build_command(0, 0, 0))
            time.sleep(0.02)
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description='Motor hardware diagnostic')
    ap.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    ap.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = ap.parse_args()

    print('=' * 65)
    print('  MOTOR HARDWARE DIAGNOSTIC')
    print('=' * 65)
    print(f'  Port: {args.port} @ {args.baud}')
    print()

    # Open serial
    try:
        ser = serial.Serial(
            args.port, args.baud, timeout=0.05,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        ser.dtr = False
        ser.rts = False
    except Exception as e:
        print(f'[FAIL] Cannot open serial port: {e}')
        return

    time.sleep(0.3)
    ser.reset_input_buffer()

    # ==================================================================
    # TEST 1: Serial communication
    # ==================================================================
    ser.reset_input_buffer()   # flush stale bytes before each phase
    print('-' * 65)
    print('TEST 1: Serial Communication')
    print('  Listening for feedback packets (2 seconds, no commands)...')
    print()

    results = read_feedback_for(ser, duration_s=2.0, send_rpm=None)
    pkt_count = len(results)

    if pkt_count == 0:
        print(f'  [FAIL] No feedback packets received in 2 seconds!')
        print('  → STM32 is not sending data. Check:')
        print('    1. Is firmware flashed?')
        print('    2. Is USB cable connected?')
        print('    3. Is STM32 powered?')
        ser.close()
        return
    else:
        rate = pkt_count / 2.0
        print(f'  [OK] Received {pkt_count} packets ({rate:.0f} Hz)')
        if abs(rate - 50) < 15:
            print('  → Feedback rate matches expected 50 Hz ✓')
        else:
            print(f'  → WARNING: Expected ~50 Hz, got {rate:.0f} Hz')

    # Check if any ticks are non-zero (from hand spinning or noise)
    total_ticks = [0, 0, 0]
    for _, d1, d2, d3, _, _, _, _, _, _ in results:
        total_ticks[0] += abs(d1)
        total_ticks[1] += abs(d2)
        total_ticks[2] += abs(d3)

    print(f'  Tick activity (no commands): '
          f'Enc1={total_ticks[0]}  Enc2={total_ticks[1]}  Enc3={total_ticks[2]}')
    print()

    # ==================================================================
    # TEST 2: Encoder check (interactive)
    # ==================================================================
    ser.reset_input_buffer()   # flush stale bytes before each phase
    print('-' * 65)
    print('TEST 2: Encoder Feedback (Manual Wheel Spin)')
    print('  Sending RPM=0 so comm_healthy=true (enables tick accumulation)')
    print()
    print('  >>> SPIN EACH WHEEL BY HAND for the next 5 seconds <<<')
    print()

    # Send RPM=0 to keep comm_healthy = true (so PID task accumulates ticks)
    results = read_feedback_for(ser, duration_s=5.0, send_rpm=[0, 0, 0])

    total_ticks = [0, 0, 0]
    for _, d1, d2, d3, _, _, _, _, _, _ in results:
        total_ticks[0] += abs(d1)
        total_ticks[1] += abs(d2)
        total_ticks[2] += abs(d3)

    print(f'  Tick totals: Enc1={total_ticks[0]}  Enc2={total_ticks[1]}  Enc3={total_ticks[2]}')

    enc_ok = [t > 10 for t in total_ticks]
    any_enc_ok = any(enc_ok)

    if any_enc_ok:
        for i in range(3):
            status = '[OK]' if enc_ok[i] else '[---]'
            print(f'  {status} Encoder {i+1}: {total_ticks[i]} ticks')
        print()
        print('  → Encoders are working! PID task is running. ✓')
    else:
        print()
        print('  [WARN] No encoder ticks detected.')
        print('  Possible causes:')
        print('    a) You didn\'t spin any wheel (try again)')
        print('    b) GPIO pull-ups not flashed (need to reflash firmware)')
        print('    c) Encoder wiring issue')
        print('    d) PID task not running (TIM6 issue)')
    print()

    # ==================================================================
    # TEST 3: Motor drive at increasing RPM
    # ==================================================================
    print('-' * 65)
    print('TEST 3: Motor Drive (Automatic)')
    print('  Will send increasing RPM to Motor 1:')
    print('    Phase A: RPM=100 for 2s')
    print('    Phase B: RPM=300 for 2s')
    print('    Phase C: RPM=500 for 2s')
    print()
    print('  >>> WATCH MOTOR 1 — Does it spin? <<<')
    print()

    # Warm-up: establish comm_healthy and flush stale bytes
    warmup_end = time.time() + 0.5
    while time.time() < warmup_end:
        ser.write(build_command(0, 0, 0))
        time.sleep(0.02)
    ser.reset_input_buffer()   # flush stale bytes before motor drive phase

    motor_moved = False
    phase_results = {}

    phase_plan = [
        ('A', 100, 2.0),
        ('B', 300, 2.0),
        ('C', 500, 2.0),
    ]

    print('  Running A→B→C with CONTINUOUS keepalive at 50ms interval...')
    all_phase_samples, keepalive_stats = run_motor_drive_phases(
        ser,
        phase_plan,
        cmd_interval_s=KEEPALIVE_INTERVAL_S,
    )

    for label, rpm_target, _ in phase_plan:
        print(f'  Phase {label}: Sending RPM={rpm_target} to Motor 1...')

        results = all_phase_samples[label]

        # Analyze ticks from motor 1
        total_abs = 0
        max_rpm_seen = 0.0
        for _, d1, d2, d3, _, _, _, _, _, _ in results:
            total_abs += abs(d1)
            dt = 0.02  # approximate 50 Hz
            rpm = abs(d1) * 60.0 / (TICKS_PER_REV * dt) if dt > 0 else 0
            if rpm > max_rpm_seen:
                max_rpm_seen = rpm

        phase_results[label] = {
            'rpm': rpm_target,
            'total_ticks': total_abs,
            'max_rpm': max_rpm_seen,
            'samples': len(results),
        }

        status = '[OK]' if total_abs > 5 else '[---]'
        print(f'    {status} Ticks={total_abs}  MaxRPM≈{max_rpm_seen:.0f}  '
              f'(target={rpm_target}, {len(results)} samples)')

        if total_abs > 5:
            motor_moved = True

    print()
    print(f"  Keepalive packets sent: {keepalive_stats['sent']}")
    print(f"  Max keepalive gap: {keepalive_stats['max_gap_ms']:.1f} ms")
    if keepalive_stats['max_gap_s'] > MAX_KEEPALIVE_GAP_S:
        print(f'  [WARN] Max gap > {int(MAX_KEEPALIVE_GAP_S * 1000)} ms (watchdog risk)')
    else:
        print(f'  [OK] Max gap <= {int(MAX_KEEPALIVE_GAP_S * 1000)} ms')

    print()

    # Stop motors
    print('  Stopping motors...')
    send_stop(ser)

    # ==================================================================
    # DIAGNOSIS
    # ==================================================================
    print()
    print('=' * 65)
    print('  DIAGNOSIS')
    print('=' * 65)
    print()

    if motor_moved:
        print('  ✓ Motor 1 is WORKING!')
        print()
        print('  If RPM=50 didn\'t work earlier, the motor\'s static')
        print('  friction + gearbox require higher starting RPM.')
        print('  → Increase feedforward (CONTROL_FF_OFFSET) in firmware')
        print('  → Or test with higher RPM values')
    else:
        if any_enc_ok:
            print('  RESULT: Encoders work but motor does NOT spin.')
            print()
            print('  This means:')
            print('    ✓ STM32 firmware is running')
            print('    ✓ PID task running (accumulates encoder ticks)')
            print('    ✓ Serial communication works')
            print('    ✗ Motor output chain is broken')
            print()
            print('  Most likely HARDWARE issues (check in order):')
            print()
            print('  1. BTS7960 MOTOR POWER (VM / B+)')
            print('     → Is 12V/24V power supply connected to BTS7960?')
            print('     → Is power supply turned ON?')
            print('     → Check with multimeter on B+ and GND of BTS7960')
            print()
            print('  2. EN_MOTOR wiring (PB0 → BTS7960)')
            print('     → PB0 must connect to BOTH L_EN AND R_EN on BTS7960')
            print('     → With motor running, check PB0 voltage (should be 3.3V)')
            print('     → Some BTS7960 modules have EN pins tied HIGH internally')
            print()
            print('  3. PWM wiring (STM32 → BTS7960)')
            print('     → PA8 (M1_RPWM) → BTS7960 Motor1 R_PWM')
            print('     → PA9 (M1_LPWM) → BTS7960 Motor1 L_PWM')
            print('     → Check with oscilloscope or logic analyzer')
            print()
            print('  4. Motor wiring (BTS7960 → Motor)')
            print('     → M+ and M− from BTS7960 to motor terminals')
            print()
            print('  FIRMWARE DEBUG OPTION:')
            print('     Flash firmware with boot motor self-test added')
            print('     to main.c — motor should twitch at power-on.')
            print('     If it doesn\'t, it\'s definitely hardware.')
        else:
            print('  RESULT: No encoder ticks AND motor does NOT spin.')
            print()
            print('  Possible scenarios:')
            print()
            print('  a) Firmware not flashed with latest changes')
            print('     → Did you run: cd ~/STM32_Firmware_Robot && pio run -t upload ?')
            print('     → The GPIO_PULLUP encoder fix requires reflashing!')
            print()
            print('  b) PID task not running (TIM6 issue)')
            print('     → Flash the firmware with boot motor self-test')
            print('     → This tests hardware independently of RTOS')
            print()
            print('  c) Multiple hardware issues (encoder + motor)')
            print('     → Check all wiring with multimeter')

    print()
    print('=' * 65)
    ser.close()


if __name__ == '__main__':
    main()
