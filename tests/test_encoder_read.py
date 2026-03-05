#!/usr/bin/env python3
"""
Encoder Read Test — Baca ticks dari 3 encoder via feedback STM32.

Cara pakai:
  1. Jalankan script ini (STM32 harus sudah nyala dan UART terhubung)
  2. Putar roda secara manual
  3. Tekan Ctrl+C → tampilkan total ticks

Tidak mengirim command apapun — motor tetap mati (aman untuk putar tangan).
STM32 tetap streaming feedback 50Hz tanpa perlu command.

Tujuan: verifikasi ticks_per_rev.
  Putar 1 roda tepat 1 putaran penuh → total ticks harusnya ~380.
"""
import argparse
import serial
import struct
import time

# ---------------------------------------------------------------------------
# Protokol serial (identik dengan firmware STM32 & ESP32)
# ---------------------------------------------------------------------------
PACKET_HEADER = 0xA5
FEEDBACK_SIZE = 26  # header(1) + 3×int32(12) + 3×int16 gyro(6) + 3×int16 accel(6) + xor(1)


def xor_checksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def parse_feedback(packet: bytes):
    """Parse 26-byte feedback → (t1, t2, t3, gx, gy, gz, ax, ay, az) or None."""
    if len(packet) != FEEDBACK_SIZE:
        return None
    if packet[0] != PACKET_HEADER:
        return None
    if xor_checksum(packet[:-1]) != packet[-1]:
        return None
    ticks = struct.unpack('>iii', packet[1:13])
    gyro = struct.unpack('>hhh', packet[13:19])
    accel = struct.unpack('>hhh', packet[19:25])
    return ticks[0], ticks[1], ticks[2], gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]


def extract_packets(buf: bytearray) -> list:
    """Extract semua feedback packet valid dari buffer, sisakan sisa."""
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
            # Checksum gagal — buang 1 byte, cari header berikutnya
            del buf[:1]
    return packets


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description='Baca encoder ticks dari STM32 (putar roda manual)')
    ap.add_argument('--port', default='/dev/ttyACM0', help='Serial port (default: /dev/ttyACM0)')
    ap.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = ap.parse_args()

    print(f'Membuka {args.port} @ {args.baud}...')
    ser = serial.Serial(
        args.port, args.baud, timeout=0.05,
        xonxoff=False, rtscts=False, dsrdtr=False,
    )
    ser.dtr = False
    ser.rts = False

    # Buang data sisa di buffer agar mulai dari state bersih
    time.sleep(0.1)
    ser.reset_input_buffer()

    total = [0, 0, 0]
    buf = bytearray()
    pkt_count = 0

    print('\nPutar roda secara manual. Tekan Ctrl+C untuk stop.\n')
    print(f'{"#":>6}  {"δ1":>7}  {"δ2":>7}  {"δ3":>7}  '
          f'{"Total1":>8}  {"Total2":>8}  {"Total3":>8}')
    print('-' * 68)

    try:
        while True:
            waiting = ser.in_waiting
            if waiting > 0:
                buf.extend(ser.read(waiting))
            else:
                # Blocking read 1 byte agar tidak busy-spin
                buf.extend(ser.read(1))

            for pkt in extract_packets(buf):
                result = parse_feedback(pkt)
                if result is None:
                    continue

                d1, d2, d3, _gx, _gy, _gz, _ax, _ay, _az = result
                total[0] += d1
                total[1] += d2
                total[2] += d3
                pkt_count += 1

                # Print setiap ~10 paket (~5x/detik) agar tidak flooding terminal
                if pkt_count % 10 == 0:
                    print(f'{pkt_count:6d}  {d1:7d}  {d2:7d}  {d3:7d}  '
                          f'{total[0]:8d}  {total[1]:8d}  {total[2]:8d}')

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    print('\n' + '=' * 68)
    print('HASIL AKHIR')
    print(f'  Encoder 1 total ticks : {total[0]}')
    print(f'  Encoder 2 total ticks : {total[1]}')
    print(f'  Encoder 3 total ticks : {total[2]}')
    print(f'  Total packets diterima: {pkt_count}')
    print(f'\n  Referensi: ~380 ticks = 1 putaran penuh (full quadrature)')
    print('=' * 68)


if __name__ == '__main__':
    main()
