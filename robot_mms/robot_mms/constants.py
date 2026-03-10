#!/usr/bin/env python3
"""
constants.py — Single source of truth untuk semua konstanta robot LKS.

JANGAN hardcode angka-angka ini di tempat lain — import dari sini.

Referensi:
    - PROJECT.md section 2.3, 2.4, 3.4
    - STM32_Firmware_Robot/include/control_config.h
    - STM32_Firmware_Robot/include/serial_protocol.h
    - mms_params.yaml
"""
import math

# =====================================================================
# Robot Geometry
# =====================================================================

WHEEL_RADIUS_M = 0.05
"""Radius roda omniwheel (m). Diameter 100 mm / 2."""

WHEEL_DIAMETER_M = 0.10
"""Diameter roda omniwheel (m)."""

WHEEL_BASE_RADIUS_M = 0.20
"""Jarak pusat robot ke titik kontak roda (m)."""

MOTOR_ANGLES_DEG = (180.0, 300.0, 60.0)
"""Sudut motor (M1, M2, M3) dalam derajat, CCW convention, 0° = depan robot."""

MOTOR_ANGLES_RAD = tuple(math.radians(a) for a in MOTOR_ANGLES_DEG)
"""Sudut motor (M1, M2, M3) dalam radian."""

# =====================================================================
# Encoder & Gearbox
# =====================================================================

ENCODER_PPR_RAW = 7
"""Pulse per revolution encoder motor (sebelum quadrature)."""

QUADRATURE_MULTIPLIER = 4
"""Multiplier quadrature decoding (4x)."""

GEAR_RATIO = 95.0 / 7.0
"""Rasio gearbox PG36: 95/7 = 13.571428... (exact fraction)."""

TICKS_PER_REV = 380.0
"""Encoder ticks per putaran output shaft: 7 × 4 × (95/7) = 380.0 exact."""

# =====================================================================
# Motor
# =====================================================================

MAX_RPM = 600
"""RPM maksimum motor (sebelum gearbox)."""

MAX_RPM_OUTPUT = MAX_RPM / GEAR_RATIO
"""RPM maksimum output shaft setelah gearbox: ~44.2 RPM."""

MAX_WHEEL_SPEED_MS = MAX_RPM_OUTPUT * math.pi * WHEEL_DIAMETER_M / 60.0
"""Kecepatan linear maksimum per roda (m/s): ~0.231 m/s."""

# =====================================================================
# PID Control (STM32 side — referensi saja, tidak dipakai di Python)
# =====================================================================

PID_FREQUENCY_HZ = 200
"""Frekuensi loop PID di STM32 (Hz)."""

PID_KP = 0.40
"""PID proportional gain (referensi dari control_config.h)."""

PID_KI = 3.60
"""PID integral gain (referensi dari control_config.h)."""

PID_KD = 0.04
"""PID derivative gain (referensi dari control_config.h)."""

# =====================================================================
# IMU
# =====================================================================

IMU_COMPLEMENTARY_ALPHA = 0.70
"""Bobot gyro pada complementary filter: 70% gyro / 30% accel."""

IMU_I2C_ADDR = 0x68
"""Alamat I2C MPU6050 (AD0 pin low)."""

# =====================================================================
# Serial Protocol (STM32 ↔ ROS2)
# =====================================================================

SERIAL_PORT = "/dev/ttyACM0"
"""Port serial default: ST-LINK VCP mem-bridge USART2 ke USB."""

SERIAL_BAUDRATE = 115200
"""Baudrate serial: 115200, 8N1, no flow control."""

SERIAL_FRAME_HEADER = 0xA5
"""Header byte untuk semua paket serial."""

SERIAL_CMD_PACKET_SIZE = 8
"""Ukuran paket command Pi→STM32: [0xA5][RPM1(2)][RPM2(2)][RPM3(2)][XOR] = 8 bytes."""

SERIAL_FEEDBACK_PACKET_SIZE = 42
"""Ukuran paket feedback STM32→Pi: 42 bytes (encoder+IMU+ultrasonic+XOR).
Updated from 26 to 42 bytes — ADDED(phase2-ultrasonic)."""

SERIAL_FEEDBACK_RATE_HZ = 50
"""Rate feedback dari STM32 (Hz). TaskSerial mengirim setiap 20 ms."""

STM32_WATCHDOG_TIMEOUT_MS = 250
"""Watchdog timeout di STM32 (ms). Motor stop jika tidak ada command."""

# =====================================================================
# Motor Driver Hardware
# =====================================================================

EN_MOTOR_PIN = "PB0"
"""Pin enable motor (shared semua BTS7960), active HIGH."""

MOTOR_TIMER_MAP = {
    1: {"pwm_timer": "TIM1", "pwm_channels": ("CH1", "CH2"), "encoder_timer": "TIM3"},
    2: {"pwm_timer": "TIM1", "pwm_channels": ("CH3", "CH4"), "encoder_timer": "TIM4"},
    3: {"pwm_timer": "TIM2", "pwm_channels": ("CH1", "CH2"), "encoder_timer": "TIM8"},
}
"""Mapping motor index → PWM timer/channel dan encoder timer (referensi hardware)."""

# =====================================================================
# Safety / Velocity Clamping
# =====================================================================

MAX_LINEAR_VELOCITY_MS = 2.0
"""Batas kecepatan linear yang diterima dari cmd_vel (m/s)."""

MAX_ANGULAR_VELOCITY_RADS = 10.0
"""Batas kecepatan angular yang diterima dari cmd_vel (rad/s)."""

WATCHDOG_TIMEOUT_S = 0.5
"""Watchdog timeout di ROS2 node (s). Motor stop jika cmd_vel berhenti."""

# =====================================================================
# Odometry
# =====================================================================

ODOM_K_XY = 0.01
"""Kovariansi translasi per meter perjalanan (m²/m)."""

ODOM_K_THETA = 0.02
"""Kovariansi rotasi per radian rotasi (rad²/rad)."""

# =====================================================================
# LKS Competition Parameters
# =====================================================================

ARENA_WIDTH_M = 2.40
"""Lebar arena LKS (m)."""

ARENA_LENGTH_M = 3.60
"""Panjang arena LKS (m)."""

CUBE_SIZE_MM = 50
"""Ukuran kubus (mm) — LKS standard object."""

CUBE_SIZE_M = CUBE_SIZE_MM / 1000.0
"""Ukuran kubus (m)."""

OBSTACLE_SAFE_DISTANCE_CM = 15
"""Jarak aman dari obstacle (cm) — minimum clearance."""

OBSTACLE_SAFE_DISTANCE_M = OBSTACLE_SAFE_DISTANCE_CM / 100.0
"""Jarak aman dari obstacle (m)."""

LINE_FOLLOW_THRESHOLD = 0.5
"""Threshold sensor garis digital (0.0–1.0, referensi kalibrasi)."""

# =====================================================================
# Ultrasonic Sensors — ADDED(phase2-ultrasonic)
# =====================================================================

ULTRASONIC_COUNT: int = 8
"""Jumlah sensor ultrasonik (2 per arah × 4 arah)."""

ULTRASONIC_SENSOR_NAMES: tuple = (
    "front_a", "front_b",
    "right_a", "right_b",
    "rear_a",  "rear_b",
    "left_a",  "left_b",
)
"""Nama sensor index 0-7, sesuai urutan di feedback packet."""

ULTRASONIC_DIRECTION_NAMES: tuple = ("front", "right", "rear", "left")
"""4 arah, masing-masing punya 2 sensor (a dan b)."""

ULTRASONIC_MIN_DIST_M: float = 0.020
"""Jarak minimum valid (m) — 20 mm."""

ULTRASONIC_MAX_DIST_M: float = 4.000
"""Jarak maksimum valid (m) — 4000 mm."""

ULTRASONIC_INVALID_MM: int = 0xFFFF
"""Sentinel value dari STM32 untuk sensor invalid/timeout."""

OBSTACLE_STOP_DIST_M: float = 0.075
"""Jarak berhenti obstacle (m) — Modul D2 LKS: 50-100 mm, gunakan tengah 75 mm."""

# =====================================================================
# Line Sensor — ADDED(phase2-ultrasonic)
# =====================================================================

LINE_SENSOR_COUNT: int = 2
"""Jumlah sensor garis digital."""

LINE_SENSOR_NAMES: tuple = ("D1", "D2")
"""Nama sensor garis, sesuai pin LINE_D1/LINE_D2 di STM32."""

LINE_STATE_OFF: int = 0
"""State sensor garis: tidak mendeteksi garis."""

LINE_STATE_ON: int = 1
"""State sensor garis: mendeteksi garis."""

# =====================================================================
# Balance (Wall Alignment) — ADDED(phase2-balance)
# =====================================================================

BALANCE_STOP_DIST_M: float = 0.050
"""
Jarak berhenti saat BALANCE — lebih dekat dari obstacle stop.
Robot benar-benar merapat ke dinding (50mm dari dinding).
Berbeda dari OBSTACLE_STOP_DIST_M (75mm) yang untuk safety.
"""

BALANCE_APPROACH_SPEED_M_S: float = 0.05
"""Kecepatan saat balance — pelan untuk akurasi dan safety."""

BALANCE_TIMEOUT_S: float = 8.0
"""Timeout balance — jika 8 detik tidak sampai, abort."""
