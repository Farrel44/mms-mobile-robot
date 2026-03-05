# 📖 DOKUMENTASI LENGKAP — Robot Omni-Wheel 3 Roda (Kiwi Drive)

> **Proyek:** Robot Mobile LKS 2026 — Kompetisi Mobile Robotics
> **Terakhir diperbarui:** 2026-03-05
> **Repositori:** 3 repo — `mms-mobile-robot` (ROS2), `STM32_Firmware_Robot` (firmware), `MMS_ROBOTIC_ESP` (legacy)

---

## Daftar Isi

1. [Gambaran Umum Sistem](#1-gambaran-umum-sistem)
2. [Arsitektur Hardware](#2-arsitektur-hardware)
3. [Arsitektur Software](#3-arsitektur-software)
4. [STM32 Firmware (FreeRTOS)](#4-stm32-firmware-freertos)
5. [Protokol Serial (Pi ↔ STM32)](#5-protokol-serial-pi--stm32)
6. [ROS2 — robot_mms (Driver Layer)](#6-ros2--robot_mms-driver-layer)
7. [ROS2 — robot_mission (Sequencer Layer)](#7-ros2--robot_mission-sequencer-layer)
8. [Kinematika Kiwi Drive](#8-kinematika-kiwi-drive)
9. [PID & Motor Control Pipeline](#9-pid--motor-control-pipeline)
10. [IMU & Sensor Fusion](#10-imu--sensor-fusion)
11. [Odometry](#11-odometry)
12. [Foxglove Integration](#12-foxglove-integration)
13. [Konfigurasi Parameter](#13-konfigurasi-parameter)
14. [ROS2 Topics, Services & Frames](#14-ros2-topics-services--frames)
15. [Safety & Watchdog](#15-safety--watchdog)
16. [Build & Deploy](#16-build--deploy)
17. [Pin Mapping & Wiring](#17-pin-mapping--wiring)
18. [Legacy ESP32](#18-legacy-esp32)
19. [Test Infrastructure](#19-test-infrastructure)
20. [Troubleshooting](#20-troubleshooting)

---

## 1. Gambaran Umum Sistem

### Apa ini?

Robot omni-directional 3 roda (kiwi drive) untuk kompetisi LKS Mobile Robotics. Robot bisa bergerak ke segala arah (maju, mundur, geser kiri/kanan, diagonal, putar) tanpa harus belok dulu.

### Arsitektur: Indexed Runtime Command Sequencer

```
┌─────────────────────────────────────────────────────────────────┐
│                    Foxglove Studio (Laptop)                     │
│              Editor tabel step + monitoring                     │
│     Kirim: /mission/control (JSON)  ← WebSocket 8765            │
│     Terima: /mission/status, /odom, /imu, /diagnostics          │
└────────────────────────────┬────────────────────────────────────┘
                             │ WiFi / Ethernet
┌────────────────────────────▼────────────────────────────────────┐
│                    Raspberry Pi 4 (ROS2 Humble)                 │
│                                                                 │
│  ┌──────────────────────┐   ┌────────────────────────────────┐  │
│  │   mission_sequencer  │──▶│         mms_bridge             │  │
│  │   (robot_mission)    │   │        (robot_mms)             │  │
│  │                      │   │                                │  │
│  │ State machine:       │   │ /cmd_vel → inverse kin → RPM   │  │
│  │ IDLE→RUNNING→DONE    │   │ serial RX → forward kin → odom │  │
│  │ /mission/control sub │   │ /odom, /imu, /tf pub           │  │
│  │ /cmd_vel pub         │   │ /diagnostics pub (1 Hz)        │  │
│  └──────────────────────┘   └──────────┬─────────────────────┘  │
└─────────────────────────────────────────┼───────────────────────┘
                                          │ USB Serial (115200 baud)
                                          │ /dev/ttyACM0
┌─────────────────────────────────────────▼───────────────────────┐
│               STM32F446RE Nucleo (FreeRTOS)                     │
│                                                                 │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────────────────┐  │
│  │ TaskPid │  │TaskSerial│  │ TaskImu │  │  UART ISR (DMA)  │  │
│  │ 200 Hz  │  │ 50 Hz   │  │ 100 Hz  │  │  Parse commands  │  │
│  │ PID+FF  │  │ FB TX   │  │ MPU6050 │  │  → target_rpm    │  │
│  │ Encoders│  │ Watchdog │  │ Comp.fil│  └──────────────────┘  │
│  └────┬────┘  └────┬────┘  └────┬────┘                         │
│       │            │            │                               │
│  ┌────▼────────────▼────────────▼──────────────────────────┐   │
│  │  Hardware: 3× BTS7960 + 3× Encoder + MPU6050 (I2C)     │   │
│  │  Motor 1/2: TIM1 PWM (advanced timer, MOE)              │   │
│  │  Motor 3:   TIM2 PWM (general purpose)                  │   │
│  │  Encoder:   TIM3/TIM4/TIM8 (quadrature 4x)             │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Alur Data (Satu Siklus Penuh)

```
1. Foxglove → /mission/control JSON {"cmd":"RUN"}
2. mission_sequencer membaca step tabel → generate /cmd_vel (25 Hz)
3. mms_bridge terima /cmd_vel → inverse kinematics → RPM → serial TX (8 byte)
4. STM32 UART ISR terima RPM → target_rpm (volatile)
5. TaskPid (200 Hz) baca encoder → PID compute → PWM → motor bergerak
6. TaskSerial (50 Hz) baca tick_accum + IMU → feedback packet (26 byte) → serial TX
7. mms_bridge terima feedback → forward kinematics → odometry
8. Publish /odom, /imu/data, /tf, /encoder_ticks
9. mission_sequencer baca /odom → cek exit condition → lanjut step / selesai
10. Foxglove menampilkan posisi, status, telemetry real-time
```

---

## 2. Arsitektur Hardware

### Komponen Utama

| Komponen | Model | Fungsi |
|----------|-------|--------|
| SBC | Raspberry Pi 4 (4GB) | Otak utama, ROS2 Humble |
| MCU | STM32F446RE (Nucleo-64) | Motor control real-time, FreeRTOS |
| Motor Driver | 3× BTS7960 | H-bridge dual PWM (RPWM/LPWM) |
| Motor + Encoder | 3× DC motor + quadrature encoder | Penggerak + feedback kecepatan |
| IMU | MPU6050 | Gyroscope + accelerometer (I2C) |
| Roda | 3× omni-wheel | Konfigurasi kiwi drive 120° |
| Baterai | 3S LiPo (11.1V nominal) | Power motor + elektronik |
| Koneksi Pi↔STM32 | USB (ST-LINK VCP) | Serial 115200 baud |

### Geometri Robot

```
        Motor 1 (240°)
           ⟲
          / \
         /   \
        /     \
       /  0.20m \     ← robot_radius (pusat ke roda)
      /    ●     \
     /   center   \
    /               \
Motor 2 (120°) ---- Motor 3 (0°)
     ⟲                 ⟲

Sudut roda (dari sumbu X positif):
  Motor 1: 240° (belakang kiri)
  Motor 2: 120° (belakang kanan)
  Motor 3:   0° (depan)

wheel_radius = 0.05 m (50 mm)
robot_radius = 0.20 m (200 mm) — jarak pusat ke roda
```

---

## 3. Arsitektur Software

### 3 Repository

| Repo | Platform | Bahasa | Fungsi |
|------|----------|--------|--------|
| `mms-mobile-robot/` | Raspberry Pi | Python 3 | ROS2 nodes (driver + sequencer) |
| `STM32_Firmware_Robot/` | STM32F446RE | C | FreeRTOS firmware (PID, serial, IMU) |
| `MMS_ROBOTIC_ESP/` | ESP32 | C++ (Arduino) | Legacy firmware (sudah diganti STM32) |

### Struktur `mms-mobile-robot/`

```
mms-mobile-robot/
├── robot_mms/                    # Paket driver low-level [OTOT]
│   ├── robot_mms/
│   │   ├── __init__.py           # Export kinematics module
│   │   ├── mms_bridge.py         # Node utama: serial bridge + odom
│   │   └── kinematics.py         # Rumus kinematika kiwi 3 roda
│   ├── config/
│   │   └── mms_params.yaml       # Semua parameter robot
│   ├── package.xml               # Dependency ROS2
│   ├── setup.py                  # Entry point: mms_bridge
│   └── setup.cfg
│
├── robot_mission/                # Paket sequencer high-level [OTAK]
│   ├── robot_mission/
│   │   └── mission_sequencer.py  # State machine + step executor
│   ├── config/
│   │   └── sequencer_params.yaml # Parameter sequencer
│   ├── package.xml
│   └── setup.py
│
├── tests/                        # Unit tests + hardware tests
│   ├── test_bug_fixes_phase1.py  # 19 tests (BUG 1-3)
│   ├── test_bug_fixes_phase2.py  # 42 tests (BUG 4-5)
│   ├── test_rec1_rec2_phase3.py  # 50 tests (REC 1-2)
│   ├── test_rec3_rec4_phase4.py  # 49 tests (REC 3-4)
│   ├── test_rec5_rec6_phase5.py  # 85 tests (REC 5-6)
│   ├── hw_test_suite1_*.py       # Hardware: bug regression
│   ├── hw_test_suite2_*.py       # Hardware: hardening validation
│   ├── hw_test_suite3_*.py       # Hardware: end-to-end
│   ├── TEST_PROTOCOL.md          # Protokol pengujian
│   ├── test_encoder_read.py      # Tool: baca encoder manual
│   ├── test_motor_diag.py        # Tool: diagnosa motor
│   └── test_rpm_step.py          # Tool: step response RPM
│
├── docs/
│   └── FOXGLOVE_PANELS.md        # Panduan setup Foxglove
├── DEPLOYMENT_CHECKLIST.md       # Checklist deployment
└── README.md
```

### Struktur `STM32_Firmware_Robot/`

```
STM32_Firmware_Robot/
├── platformio.ini                # Build config (nucleo_f446re, stm32cube)
├── include/
│   ├── app_freertos.h            # Task handle + thread flag
│   ├── control_config.h          # SEMUA tuning parameter (PID, encoder, FF)
│   ├── encoder_quad.h            # Fungsi konfigurasi encoder 4x
│   ├── motor_pwm.h               # API motor PWM (BTS7960)
│   ├── mpu6050.h                 # Driver MPU6050 (I2C)
│   ├── pid.h                     # Struct & API PID controller
│   ├── serial_protocol.h         # Definisi paket + parser
│   ├── FreeRTOSConfig.h          # Konfigurasi FreeRTOS
│   ├── main.h                    # Pin definitions (CubeMX)
│   └── tim.h / usart.h / ...     # HAL peripheral headers
├── src/
│   ├── app_freertos.c            # ★ TASK UTAMA: PID, Serial, IMU
│   ├── motor_pwm.c               # PWM output (TIM1 MOE, TIM2)
│   ├── encoder_quad.c            # Encoder 4x quadrature config
│   ├── mpu6050.c                 # IMU driver (raw read + convert)
│   ├── pid.c                     # PID compute (Kp, Ki, Kd + clamp)
│   ├── serial_protocol.c         # Packet builder + parser
│   ├── main.c                    # HAL init → FreeRTOS start
│   └── tim.c / usart.c / ...     # HAL peripheral init
├── scripts/
│   └── pio_linkflags.py          # Linker flags (FPU hard float)
└── lib/
    └── FreeRTOS/                 # FreeRTOS kernel source
```

---

## 4. STM32 Firmware (FreeRTOS)

### Overview

MCU STM32F446RE @ 180 MHz menjalankan FreeRTOS dengan 4 task:

| Task | Prioritas | Rate | Stack | Fungsi |
|------|-----------|------|-------|--------|
| `TaskPid` | High | 200 Hz | 1024 B | Baca encoder → PID → PWM motor |
| `TaskSerial` | Normal | 50 Hz | 1024 B | Kirim feedback + watchdog |
| `TaskImu` | Below Normal | 100 Hz | 1536 B | Baca MPU6050 + complementary filter |
| `TaskUltrasonic` | Low | 20 Hz | 768 B | Placeholder (belum diimplementasi) |

### TaskPid — Motor Control Loop (200 Hz)

Ini adalah task paling penting. Berjalan di 200 Hz, dipicu oleh TIM6 interrupt.

**Pipeline per tick:**

```
┌─────────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  Encoder    │───▶│  RPM     │───▶│  Ramp    │───▶│  FF+PID  │───▶│  PWM     │
│  Read Δtick │    │  (EMA)   │    │  Limit   │    │  Compute │    │  Output  │
│  TIM3/4/8   │    │  α=0.30  │    │  2.5/4.0 │    │  ±255    │    │  Motor   │
└─────────────┘    └──────────┘    └──────────┘    └──────────┘    └──────────┘
```

**Detail pipeline:**

1. **Encoder Read**: Baca counter TIM3/4/8 (16-bit), hitung delta via subtraction `(now - prev)`. Cast ke `int16_t` → handles wrap otomatis selama `|delta| < 32768`.

2. **Tick Accumulation**: Delta ticks ditambahkan ke `tick_accum` (volatile int32). TaskSerial membaca+reset ini setiap 20 ms (50 Hz).

3. **RPM Calculation**: `rpm = delta_ticks × 60 / (380 × 0.005)`

4. **EMA Filter**: `rpm_filtered = 0.30 × rpm + 0.70 × rpm_filtered_prev`
   Mengurangi noise derivatif pada PID.

5. **Ramp Limiter**: Target RPM di-ramp pelan:
   - Naik: +2.5 RPM/tick (= +500 RPM/s)
   - Turun: -4.0 RPM/tick (= -800 RPM/s)
   - Instant snap pada reversal (ganti arah)

6. **Feedforward**: `FF = sign × (OFFSET + SLOPE × |RPM|)` = `sign × (5.0 + 0.3 × |RPM|)`
   Kompensasi deadzone motor.

7. **PID Compute**: `trim = Kp×error + Ki×∫error + Kd×d(error)/dt`
   - Kp = 0.40, Ki = 3.60, Kd = 0.04
   - Anti-windup: integral clamp ±50
   - Dead-zone reset: integral di-reset jika target < 2 RPM

8. **PWM Output**: `pwm = clamp(FF + PID, -255, 255)` → rate-limit ±2/tick → `MotorPwm_Set()`

9. **Safety**: Jika `comm_healthy == false` atau semua target ≈ 0 → PWM=0, reset semua state (ramp, EMA, PID integral).

### TaskSerial — Feedback TX & Watchdog (50 Hz)

```
Setiap 20 ms:
1. Cek watchdog: jika (now - last_valid_cmd) > 250 ms → comm_healthy = false, target = 0
2. Critical section: snapshot tick_accum → reset ke 0
3. Bangun FeedbackPacket: {tick1, tick2, tick3, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z}
4. Serialize ke 26 byte → HAL_UART_Transmit
```

### TaskImu — MPU6050 Polling (100 Hz)

```
Setiap 10 ms:
1. Baca 14 byte raw data via I2C (accel xyz + gyro xyz)
2. Convert ke physical units (rad/s, m/s²)
3. EMA filter (α=0.70) on all 3 gyro axes:
   - gyro_{x,y,z}_filtered = α × prev + (1-α) × raw
4. Accel passed through unfiltered (hardware DLPF at 44 Hz is sufficient)
5. Convert gyro to milli-rad/s (×1000), accel to milli-m/s² (×1000) → int16
6. Write all 6 values under taskENTER_CRITICAL/taskEXIT_CRITICAL
7. Auto-disable jika >10 consecutive I2C errors
```

### UART ISR — Command Reception (Interrupt-Driven)

```
DMA receive-to-idle pada USART2:
1. Byte masuk ke buffer DMA (64 byte)
2. Idle line detected → ISR callback
3. Parse command packet (8 byte): header + 3×RPM + checksum
4. Jika valid: update target_rpm1/2/3 (volatile), refresh watchdog timestamp
5. Re-arm DMA reception
```

### Inisialisasi (Boot Sequence)

```
main.c:
  HAL_Init() → SystemClock_Config(180MHz) → GPIO/TIM/USART/I2C init → FreeRTOS start

app_freertos.c MX_FREERTOS_Init():
  1. Create TaskPid (High priority)
  2. Create TaskSerial (Normal)
  3. Create TaskImu (Below Normal)
  4. Create TaskUltrasonic (Low)

TaskPid init:
  1. PID_Init × 3 motors (Kp=0.40, Ki=3.60, Kd=0.04)
  2. EncoderQuad_ConfigureFullQuad (TIM3/4/8 → 4x mode)
  3. HAL_TIM_Encoder_Start × 3
  4. MotorPwm_Init (PWM start + TIM1 MOE enable + EN HIGH)
  5. HAL_TIM_Base_Start_IT(TIM6) → 200 Hz tick source
  6. Loop: wait for PID_TICK_FLAG → process
```

---

## 5. Protokol Serial (Pi ↔ STM32)

### Spesifikasi Fisik

| Parameter | Nilai |
|-----------|-------|
| Interface | USB (ST-LINK VCP → USART2) |
| Device di Pi | `/dev/ttyACM0` |
| Baud rate | 115200 |
| Format | 8N1 (8 data, no parity, 1 stop) |
| Flow control | None |

### Command Packet (Pi → STM32): 8 byte

```
Byte:  [ 0xA5 | RPM1_H | RPM1_L | RPM2_H | RPM2_L | RPM3_H | RPM3_L | XOR ]
Index: [  0   |   1    |   2    |   3    |   4    |   5    |   6    |  7  ]

- 0xA5: header byte (magic number)
- RPM1/2/3: int16 big-endian, range ±600
- XOR: XOR checksum byte[0..6]
```

### Feedback Packet (STM32 → Pi): 26 byte

```
Byte: [ 0xA5 | T1(4B) | T2(4B) | T3(4B) | GX(2B) | GY(2B) | GZ(2B) | AX(2B) | AY(2B) | AZ(2B) | XOR ]
Index:[  0   | 1-4    | 5-8    | 9-12   | 13-14  | 15-16  | 17-18  | 19-20  | 21-22  | 23-24  | 25  ]

- 0xA5: header byte
- T1/T2/T3: int32 big-endian → delta ticks (accumulated sejak feedback terakhir)
- GX/GY/GZ: int16 big-endian → EMA-filtered gyro × 1000 (milli-rad/s)
- AX/AY/AZ: int16 big-endian → raw accel × 1000 (milli-m/s²)
- XOR: XOR checksum byte[0..24]
```

### Checksum

```python
def xor_checksum(data):
    result = 0
    for byte in data:
        result ^= byte
    return result & 0xFF
```

### Sinkronisasi

STM32 langsung beroperasi dalam mode biner sejak boot (tidak perlu handshake seperti ESP32 yang butuh `'b'`). Pi cukup:
1. Tunggu 0.5s (boot STM32)
2. Flush input buffer
3. Cari paket feedback valid pertama (scan 0xA5 + verify checksum)

### Rate

| Arah | Rate | Size | Bandwidth |
|------|------|------|-----------|
| Pi → STM32 (command) | ~50 Hz (driven by cmd_vel) | 8 B | 400 B/s |
| STM32 → Pi (feedback) | 50 Hz (fixed) | 18 B | 900 B/s |

---

## 6. ROS2 — robot_mms (Driver Layer)

### Node: `motor_bridge_node` (mms_bridge.py, ~960 baris)

Node utama yang menjembatani ROS2 dengan hardware STM32.

### Fungsi Utama

#### 1. Command Path: `/cmd_vel` → Motor

```
/cmd_vel (Twist) diterima
  ↓
Rec 2: Reject NaN/Inf
  ↓
Rec 2: Clamp velocity ke max_linear/angular
  ↓
Inverse kinematics: (vx, vy, wz) → (v1, v2, v3) m/s
  ↓
velocity_to_rpm: v → RPM = v × 60 / (2π × wheel_radius)
  ↓
Warn jika RPM > 600 (saturation)
  ↓
build_command_packet: [0xA5, RPM1, RPM2, RPM3, XOR] → 8 byte
  ↓
serial.write() → STM32
```

#### 2. Feedback Path: Serial RX → Odometry

```
Timer 50 Hz: read serial bytes
  ↓
Feed ke rx_buf (bytearray)
  ↓
_extract_feedback_packets: scan 0xA5 → verify checksum → list[packet]
  ↓
parse_feedback_packet: unpack int32×3 + int16×3 + int16×3 → (delta_ticks, gyro_xyz, accel_xyz)
  ↓
_integrate_odometry: forward kinematics → pose (x, y, yaw) update
  ↓
_publish_all: /odom + /tf + /imu/data + /encoder_ticks + /imu_raw
```

#### 3. Auto-Reconnect (Rec 4)

```
Timer setiap 5s: _serial_health_check()
  ↓
Jika serial_connected == False:
  ↓
_open_serial(): close stale → Serial() → init → sync → mark connected
```

#### 4. Diagnostic Publisher (Rec 6)

```
Timer 1 Hz: _publish_diagnostics()
  ↓
DiagnosticArray dengan 3 status:
  1. Serial Health: connected/disconnected, rx_packets, checksum_errors
  2. Odometry Health: feedback_age, total_distance, stale warning
  3. cmd_vel Watchdog: triggered/active, cmd_vel_age
  ↓
Publish ke /diagnostics
```

#### 5. Watchdog

```
Timer 10 Hz: watchdog_callback()
  ↓
Jika (now - last_cmd_stamp) > watchdog_timeout_s (0.5s):
  ↓
send_stop_command → RPM = (0, 0, 0)
watchdog_triggered = True
```

#### 6. Service: Reset Odometry

```
ros2 service call /reset_odom std_srvs/srv/Empty
  ↓
x = y = yaw = 0, total_distance = total_rotation = 0
```

---

## 7. ROS2 — robot_mission (Sequencer Layer)

### Node: `mission_sequencer` (mission_sequencer.py, ~982 baris)

"Otak" robot — menerima tabel step dari Foxglove, mengeksekusi step satu per satu.

### State Machine

```
         EXECUTE_RAW
    ┌────────────────────┐
    │                    ▼
    │    ┌──────────┐  RUN   ┌──────────┐
    │    │   IDLE   │───────▶│ RUNNING  │
    │    └──────────┘        └─────┬────┘
    │         ▲                    │
    │    RESUME│         ┌────────┼────────┐
    │         │         │        │        │
    │    ┌────┴────┐ PAUSE  exit_ok   ABORT
    │    │ PAUSED  │◀───┘    │        │
    │    └─────────┘         ▼        ▼
    │              ┌──────────┐  ┌──────────┐
    │              │   DONE   │  │ ABORTED  │
    │              └──────────┘  └──────────┘
    │                                 │
    └─────────────────────────────────┘
                (error → ERROR state)
```

### Command Dictionary

Robot bergerak berdasarkan **CMD intent**, bukan velocity mentah:

| CMD | Arti | Parameter |
|-----|------|-----------|
| `MAJU` | Maju (+X body) | X = speed |
| `MUNDUR` | Mundur (-X body) | X = speed |
| `KIRI` / `GESER_KIRI` | Geser kiri (+Y body) | Y = speed |
| `KANAN` / `GESER_KANAN` | Geser kanan (-Y body) | Y = speed |
| `PUTAR_KIRI` | Putar CCW | W = speed angular |
| `PUTAR_KANAN` | Putar CW | W = speed angular |
| `STRAFE` | Diagonal (free) | X, Y = speed |
| `STRAFE_MAJU_KIRI` | Diagonal maju-kiri | X, Y = speed |
| `STRAFE_MAJU_KANAN` | Diagonal maju-kanan | X, Y = speed |
| `STRAFE_MUNDUR_KIRI` | Diagonal mundur-kiri | X, Y = speed |
| `STRAFE_MUNDUR_KANAN` | Diagonal mundur-kanan | X, Y = speed |
| `BALANCE` | Pass-through | X, Y, W langsung |
| `STOP` | Diam total | — |
| `MAJU_JARAK` | Maju sejauh X cm | X = jarak (auto-exit) |
| `MUNDUR_JARAK` | Mundur sejauh X cm | X = jarak (auto-exit) |

### Exit Conditions

Setiap step punya exit condition — kapan pindah ke step berikutnya:

| Mode | Satuan | Contoh |
|------|--------|--------|
| `TIME` | milidetik | `{"mode":"TIME", "op":">=", "val":2000}` → pindah setelah 2 detik |
| `ANGLE` | derajat | `{"mode":"ANGLE", "op":"<=", "val":-90}` → pindah setelah putar 90° CW |
| `DIST` | cm atau m | `{"mode":"DIST", "op":">=", "val":100}` → pindah setelah jarak 1m |

Comparator: `>=`, `<=`, `>`, `<`, `==`

### Unit Conversion

Default di `sequencer_params.yaml`:
- `input_linear_unit: cm` → step X/Y dalam cm/s, dikonversi ke m/s sebelum publish
- `input_angular_unit: deg` → step W dalam deg/s, dikonversi ke rad/s
- `exit_dist_unit: cm` → exit DIST val dalam cm

### Contoh Misi: Jalan Kotak 1m × 1m

```json
{
  "cmd": "EXECUTE_RAW",
  "steps": [
    {"cmd":"MAJU",        "x":20, "y":0, "w":0, "exit":{"mode":"DIST","op":">=","val":100}},
    {"cmd":"KIRI",        "x":0, "y":20, "w":0, "exit":{"mode":"DIST","op":">=","val":100}},
    {"cmd":"MUNDUR",      "x":20, "y":0, "w":0, "exit":{"mode":"DIST","op":">=","val":100}},
    {"cmd":"KANAN",       "x":0, "y":20, "w":0, "exit":{"mode":"DIST","op":">=","val":100}},
    {"cmd":"STOP",        "x":0, "y":0, "w":0, "exit":{"mode":"TIME","op":">=","val":1000}}
  ]
}
```

### Foxglove Control Topics

| Topic | Tipe | Arah | Fungsi |
|-------|------|------|--------|
| `/mission/control` | `std_msgs/String` | Foxglove → Pi | Command JSON |
| `/mission/status` | `std_msgs/String` | Pi → Foxglove | Status JSON (5 Hz) |

---

## 8. Kinematika Kiwi Drive

### Inverse Kinematics (cmd_vel → kecepatan roda)

Dari velocity body (vx, vy, wz) ke kecepatan roda (v1, v2, v3):

$$v_1 = -0.5 \cdot v_x - \frac{\sqrt{3}}{2} \cdot v_y + L \cdot \omega_z$$

$$v_2 = -0.5 \cdot v_x + \frac{\sqrt{3}}{2} \cdot v_y + L \cdot \omega_z$$

$$v_3 = v_x + L \cdot \omega_z$$

Dimana $L$ = `robot_radius` = 0.20 m.

### Forward Kinematics (kecepatan roda → twist body)

$$\omega_z = \frac{v_1 + v_2 + v_3}{3L}$$

$$v_x = v_3 - L \cdot \omega_z$$

$$v_y = \frac{v_2 - v_1}{2 \cdot \frac{\sqrt{3}}{2}} = \frac{v_2 - v_1}{\sqrt{3}}$$

### Conversion: Ticks → m/s

$$v_{roda} = \frac{\Delta_{ticks} \times 2\pi \times r_{roda}}{T_{ticks/rev} \times \Delta t}$$

Dimana:
- $\Delta_{ticks}$ = delta encoder ticks
- $r_{roda}$ = wheel_radius = 0.05 m
- $T_{ticks/rev}$ = ticks_per_rev = 380
- $\Delta t$ = selang waktu (s)

### Conversion: m/s → RPM

$$RPM = \frac{v \times 60}{2\pi \times r_{roda}}$$

### Contoh Perhitungan

**Maju 0.2 m/s (vx=0.2, vy=0, wz=0):**
- v1 = -0.5 × 0.2 = -0.1 m/s
- v2 = -0.5 × 0.2 = -0.1 m/s
- v3 = 0.2 m/s
- RPM1 = -0.1 × 60 / (2π × 0.05) = -19.1 RPM
- RPM2 = -19.1 RPM
- RPM3 = +38.2 RPM

**Putar CCW 1.0 rad/s (vx=0, vy=0, wz=1.0):**
- v1 = v2 = v3 = 0.20 × 1.0 = 0.2 m/s
- RPM1 = RPM2 = RPM3 = +38.2 RPM (semua roda putar searah)

---

## 9. PID & Motor Control Pipeline

### PID Controller (C, di STM32)

```c
// Discrete PID with anti-windup
float error = setpoint - measured;
integral += error * dt;
integral = clamp(integral, -integral_max, integral_max);
float derivative = (error - prev_error) / dt;
float output = Kp * error + Ki * integral + Kd * derivative;
output = clamp(output, -255, 255);
```

### Tuning Parameters

| Parameter | Nilai | Satuan |
|-----------|-------|--------|
| Kp | 0.40 | PWM/RPM |
| Ki | 3.60 | PWM/(RPM·s) |
| Kd | 0.04 | PWM·s/RPM |
| Integral limit | ±50 | PWM |
| Output range | ±255 | PWM counts |
| Dead-zone | 2 RPM | Reset integral |

### Feedforward

Kompensasi non-linearitas motor (stiction + back-EMF):

$$PWM_{FF} = \text{sign}(RPM) \times (5.0 + 0.3 \times |RPM|)$$

- Deadband: jika |RPM| < 3 → FF = 0
- Total output: `PWM = FF + PID_trim`

### Rate Limiting

PWM berubah max ±2 digit per tick (200 Hz) = ±400/s.
Mencegah lonjakan arus saat motor ganti arah mendadak.

### RPM Ramp

Target RPM di-slew-rate-limit:
- Naik: +2.5 RPM/tick = +500 RPM/s
- Turun: -4.0 RPM/tick = -800 RPM/s (lebih agresif untuk braking)
- Instant snap pada reversal (ganti arah)
- Snap ke 0 jika target = 0 dan actual < 0.3 RPM

---

## 10. IMU & Sensor Fusion

### MPU6050 Configuration

| Setting | Nilai |
|---------|-------|
| Gyro range | ±250 °/s (sensitivity: 131 LSB/°/s) |
| Accel range | ±2g (sensitivity: 16384 LSB/g) |
| Sample rate | ~100 Hz (DLPF enabled) |
| I2C address | 0x68 (AD0 = LOW) |

### Scaling Factors

```c
GYRO_SCALE  = 1/131 × π/180  = 0.000133 rad/s per LSB
ACCEL_SCALE = 9.80665/16384   = 0.000599 m/s² per LSB
```

### EMA Gyro Filter (STM32)

```
α = 0.70 (CONTROL_IMU_FILTER_ALPHA)

gyro_{x,y,z}_filtered = α × prev + (1-α) × raw
```

- α = 0.70 → 70% previous (smoothing), 30% new sample
- Cutoff frequency ≈ 4.8 Hz at 100 Hz sample rate
- Applied to all 3 gyro axes; accel is passed raw (hardware DLPF at 44 Hz)
- Settles dalam ~200-500 ms (cocok untuk manuver cepat omni-drive)
- Sebelumnya α=0.95 (terlalu lamban, f_c ≈ 0.8 Hz) — diubah dalam audit

### Data yang Dikirim ke Pi

```
gyro_x/y/z: EMA-filtered angular rate × 1000 → int16 (milli-rad/s)
accel_x/y/z: raw linear accel     × 1000 → int16 (milli-m/s²)
```

### Auto-Disable

Jika MPU6050 I2C gagal >10 kali berturut-turut → `imu_ready = false`, task tetap jalan tapi output freeze. Odometry tetap berfungsi dari encoder saja.

---

## 11. Odometry

### Cara Kerja

Odometry dihitung di Pi (mms_bridge.py), bukan di STM32.

```
Setiap batch feedback (50 Hz):
1. Hitung dt_total = now - last_feedback_stamp
2. Bagi dt rata ke N sample: dt_per_sample = dt_total / N
3. Per sample:
   a. Forward kinematics: delta_ticks → vx, vy (body frame)
   b. delta_yaw = gyro_z × dt (dari IMU, bukan encoder)
   c. yaw_mid = yaw + 0.5 × delta_yaw
   d. dx_odom = dx_body × cos(yaw_mid) - dy_body × sin(yaw_mid)
   e. dy_odom = dx_body × sin(yaw_mid) + dy_body × cos(yaw_mid)
   f. x += dx_odom, y += dy_odom, yaw += delta_yaw
4. Normalize yaw ke [-π, π]
5. Publish /odom + /tf
```

### Kenapa Heading dari Gyro?

Untuk omni-drive, heading dari encoder kurang akurat karena:
- 3 roda omni sering slip (rollernya bebas berputar lateral)
- Gyro langsung mengukur angular velocity tanpa asumsi no-slip

Kekurangan: gyro drift over time (no absolute reference). Untuk kompetisi pendek (<30 menit), drift masih acceptable.

### Odometry Covariance (Rec 1)

Covariance bertambah seiring jarak/rotasi yang ditempuh (dead-reckoning drift):

```python
σ²_xy = max(odom_k_xy × total_distance, 1e-4)     # default: 0.01 m²/m
σ²_θ  = max(odom_k_theta × total_rotation, 1e-4)   # default: 0.02 rad²/rad
```

Ini penting untuk sensor fusion (robot_localization / nav2) agar tahu confidence odometry menurun seiring waktu.

---

## 12. Foxglove Integration

### Setup

```bash
# Install foxglove_bridge
sudo apt install ros-humble-foxglove-bridge

# Jalankan
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### Koneksi dari Laptop

Foxglove Studio → Open Connection → Foxglove WebSocket → `ws://<PI_IP>:8765`

### Panel Layout (lihat docs/FOXGLOVE_PANELS.md)

```
┌─────────────────────────────────────────┐
│         3D Visualization (odom+TF)      │
├──────────────┬──────────────────────────┤
│ Mission      │ Mission Status           │
│ Control      │ (/mission/status)        │
├──────────────┴──────────────────────────┤
│     Telemetry Plots (position/vel)      │
├──────────────┬──────────────────────────┤
│ Encoders     │ IMU Raw                  │
├──────────────┴──────────────────────────┤
│     Diagnostics (/diagnostics)          │
└─────────────────────────────────────────┘
```

### Workflow

1. Setup panel di Foxglove (one-time)
2. Connect ke robot via WebSocket
3. Gunakan Publish panel untuk mengirim JSON ke `/mission/control`
4. Upload tabel step (`EXECUTE_RAW`)
5. Kirim `RUN` → robot bergerak
6. Monitor real-time: posisi, velocity, encoder, IMU, diagnostics
7. `PAUSE` / `ABORT` jika ada masalah

---

## 13. Konfigurasi Parameter

### mms_params.yaml (robot_mms)

```yaml
motor_bridge_node:
  ros__parameters:
    # Serial
    serial_port: /dev/ttyACM0          # STM32 Nucleo via USB
    serial_baudrate: 115200
    serial_timeout_s: 0.01
    serial_boot_wait_s: 0.5
    serial_sync_max_attempts: 2000
    serial_reset_buffers_on_start: false

    # Geometry
    wheel_radius: 0.05                 # meter (50mm)
    robot_radius: 0.20                 # meter (200mm, pusat→roda)
    ticks_per_rev: 380                 # ticks encoder per putaran
    max_rpm: 600                       # batas RPM motor

    # Safety
    watchdog_timeout_s: 0.5            # stop motor jika cmd_vel timeout

    # Velocity clamping (Rec 2)
    max_linear_velocity: 2.0           # m/s
    max_angular_velocity: 10.0         # rad/s

    # Odometry covariance (Rec 1)
    odom_k_xy: 0.01                    # m²/m traveled
    odom_k_theta: 0.02                 # rad²/rad rotated

    # Serial reconnect (Rec 4)
    serial_reconnect_interval_s: 5.0   # interval retry

    # Frames
    odom_frame_id: odom
    base_frame_id: base_link

    # Topics
    cmd_vel_topic: /cmd_vel
    encoder_ticks_topic: /encoder_ticks
    imu_raw_topic: /imu_raw
    imu_data_topic: /imu/data
    odom_topic: /odom
    reset_odom_service: /reset_odom
```

### sequencer_params.yaml (robot_mission)

```yaml
mission_sequencer:
  ros__parameters:
    publish_rate_hz: 25.0              # Rate publish /cmd_vel
    odom_max_age_ms: 500.0             # Stale odom threshold
    stop_hold_ticks: 10                # Mempastikan motor berhenti penuh
    idle_keepalive_hz: 5.0             # Keepalive agar watchdog tidak aktif
    max_linear_speed: 1.0              # m/s
    max_angular_speed: 3.14            # rad/s
    input_linear_unit: cm              # Step X/Y dalam cm/s
    input_angular_unit: deg            # Step W dalam deg/s
    exit_dist_unit: cm                 # Exit DIST dalam cm
    default_linear_speed: 20.0         # Default speed MAJU_JARAK (cm/s)
```

### control_config.h (STM32 Firmware)

```c
// Encoder
#define CONTROL_TICKS_PER_REV       380.0f
#define CONTROL_INVERT_ENC1         false
#define CONTROL_INVERT_ENC2         false
#define CONTROL_INVERT_ENC3         true    // wiring terbalik

// PID
#define CONTROL_PID_KP              0.40f
#define CONTROL_PID_KI              3.60f
#define CONTROL_PID_KD              0.04f
#define CONTROL_PID_INTEGRAL_LIMIT  50.0f

// Motor
#define CONTROL_PWM_MAX             255
#define CONTROL_INVERT_PWM1         false
#define CONTROL_INVERT_PWM2         false
#define CONTROL_INVERT_PWM3         false

// Feedforward
#define CONTROL_FF_SLOPE            0.3f
#define CONTROL_FF_OFFSET           5.0f
#define CONTROL_FF_DEADBAND_RPM     3.0f

// RPM
#define CONTROL_MAX_RPM             600
#define CONTROL_RAMP_UP_RATE        2.5f
#define CONTROL_RAMP_DOWN_RATE      4.0f
#define CONTROL_DEAD_ZONE_RPM       2.0f
#define CONTROL_ZERO_THRESHOLD      0.3f

// EMA filter
#define CONTROL_EMA_ALPHA           0.30f
#define CONTROL_EMA_BETA            0.70f

// PWM rate limit
#define CONTROL_PWM_RATE_LIMIT_PER_TICK  2

// IMU
#define CONTROL_IMU_FILTER_ALPHA    0.70f
#define CONTROL_IMU_ERROR_THRESHOLD 10
```

---

## 14. ROS2 Topics, Services & Frames

### Topics

| Topic | Tipe | Publisher | Rate | Deskripsi |
|-------|------|----------|------|-----------|
| `/cmd_vel` | `Twist` | mission_sequencer | 25 Hz | Perintah kecepatan robot |
| `/odom` | `Odometry` | mms_bridge | 50 Hz + 10 Hz | Posisi robot (encoder+IMU) |
| `/tf` | `TFMessage` | mms_bridge | 50 Hz + 10 Hz | Transform odom→base_link |
| `/imu/data` | `Imu` | mms_bridge | 50 Hz | Full 6-axis IMU (3 gyro + 3 accel) |
| `/imu_raw` | `Float32MultiArray` | mms_bridge | 50 Hz | Debug: [gx, gy, gz, ax, ay, az] |
| `/encoder_ticks` | `Float32MultiArray` | mms_bridge | 50 Hz | Debug: [t1, t2, t3] delta ticks |
| `/diagnostics` | `DiagnosticArray` | mms_bridge | 1 Hz | Health: serial, odom, watchdog |
| `/mission/control` | `String` | Foxglove | On-demand | Command JSON ke sequencer |
| `/mission/status` | `String` | mission_sequencer | 5 Hz | Status JSON dari sequencer |

### Services

| Service | Tipe | Node | Deskripsi |
|---------|------|------|-----------|
| `/reset_odom` | `std_srvs/Empty` | mms_bridge | Reset odometry ke (0,0,0) |

### TF Frames

```
odom → base_link
```

- `odom`: fixed frame (origin = posisi awal robot)
- `base_link`: robot body frame (bergerak dengan robot)

---

## 15. Safety & Watchdog

### Multi-Layer Safety

```
Layer 1: Sequencer speed clamp (max_linear_speed, max_angular_speed)
    ↓
Layer 2: Bridge velocity clamp (max_linear_velocity, max_angular_velocity)
    ↓
Layer 3: Bridge RPM clamp (±max_rpm = ±600)
    ↓
Layer 4: Bridge watchdog (0.5s timeout → send RPM=0)
    ↓
Layer 5: STM32 firmware RPM clamp (±CONTROL_MAX_RPM = ±600)
    ↓
Layer 6: STM32 firmware watchdog (250ms timeout → comm_healthy=false → PWM=0)
    ↓
Layer 7: STM32 PID output clamp (±255 PWM)
    ↓
Layer 8: STM32 PWM rate limit (±2 per 5ms tick)
```

### Watchdog Timeline

```
t=0:     Last cmd_vel received
t=0.5s:  Pi bridge watchdog → send RPM=(0,0,0) to STM32
t=0.5s+: STM32 receives RPM=0 → motors stop via PID
t=0.75s: (worst case) STM32 firmware watchdog → comm_healthy=false → PWM=0 force
```

### NaN/Inf Rejection (Rec 2)

```python
if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
    self.get_logger().warn('cmd_vel rejected: non-finite values')
    return  # don't send to motor
```

### Emergency Stop Options

1. **Fisik: Cabut baterai** (instant)
2. **Ctrl+C pada mms_bridge** (sends RPM=0 sebelum exit)
3. **Publish zero:** `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'`
4. **Abort mission:** `{"cmd":"ABORT"}`
5. **Cabut USB** → STM32 watchdog 250ms → motor stop

---

## 16. Build & Deploy

### STM32 Firmware

```bash
cd ~/STM32_Firmware_Robot

# Build
pio run

# Flash via ST-LINK
pio run -t upload

# Monitor serial (debug)
pio device monitor -b 115200
```

Requirement:
- PlatformIO CLI
- ST-LINK driver (biasanya sudah built-in di Linux)
- Platform: `ststm32`, Board: `nucleo_f446re`, Framework: `stm32cube`

### ROS2 Packages

```bash
cd ~/mms-mobile-robot

# Build
colcon build --symlink-install

# Source
source install/setup.bash

# Jalankan mms_bridge
ros2 run robot_mms mms_bridge --ros-args --params-file \
  ~/mms-mobile-robot/robot_mms/config/mms_params.yaml

# Jalankan mission_sequencer
ros2 run robot_mission mission_sequencer --ros-args --params-file \
  ~/mms-mobile-robot/robot_mission/config/sequencer_params.yaml
```

### Dependency

```
ROS2 Humble di Raspberry Pi:
  rclpy, geometry_msgs, nav_msgs, sensor_msgs,
  std_msgs, std_srvs, tf2_ros, diagnostic_msgs

Python:
  pyserial (pip3 install pyserial)

Serial Permission:
  sudo usermod -aG dialout $USER
  # logout/login setelahnya
```

### Auto-Start (systemd)

Bisa dibuat systemd service agar node otomatis start saat Pi boot:

```bash
# /etc/systemd/system/mms_bridge.service
[Unit]
Description=MMS Motor Bridge
After=network.target

[Service]
Type=simple
User=pi
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && \
  source /home/pi/mms-mobile-robot/install/setup.bash && \
  ros2 run robot_mms mms_bridge --ros-args --params-file \
  /home/pi/mms-mobile-robot/robot_mms/config/mms_params.yaml"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

---

## 17. Pin Mapping & Wiring

### STM32F446RE Nucleo-64 Pin Assignments

#### Motor PWM (BTS7960)

| Motor | RPWM Pin | LPWM Pin | Timer | Channel |
|-------|----------|----------|-------|---------|
| Motor 1 | PA8 (M1_RPWM) | PA9 (M1_LPWM) | TIM1 | CH1 / CH2 |
| Motor 2 | PA10 (M2_RPWM) | PA11 (M2_LPWM) | TIM1 | CH3 / CH4 |
| Motor 3 | PA0 (M3_RPWM) | PA1 (M3_LPWM) | TIM2 | CH1 / CH2 |

**Penting:** TIM1 = advanced timer, butuh MOE (Main Output Enable) bit di BDTR register. Tanpa `__HAL_TIM_MOE_ENABLE(&htim1)`, Motor 1 & 2 tidak akan bergerak!

#### Motor Enable

| Pin | Port | Fungsi |
|-----|------|--------|
| PB0 | GPIOB | EN_MOTOR (shared, active HIGH) |

Catatan: Ketiga BTS7960 berbagi 1 pin enable. Individual disable hanya bisa via PWM=0.

#### Encoder (Quadrature 4x)

| Motor | CH_A Pin | CH_B Pin | Timer |
|-------|----------|----------|-------|
| Encoder 1 | PA6 (ENC1_CH_A) | PA7 (ENC1_CH_B) | TIM3 |
| Encoder 2 | PB6 (ENC2_CH_A) | PB7 (ENC2_CH_B) | TIM4 |
| Encoder 3 | PC6 (ENC3_CH_A) | PC7 (ENC3_CH_B) | TIM8 |

Semua timer 16-bit (0-65535). Mode: TI12 (4× quadrature), ICFilter=5.

#### IMU (MPU6050)

| Pin | Port | Fungsi |
|-----|------|--------|
| PB8 | GPIOB | MPU_SCL (I2C1) |
| PB9 | GPIOB | MPU_SDA (I2C1) |

#### Serial (USART2 via ST-LINK)

| Pin | Port | Fungsi |
|-----|------|--------|
| PA2 | GPIOA | USART2_TX (ke Pi via ST-LINK VCP) |
| PA3 | GPIOA | USART2_RX (dari Pi via ST-LINK VCP) |

DMA: USART2_RX via DMA (receive-to-idle).

#### Timer Assignment Summary

| Timer | Fungsi | Rate/Mode |
|-------|--------|-----------|
| TIM1 | Motor 1/2 PWM (advanced, MOE) | PWM output |
| TIM2 | Motor 3 PWM (general) | PWM output |
| TIM3 | Encoder 1 | Quadrature 4x |
| TIM4 | Encoder 2 | Quadrature 4x |
| TIM5 | PID tick source backup | — |
| TIM6 | PID tick 200 Hz (ISR) | 200 Hz update interrupt |
| TIM8 | Encoder 3 | Quadrature 4x |
| TIM12 | Servo/Stepper (optional) | PWM output |

#### Other Pins

| Pin | Fungsi | Status |
|-----|--------|--------|
| PA5 | EN_STEPPER | Stepper motor enable (future) |
| PB10 | STP1 | Stepper 1 step pin (future) |
| PB4/PB5 | STP2/STP3 | Stepper 2/3 step (future) |
| PA12/PA15/PB3 | DIR1/DIR2/DIR3 | Stepper direction (future) |
| PB14/PB15 | SERVO2/SERVO3 | Servo PWM (TIM12, future) |
| PC0/PC1 | LINE_D1/LINE_D2 | Line sensor (future) |
| PC2/PC3/PC12/PD2 | TRIG_* | Ultrasonic trigger (future) |
| PC4-5/PB2/PB12/PC8-11 | ECHO_* | Ultrasonic echo (future) |

---

## 18. Legacy ESP32

### MMS_ROBOTIC_ESP/

Repository asli menggunakan ESP32 sebagai motor controller. **Sudah diganti STM32** untuk performa lebih baik (180 MHz vs 240 MHz tapi FreeRTOS lebih deterministic).

Perbedaan utama ESP32 vs STM32:
- ESP32 butuh handshake `'b'` untuk masuk binary mode; STM32 langsung binary
- ESP32 PID di 100 Hz; STM32 200 Hz
- ESP32 pakai `micros()` per PID; STM32 pakai fixed dt dari timer
- ESP32 pakai library Arduino (ESP32Encoder, Adafruit MPU6050); STM32 pakai bare HAL
- Protokol serial identik (8 byte cmd, 26 byte feedback, XOR checksum)
- ESP32 watchdog 1000 ms; STM32 watchdog 250 ms

Repository ini dipertahankan untuk referensi dan sebagai plan B jika STM32 bermasalah.

---

## 19. Test Infrastructure

### Unit Tests (245 total, semua PASS)

```bash
cd ~/mms-mobile-robot
python3 -m pytest tests/test_*.py -v
```

| File | Tests | Coverage |
|------|:-----:|----------|
| `test_bug_fixes_phase1.py` | 19 | BUG 1-3: robot_radius, logging, rpm saturation |
| `test_bug_fixes_phase2.py` | 42 | BUG 4-5: yaw normalize, IMU covariance |
| `test_rec1_rec2_phase3.py` | 50 | REC 1-2: odom covariance, velocity clamp |
| `test_rec3_rec4_phase4.py` | 49 | REC 3-4: IMU filter α, serial reconnect |
| `test_rec5_rec6_phase5.py` | 85 | REC 5-6: encoder overflow doc, diagnostics |

### Hardware Test Scripts

| File | Tests | Requirement |
|------|:-----:|-------------|
| `hw_test_suite1_bug_regression.py` | 5 | Robot ON, wheels up |
| `hw_test_suite2_hardening.py` | 6 | Robot ON, some need movement |
| `hw_test_suite3_e2e.py` | 4 | Robot ON, clear floor |

### Standalone Tools

| File | Fungsi |
|------|--------|
| `test_encoder_read.py` | Baca encoder manual (putar roda → cek 380 ticks) |
| `test_motor_diag.py` | Diagnosa motor: serial, encoder, PWM drive |
| `test_rpm_step.py` | Step response RPM → CSV untuk tuning PID |

---

## 20. Troubleshooting

### Motor tidak bergerak

```
1. Cek baterai > 11.7V
2. Cek BTS7960 power LED menyala
3. python3 tests/test_motor_diag.py → ikuti diagnosis
4. Cek /dev/ttyACM0 ada: ls /dev/ttyACM*
5. Cek serial permission: groups $USER → harus ada 'dialout'
6. Cek TIM1 MOE (Motor 1/2 only): sudah di-fix dalam firmware
```

### Odometry drift

```
1. Cek roda tidak slip (encoder count = 380/putaran)
2. Cek IMU gyro offset: ros2 topic echo /imu/data → angular_velocity.z harus ≈ 0 saat diam
3. Increase odom_k_xy/odom_k_theta jika covariance terlalu kecil
4. Cek robot_radius = 0.20 (ukur fisik!)
5. Cek wheel_radius = 0.05 (ukur fisik!)
6. Reset odom: ros2 service call /reset_odom std_srvs/srv/Empty
```

### Serial disconnected

```
1. Cek kabel USB (strain relief!)
2. Cek log: ros2 topic echo /diagnostics → Serial Health
3. Auto-reconnect aktif (Rec 4) — tunggu 5 detik
4. Jika gagal: restart mms_bridge
5. Cek dmesg | tail → apakah USB device muncul
```

### Mission sequencer stuck

```
1. Cek /mission/status: ros2 topic echo /mission/status
2. Cek state: IDLE/RUNNING/PAUSED/ERROR/ABORTED/DONE
3. Jika ERROR: baca error_reason di status JSON
4. Jika odom stale: cek /odom rate (ros2 topic hz /odom → harus 50 Hz)
5. Abort + restart: {"cmd":"ABORT"} lalu {"cmd":"RUN"}
```

### Node crash

```
1. ros2 node list → node mana yang hilang?
2. Restart node yang mati (lihat section Build & Deploy)
3. Reset odom jika perlu
4. Cek dmesg dan journalctl untuk error
```

---

## Ringkasan Angka-Angka Penting

| Item | Nilai | Catatan |
|------|-------|--------|
| CPU Frequency | 180 MHz | STM32F446RE |
| PID Rate | 200 Hz | TIM6 interrupt |
| Feedback Rate | 50 Hz | STM32 → Pi |
| IMU Rate | 100 Hz | MPU6050 polling |
| Sequencer Rate | 25 Hz | /cmd_vel publish |
| Encoder Resolution | 380 ticks/rev | 7 PPR × 4 (quad) × 95/7 (gearbox) |
| Max RPM | 600 | Motor + gearbox |
| Max Tick Rate | 3800 ticks/s | @ 600 RPM |
| Wheel Radius | 0.05 m | 50 mm |
| Robot Radius | 0.20 m | 200 mm |
| Max Linear Velocity | ~3.14 m/s | Theoretical (600 RPM) |
| Serial Baud | 115200 | 8N1 |
| Command Packet | 8 bytes | Pi → STM32 |
| Feedback Packet | 26 bytes | STM32 → Pi |
| Pi Watchdog | 500 ms | cmd_vel timeout |
| STM32 Watchdog | 250 ms | comm timeout |
| IMU Filter α | 0.70 | EMA gyro filter (3 axes) |
| PID Gains | 0.40/3.60/0.04 | Kp/Ki/Kd |
| Unit Tests | 245 | All passing |

---

*Dokumen ini dihasilkan otomatis dari analisis kode sumber 3 repository pada 2026-03-05.*
