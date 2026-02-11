# Robot Omni-Wheel LKS

Sistem navigasi robot omni-wheel 3 roda untuk kompetisi LKS Mobile Robotics. Menggunakan ROS 2 pada Raspberry Pi sebagai otak utama dan ESP32 sebagai motor controller.

Arsitektur: **Indexed Runtime Command Sequencer** (Table-based Motion Sequencer).
Foxglove = editor tabel step (programming panel). Robot = interpreter + executor.

## Struktur Direktori

```
robot_2026/
├── robot_mms/                  # [OTOT] Paket driver low-level
│   ├── robot_mms/
│   │   ├── mms_bridge.py       # Bridge serial ROS-ESP32
│   │   └── kinematics.py       # Rumus kinematika omni 3 roda
│   └── config/
│       └── mms_params.yaml     # Parameter robot
│
├── robot_mission/              # [OTAK] Paket sequencer high-level
│   ├── robot_mission/
│   │   └── mission_sequencer.py  # Indexed Runtime Command Sequencer
│   └── config/
│       └── sequencer_params.yaml # Parameter sequencer
│
├── build/                      # Hasil build (auto-generated)
├── install/                    # Instalasi paket (auto-generated)
└── log/                        # Log build (auto-generated)
```

## Instalasi

### Prasyarat
- ROS 2 (Humble/Iron)
- Python 3.8+
- pyserial

### Build Semua Paket

```bash
cd ~/robot_2026
colcon build
source install/setup.bash
```

Tambahkan ke `~/.bashrc` agar otomatis source saat buka terminal:
```bash
echo "source ~/robot_2026/install/setup.bash" >> ~/.bashrc
```

## Penggunaan

### 1. Jalankan Driver (robot_mms)

Pastikan ESP32 terhubung via USB.

```bash
ros2 run robot_mms mms_bridge
```

Dengan parameter kustom:
```bash
ros2 run robot_mms mms_bridge --ros-args \
  --params-file $(ros2 pkg prefix robot_mms)/share/robot_mms/config/mms_params.yaml
```

### 2. Jalankan Sequencer (robot_mission)

```bash
ros2 run robot_mission mission_sequencer
```

### 3. Jalankan Foxglove Bridge untuk visualisasi
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Foxglove Bridge akan otomatis subscribe semua topics dan menyediakannya via WebSocket di port 8765.

### 4. Hubungkan Foxglove ke Robot

**Di Foxglove Studio:**
1. Klik **"Open connection"** (dashboard atau menu kiri)
2. Pilih **"Foxglove WebSocket"**
3. Masukkan URL: `ws://192.168.0.67:8765` (ganti IP sesuai robot)
4. Klik **"Open"**

> **Catatan:** Untuk development lokal, gunakan `ws://localhost:8765`

### 5. Kontrol via Foxglove

Gunakan panel **Publish** untuk mengirim perintah ke topic `/mission/control` dengan tipe `std_msgs/String`.

**Konsep:** Foxglove adalah **editor tabel runtime** (bukan teleop). Setiap baris = 1 step. Robot mengeksekusi step berdasarkan index, berpindah saat exit condition terpenuhi.

> 📖 **Panduan Setup Panel Foxglove:** Lihat [docs/FOXGLOVE_PANELS.md](docs/FOXGLOVE_PANELS.md) untuk panduan lengkap setup visualization, mission control, telemetry, dan debug panels.

Contoh command:

| Aksi | JSON |
|------|------|
| Kirim tabel + siap RUN | `{"cmd":"EXECUTE_RAW","steps":[{"cmd":"MAJU","x":0.2,"y":0,"w":0,"exit":{"mode":"TIME","op":">=","val":2000}},{"cmd":"PUTAR_KANAN","x":0,"y":0,"w":1.0,"exit":{"mode":"ANGLE","op":"<=","val":-90}}]}` |
| RUN | `{"cmd":"RUN"}` |
| Pause | `{"cmd":"PAUSE"}` |
| Resume | `{"cmd":"RESUME"}` |
| Abort | `{"cmd":"ABORT"}` |
| Set index | `{"cmd":"SET_INDEX","index":2}` |
| Update tabel (saat RUNNING: lock step aktif) | `{"cmd":"UPDATE_STEPS","steps":[...]}` |

### 5. Monitor Status

Status sequencer tersedia di topic `/mission/status` dalam format JSON.

```bash
ros2 topic echo /mission/status
```

## Command Dictionary (CMD yang Didukung)

Setiap CMD = intent, bukan velocity mentah. X/Y/W = parameter per-CMD.

| CMD | Deskripsi | X | Y | W |
|-----|-----------|---|---|---|
| MAJU | Maju (+X base_link) | speed (m/s) | - | - |
| MUNDUR | Mundur (-X base_link) | speed (m/s) | - | - |
| KIRI | Strafe kiri (+Y) | - | speed (m/s) | - |
| KANAN | Strafe kanan (-Y) | - | speed (m/s) | - |
| PUTAR_KIRI | Putar CCW | - | - | speed (rad/s) |
| PUTAR_KANAN | Putar CW | - | - | speed (rad/s) |
| BALANCE | Stabilisasi (pass-through) | param | param | param |
| STOP | Diam total | - | - | - |

## Exit Condition (per step)

Step berpindah **hanya** jika exit condition terpenuhi.

| Mode | Satuan | Deskripsi |
|------|--------|-----------|
| TIME | ms | Waktu sejak step dimulai |
| ANGLE | degree | Delta yaw akumulasi (CCW +, CW −) |
| DIST | meter | Jarak euclidean dari posisi start |

Comparator (`op`): `>=`, `<=`, `>`, `<`, `==`

## Contoh Payload (Referensi)

```json
{
  "cmd": "EXECUTE_RAW",
  "steps": [
    { "cmd": "BALANCE", "x": 10, "y": 0, "w": 0, "exit": { "mode": "TIME", "op": ">=", "val": 3000 } },
    { "cmd": "MAJU", "x": 0, "y": 0, "w": 0, "exit": { "mode": "TIME", "op": ">=", "val": 700 } },
    { "cmd": "PUTAR_KANAN", "x": 0, "y": 0, "w": 0, "exit": { "mode": "ANGLE", "op": "<=", "val": -180 } }
  ]
}
```

Representasi visual di Foxglove:
```
[0] CMD=BALANCE      X=10  Y=0  W=0  EXIT=TIME >= 3000
[1] CMD=MAJU         X=0   Y=0  W=0  EXIT=TIME >= 700
[2] CMD=PUTAR_KANAN  X=0   Y=0  W=0  EXIT=ANGLE <= -180
```

## Parameter Robot (mms_params.yaml)

| Parameter | Default | Deskripsi |
|-----------|---------|-----------|
| serial_port | /dev/ttyUSB0 | Port serial ESP32 |
| wheel_radius | 0.05 | Radius roda (meter) |
| robot_radius | 0.20 | Jarak pusat ke roda (meter) |
| ticks_per_rev | 380 | Tick encoder per putaran |
| watchdog_timeout_s | 0.5 | Timeout stop motor jika cmd_vel berhenti |

## Parameter Sequencer (sequencer_params.yaml)

| Parameter | Default | Deskripsi |
|-----------|---------|-----------|
| publish_rate_hz | 25.0 | Rate publish /cmd_vel saat RUNNING |
| odom_max_age_ms | 500.0 | Batas umur odom sebelum stale → ERROR |
| stop_hold_ticks | 10 | Tick publish 0 saat berhenti |
| max_linear_speed | 1.0 | Batas kecepatan linear (m/s) |
| max_angular_speed | 3.14 | Batas kecepatan angular (rad/s) |

## Topic ROS

| Topic | Tipe | Arah | Deskripsi |
|-------|------|------|-----------|
| /cmd_vel | geometry_msgs/Twist | Sub | Perintah kecepatan |
| /odom | nav_msgs/Odometry | Pub | Posisi robot |
| /mission/control | std_msgs/String | Sub | Kontrol misi (JSON) |
| /mission/status | std_msgs/String | Pub | Status misi (JSON) |
| /imu/data | sensor_msgs/Imu | Pub | Data IMU |
| /encoder_ticks | std_msgs/Float32MultiArray | Pub | Debug encoder |

## Service

| Service | Tipe | Deskripsi |
|---------|------|-----------|
| /reset_odom | std_srvs/Empty | Reset posisi odometry ke (0,0,0) |

## Troubleshooting

### Cek node berjalan
```bash
ros2 node list
```

### Cek topic aktif
```bash
ros2 topic list
ros2 topic info /cmd_vel
```

### Monitor feedback ESP32
```bash
ros2 topic echo /encoder_ticks
ros2 topic echo /imu_raw
```

### Cek koneksi serial
```bash
ls /dev/ttyUSB*
```

### Reset odometry manual
```bash
ros2 service call /reset_odom std_srvs/srv/Empty "{}"
```

### Setup Foxglove panels
Lihat panduan lengkap di [docs/FOXGLOVE_PANELS.md](docs/FOXGLOVE_PANELS.md) untuk:
- Layout panel yang recommended
- Konfigurasi 3D visualization, mission control, dan telemetry
- Mission testing workflow
- Troubleshooting common issues