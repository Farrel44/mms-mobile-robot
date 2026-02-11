# Panduan Setup Panel Foxglove Studio

Setup panel Foxglove untuk monitoring dan kontrol robot omni-wheel LKS dengan arsitektur **Indexed Runtime Command Sequencer**.

## Layout Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      3D Visualization                       │
│                    (Robot + TF + Path)                      │
│                                                             │
├──────────────────────┬──────────────────────────────────────┤
│   Mission Control    │        Mission Status               │
│   (Publish Panel)    │     (Raw Messages Panel)            │
│                      │                                      │
├──────────────────────┴──────────────────────────────────────┤
│              Telemetry Plots (Multi-plot)                   │
│          Position / Velocity / Orientation                  │
│                                                             │
├──────────────────────┬──────────────────────────────────────┤
│   Debug: Encoders    │      Debug: IMU Raw                 │
│   (Raw Messages)     │     (Raw Messages)                  │
├──────────────────────┴──────────────────────────────────────┤
│              Diagnostics & Topic Health                     │
└─────────────────────────────────────────────────────────────┘
```

---

## Panel 1: 3D Visualization (TOP - Full Width)

**Panel Type:** `3D`

**Settings:**
- **Fixed Frame:** `odom`
- **Follow Mode:** `pose` (optional, untuk auto-follow robot)
- **Background:** Dark atau sesuai preferensi

**Topics to Add:**

1. **TF (Transform Tree)**
   - Topic: `/tf`
   - Type: `tf2_msgs/TFMessage`
   - Display: Show frame axes (`odom` ↔ `base_link`)

2. **Robot Footprint (Marker/Polygon)**
   - Buat custom marker atau publish `visualization_msgs/Marker` dari robot
   - Untuk sementara bisa pakai `Axes` dari TF `base_link` (radius 0.2m sesuai `robot_radius`)

3. **Odometry Path**
   - Topic: `/odom`
   - Type: `nav_msgs/Odometry`
   - Display: 
     - ✅ Show pose (robot position)
     - ✅ Show covariance (optional)
     - ✅ Show path history (set history: 1000 messages)
     - Line color: Cyan/Blue

4. **Grid**
   - Enable grid di settings
   - Cell size: 0.5m atau 1m

**Gunakan untuk:**
- ✅ Monitor posisi real-time robot
- ✅ Visualize trajectory/path yang sudah dijalani
- ✅ Verify orientasi robot (yaw dari TF)
- ✅ Debug: apakah robot bergerak sesuai command?

---

## Panel 2: Mission Control (LEFT - Below 3D)

**Panel Type:** `Publish`

**Settings:**
- **Topic:** `/mission/control`
- **Datatype:** `std_msgs/String`

> ⚠️ **PENTING - Format Message:**  
> `std_msgs/String` memiliki field `data` yang berisi string. Di Foxglove ada **2 cara**:
> 
> **Cara 1 (RECOMMENDED) - String Mode:**
> - Di Foxglove, ubah input mode ke **"String"** (bukan JSON)
> - Paste langsung JSON command tanpa field `data`
> 
> **Cara 2 - JSON Mode:**
> - Bungkus dalam object dengan field `data` (harus escape quotes)
> - Contoh: `{"data": "{\"cmd\": \"RUN\"}"}`

**Quick Actions (Save as templates):**

### Template 1: EXECUTE_RAW (Load Mission)

**String Mode (recommended):**
```json
{"cmd": "EXECUTE_RAW", "steps": [{"cmd": "MAJU", "x": 0.2, "y": 0, "w": 0, "exit": {"mode": "TIME", "op": ">=", "val": 2000}}, {"cmd": "PUTAR_KANAN", "x": 0, "y": 0, "w": 1.0, "exit": {"mode": "ANGLE", "op": "<=", "val": -90}}]}
```

**JSON Mode (alternative):**
```json
{
  "data": "{\"cmd\": \"EXECUTE_RAW\", \"steps\": [{\"cmd\": \"MAJU\", \"x\": 0.2, \"y\": 0, \"w\": 0, \"exit\": {\"mode\": \"TIME\", \"op\": \">=\", \"val\": 2000}}, {\"cmd\": \"PUTAR_KANAN\", \"x\": 0, \"y\": 0, \"w\": 1.0, \"exit\": {\"mode\": \"ANGLE\", \"op\": \"<=\", \"val\": -90}}]}"
}
```

### Template 2: RUN

**String Mode:**
```json
{"cmd": "RUN"}
```

**JSON Mode:**
```json
{"data": "{\"cmd\": \"RUN\"}"}
```

### Template 3: PAUSE

**String Mode:**
```json
{"cmd": "PAUSE"}
```

**JSON Mode:**
```json
{"data": "{\"cmd\": \"PAUSE\"}"}
```

### Template 4: RESUME

**String Mode:**
```json
{"cmd": "RESUME"}
```

**JSON Mode:**
```json
{"data": "{\"cmd\": \"RESUME\"}"}
```

### Template 5: ABORT

**String Mode:**
```json
{"cmd": "ABORT"}
```

**JSON Mode:**
```json
{"data": "{\"cmd\": \"ABORT\"}"}
```

### Template 6: SET_INDEX (Jump to Step)

**String Mode:**
```json
{"cmd": "SET_INDEX", "index": 0}
```

**JSON Mode:**
```json
{"data": "{\"cmd\": \"SET_INDEX\", \"index\": 0}"}
```

**Gunakan untuk:**
- ✅ Load mission table (EXECUTE_RAW)
- ✅ RUN/PAUSE/RESUME/ABORT mission
- ✅ Jump ke step tertentu (SET_INDEX)

---

## Panel 3: Mission Status (RIGHT - Below 3D)

**Panel Type:** `Raw Messages`

**Settings:**
- **Topic:** `/mission/status`
- **Expansion depth:** 3-4 (untuk expand JSON nested)
- **Auto-scroll:** ON

**Gunakan untuk:**
- ✅ Monitor state machine (IDLE/RUNNING/PAUSED/DONE/ABORTED/ERROR)
- ✅ Monitor current step index
- ✅ Monitor exit condition progress (time_ms, angle, distance)
- ✅ Monitor error messages

**Status JSON Structure (Reference):**
```json
{
  "state": "RUNNING",
  "current_step": 0,
  "total_steps": 3,
  "cmd": "MAJU",
  "exit_mode": "TIME",
  "exit_condition": ">=",
  "exit_value": 2000,
  "time_ms": 1250,
  "angle_deg": 0.0,
  "distance_m": 0.25,
  "message": "Step 0: MAJU in progress"
}
```

---

## Panel 4: Telemetry Plots (MIDDLE - Full Width)

**Panel Type:** `Plot`

**Settings:**
- **Y-Axis Mode:** Multiple series dengan legend

**Series to Add:**

### Plot 1: Position X/Y
- **Topic:** `/odom` → `pose.pose.position.x`
- **Label:** `X Position (m)`
- **Color:** Red
- **Topic:** `/odom` → `pose.pose.position.y`
- **Label:** `Y Position (m)`
- **Color:** Green

### Plot 2: Linear Velocity
- **Topic:** `/odom` → `twist.twist.linear.x`
- **Label:** `Vx (m/s)`
- **Color:** Cyan
- **Topic:** `/odom` → `twist.twist.linear.y`
- **Label:** `Vy (m/s)`
- **Color:** Magenta

### Plot 3: Angular Velocity
- **Topic:** `/odom` → `twist.twist.angular.z`
- **Label:** `Wz (rad/s)`
- **Color:** Yellow

### Plot 4: Orientation (Yaw from Quaternion)
- **Topic:** `/odom` → extract yaw dari quaternion (bisa pakai custom transform atau separate node publish yaw)
- **Label:** `Yaw (deg)`
- **Color:** Orange

**Time Window:** 30-60 seconds (adjustable)

**Gunakan untuk:**
- ✅ Monitor kecepatan real-time
- ✅ Verify apakah velocity sesuai command
- ✅ Debug overshoot/undershoot di exit condition

---

## Panel 5: Debug - Encoder Ticks (BOTTOM LEFT)

**Panel Type:** `Raw Messages`

**Settings:**
- **Topic:** `/encoder_ticks`
- **Datatype:** `std_msgs/Float32MultiArray`
- **Expansion depth:** 2

**Gunakan untuk:**
- ✅ Monitor raw encoder values [tick1, tick2, tick3]
- ✅ Verify encoder feedback dari ESP32
- ✅ Debug: apakah encoder bergerak saat motor berputar?

**Format Data:**
```yaml
data:
  - 0.0   # Motor 1 ticks
  - 0.0   # Motor 2 ticks
  - 0.0   # Motor 3 ticks
```

---

## Panel 6: Debug - IMU Raw (BOTTOM RIGHT)

**Panel Type:** `Raw Messages`

**Settings:**
- **Topic:** `/imu_raw`
- **Datatype:** `std_msgs/Float32MultiArray`
- **Expansion depth:** 2

**Gunakan untuk:**
- ✅ Monitor raw IMU values [gyro_z, angle_x_rad]
- ✅ Verify gyroscope feedback
- ✅ Debug: apakah gyro_z berubah saat robot berputar?

**Format Data:**
```yaml
data:
  - -0.07   # gyro_z (rad/s) - angular velocity Z
  - -0.949  # angle_x_rad - tilt/roll dari accelerometer
```

---

## Panel 7: Diagnostics (BOTTOM - Optional)

**Panel Type:** `Topic Stats` atau `Diagnostics`

**Topics to Monitor:**
- `/odom` → rate ~10 Hz (dari periodic timer)
- `/tf` → rate ~10 Hz
- `/encoder_ticks` → rate ~20 Hz (jika feedback dari ESP32 jalan)
- `/imu_raw` → rate ~20 Hz
- `/mission/status` → rate ~25 Hz (saat RUNNING)

**Gunakan untuk:**
- ✅ Monitor topic health (apakah publish rate normal?)
- ✅ Detect issue: jika `/encoder_ticks` tidak publish → serial feedback gagal

---

## Quick Start Workflow

### 1. **Sebelum Connect:**
Pastikan semua node running:
```bash
# Terminal 1: Driver
ros2 run robot_mms mms_bridge

# Terminal 2: Mission Sequencer
ros2 run robot_mission mission_sequencer

# Terminal 3: Foxglove Bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### 2. **Connect Foxglove:**
- Open connection → Foxglove WebSocket
- URL: `ws://192.168.0.67:8765`

### 3. **Add Panels (Recommended Order):**
1. Add `3D` panel → set Fixed Frame = `odom`
2. Add `/tf` dan `/odom` ke 3D panel
3. Add `Publish` panel → setup templates untuk mission control
4. Add `Raw Messages` panel → `/mission/status`
5. Add `Plot` panel → setup telemetry series
6. Add `Raw Messages` panels → `/encoder_ticks`, `/imu_raw`

### 4. **Save Layout:**
- Save layout dengan nama `Robot LKS - Mission Control`
- Export layout file untuk backup

---

## Mission Testing Checklist

### Test 1: Basic Motion
```json
{"cmd":"EXECUTE_RAW","steps":[{"cmd":"MAJU","x":0.1,"y":0,"w":0,"exit":{"mode":"TIME","op":">=","val":3000}}]}
{"cmd":"RUN"}
```
**Expected:**
- ✅ Mission status → RUNNING
- ✅ 3D view: robot bergerak maju (X+)
- ✅ Velocity plot: Vx = 0.1 m/s
- ✅ Encoder ticks berubah

### Test 2: Rotation
```json
{"cmd":"EXECUTE_RAW","steps":[{"cmd":"PUTAR_KIRI","x":0,"y":0,"w":0.5,"exit":{"mode":"ANGLE","op":">=","val":90}}]}
{"cmd":"RUN"}
```
**Expected:**
- ✅ 3D view: robot rotate CCW 90°
- ✅ Angular velocity plot: Wz = 0.5 rad/s
- ✅ Gyro_z di `/imu_raw` berubah
- ✅ Mission DONE saat angle >= 90°

### Test 3: Multi-Step Sequence
```json
{"cmd":"EXECUTE_RAW","steps":[
  {"cmd":"MAJU","x":0.15,"y":0,"w":0,"exit":{"mode":"TIME","op":">=","val":2000}},
  {"cmd":"PUTAR_KANAN","x":0,"y":0,"w":0.8,"exit":{"mode":"ANGLE","op":"<=","val":-90}},
  {"cmd":"MUNDUR","x":0.15,"y":0,"w":0,"exit":{"mode":"DIST","op":">=","val":0.3}}
]}
{"cmd":"RUN"}
```
**Expected:**
- ✅ Step 0 → MAJU 2 detik
- ✅ Step 1 → PUTAR_KANAN sampai -90°
- ✅ Step 2 → MUNDUR 0.3 meter
- ✅ State → DONE

---

## Advanced: Custom Panels (Optional)

### Panel 8: Custom Step Table Visualizer
Bisa pakai `Table` panel atau `Custom` panel untuk visualize mission table kayak spreadsheet:

| Index | CMD | X | Y | W | Exit Mode | Exit Op | Exit Val |
|-------|-----|---|---|---|-----------|---------|----------|
| 0 | MAJU | 0.2 | 0 | 0 | TIME | >= | 2000 |
| 1 | PUTAR_KANAN | 0 | 0 | 1.0 | ANGLE | <= | -90 |

Ini requires custom script/node yang parse `/mission/status` jadi table format.

### Panel 9: Command Dictionary Reference
Buat `Text` panel atau `Image` panel dengan quick reference command dictionary:

```
MAJU        → X=speed   Maju (+X)
MUNDUR      → X=speed   Mundur (-X)
KIRI        → Y=speed   Strafe kiri (+Y)
KANAN       → Y=speed   Strafe kanan (-Y)
PUTAR_KIRI  → W=speed   CCW (+)
PUTAR_KANAN → W=speed   CW (-)
BALANCE     → passthrough
STOP        → All zero
```

---

## Troubleshooting

### Issue: Topics tidak muncul di Foxglove
- **Check:** Apakah Foxglove Bridge running? (`ros2 node list | grep foxglove`)
- **Check:** Apakah topics ada di ROS? (`ros2 topic list`)
- **Fix:** Restart Foxglove Bridge

### Issue: 3D view tidak show robot
- **Check:** Fixed Frame = `odom`?
- **Check:** Apakah `/tf` publish `odom → base_link`? (`ros2 run tf2_ros tf2_echo odom base_link`)
- **Fix:** Pastikan `mms_bridge` running (periodic TF publish)

### Issue: Mission status tidak update
- **Check:** Apakah `mission_sequencer` running?
- **Check:** Topic `/mission/status` publish? (`ros2 topic hz /mission/status`)

### Issue: Encoder/IMU raw tidak berubah
- **Check:** Serial feedback dari ESP32 berhasil? (lihat log `mms_bridge`)
- **Debug:** Jalankan `ros2 topic echo /encoder_ticks` di terminal
- **Root cause likely:** Serial communication issue ESP32 ↔ Pi

---

## Recommended Layout Files

Layout bisa di-save dan di-load ulang. Aku sarankan 2 layout:

1. **`layout_mission_control.json`** - Full panel untuk operation (semua panel aktif)
2. **`layout_debug.json`** - Focus pada debug (3D + raw messages + plots)

Export layout dari Foxglove menu: `Layout` → `Export layout to file`

---

## Summary

**MUST-HAVE Panels:**
1. ✅ 3D Visualization (`/tf` + `/odom`)
2. ✅ Mission Control (Publish `/mission/control`)
3. ✅ Mission Status (`/mission/status`)
4. ✅ Telemetry Plots (velocity/position)

**NICE-TO-HAVE Panels:**
5. Debug encoder/IMU raw
6. Topic diagnostics
7. Custom step table visualizer

**Key Goals:**
- Monitor real-time posisi robot di 3D
- Kontrol mission execution (RUN/PAUSE/ABORT)
- Monitor progress step-by-step
- Debug jika ada masalah (encoder, IMU, serial)
