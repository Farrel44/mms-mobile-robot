# 🏁 DEPLOYMENT CHECKLIST — Kiwi Drive Omni-Wheel Robot (LKS 2026)

> **Version:** 1.0 — Post-Audit (5 Bug Fixes + 6 Recommendations Applied)
> **Date:** 2026-03-05
> **System:** Raspberry Pi 4 (ROS2 Humble) + STM32F446RE (FreeRTOS) + 3× BTS7960 + MPU6050

---

## Table of Contents

1. [Hardware Setup](#1-hardware-setup)
2. [Software Setup](#2-software-setup)
3. [Firmware Verification](#3-firmware-verification)
4. [Parameter Review](#4-parameter-review)
5. [Validation Tests](#5-validation-tests)
6. [Mission Upload](#6-mission-upload)
7. [Emergency Procedures](#7-emergency-procedures)
8. [Final GO / NO-GO](#8-final-go--no-go)
9. [Quick Reference Card](#9-quick-reference-card)

---

## 1. Hardware Setup

### 1.1 Power

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 1 | Battery voltage > 11.7 V (3S LiPo, full charge ≈ 12.6 V) | ☐ | Measure with multimeter at XT60 |
| 2 | 12 V rail to BTS7960 motor drivers confirmed | ☐ | All 3 modules powered |
| 3 | 5 V regulator output to Raspberry Pi stable (5.0 ± 0.25 V) | ☐ | Under-voltage causes random crashes |
| 4 | Battery capacity sufficient for mission + margin (≥ 30 min) | ☐ | Measure with coulomb counter or timer |

### 1.2 Motor Drivers (BTS7960)

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 5 | EN pin (PB0) wired to all 3 BTS7960 modules | ☐ | Shared single enable (not individual) |
| 6 | EN pin reads HIGH when STM32 boots and motors init | ☐ | LED on BTS7960 board = powered |
| 7 | RPWM/LPWM wiring matches `CONTROL_INVERT_PWMx` flags | ☐ | Motor 1-2: not inverted, Motor 3: not inverted |
| 8 | Motor direction test: positive RPM = expected rotation | ☐ | Use `test_motor_diag.py` TEST 3 |

### 1.3 Encoders

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 9 | Encoder A/B channels wired to correct timer pins | ☐ | M1→TIM3, M2→TIM4, M3→TIM8 |
| 10 | Encoder polarity matches `CONTROL_INVERT_ENCx` flags | ☐ | ENC1/2: not inverted, ENC3: inverted |
| 11 | Spin each wheel 1 full turn → reads ≈ 380 ticks | ☐ | Use `test_encoder_read.py`, tolerance ±5 |
| 12 | No encoder cable loose (strain relief applied) | ☐ | Vibration can disconnect during match |

### 1.4 IMU (MPU6050)

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 13 | I2C1 SDA/SCL wired (address 0x68, AD0 = LOW) | ☐ | |
| 14 | MPU6050 WHO_AM_I returns 0x68 (init success) | ☐ | STM32 UART debug prints "MPU6050 OK" |
| 15 | Gyro offset at rest < 0.5 °/s (bias check) | ☐ | Read `/imu/data` angular_velocity.z while stationary |
| 16 | IMU mounted rigidly on base plate (no vibration isolation needed at α=0.70) | ☐ | |

### 1.5 Serial Cable

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 17 | USB cable from Nucleo ST-LINK to Raspberry Pi | ☐ | Appears as `/dev/ttyACM0` |
| 18 | Cable secured with strain relief / zip-tie | ☐ | Disconnect during mission = 250 ms motor stop |
| 19 | No other USB-serial devices on Pi (avoid /dev/ttyACM1 confusion) | ☐ | Or update `serial_port` param |

### 1.6 Mechanical

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 20 | All 3 omni-wheels tight on shafts (no slip) | ☐ | Wheel slip = odometry error |
| 21 | Wheel rollers free-spinning (no debris jammed) | ☐ | |
| 22 | Robot center-of-mass roughly centered (tipping check) | ☐ | Uneven weight = IMU bias |
| 23 | `robot_radius` measured center-to-wheel = 0.20 m ± 2 mm | ☐ | Critical for kinematics |
| 24 | `wheel_radius` measured = 0.05 m ± 1 mm | ☐ | Critical for RPM↔m/s conversion |

---

## 2. Software Setup

### 2.1 Raspberry Pi OS & ROS2

| # | Item | Command / Action | Check |
|---|------|-----------------|:-----:|
| 1 | ROS2 Humble installed | `ros2 --version` → "humble" | ☐ |
| 2 | Workspace sourced | `source ~/mms-mobile-robot/install/setup.bash` | ☐ |
| 3 | Colcon build successful | `cd ~/mms-mobile-robot && colcon build --symlink-install` | ☐ |
| 4 | No build errors | Check terminal output — 0 errors, 0 warnings | ☐ |
| 5 | Serial port permissions | `sudo usermod -aG dialout $USER` + re-login | ☐ |
| 6 | Verify serial device exists | `ls -la /dev/ttyACM0` | ☐ |
| 7 | Python dependencies | `pip3 list \| grep pyserial` → pyserial installed | ☐ |
| 8 | diagnostic_msgs available | `ros2 interface show diagnostic_msgs/msg/DiagnosticArray` | ☐ |

### 2.2 Foxglove Bridge (Optional but Recommended)

| # | Item | Command / Action | Check |
|---|------|-----------------|:-----:|
| 9 | foxglove_bridge installed | `ros2 pkg list \| grep foxglove` | ☐ |
| 10 | Bridge launches | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765` | ☐ |
| 11 | Foxglove Studio connects | Open `ws://<PI_IP>:8765` in Foxglove | ☐ |
| 12 | Dashboard layout loaded | See `docs/FOXGLOVE_PANELS.md` for setup | ☐ |

### 2.3 Node Launch

Start nodes in this order:

```bash
# Terminal 1: Motor bridge (must start first)
ros2 run robot_mms mms_bridge --ros-args --params-file \
  ~/mms-mobile-robot/robot_mms/config/mms_params.yaml

# Terminal 2: Mission sequencer
ros2 run robot_mission mission_sequencer --ros-args --params-file \
  ~/mms-mobile-robot/robot_mission/config/sequencer_params.yaml

# Terminal 3 (optional): Foxglove bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

---

## 3. Firmware Verification

### 3.1 STM32 Flash

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 1 | PlatformIO environment: `nucleo_f446re` | ☐ | `platformio.ini` |
| 2 | Build succeeds: `pio run` | ☐ | 0 errors |
| 3 | Flash succeeds: `pio run -t upload` | ☐ | ST-LINK connected |
| 4 | Firmware contains all audit fixes: | | |
|   | — TIM1 MOE bit set (Motor 1/2 PWM works) | ☐ | `motor_pwm.c: MotorPwm_Init()` |
|   | — IMU filter α = 0.70 (not 0.95) | ☐ | `control_config.h: CONTROL_IMU_FILTER_ALPHA` |
|   | — Watchdog timeout = 250 ms | ☐ | `serial_protocol.h: WATCHDOG_TIMEOUT_MS` |
|   | — Encoder 4× quadrature (TI12 mode) | ☐ | `encoder_quad.c` |
|   | — PID rate = 200 Hz (TIM5 period) | ☐ | `tim.c: MX_TIM5_Init()` |

### 3.2 Firmware Version Tracking

| Field | Value |
|-------|-------|
| Git commit hash | `git -C ~/STM32_Firmware_Robot log --oneline -1` |
| Build date | _______________________ |
| Flash date | _______________________ |
| Operator | _______________________ |

---

## 4. Parameter Review

### 4.1 `mms_params.yaml` — Critical Parameters

| Parameter | Expected Value | Actual | OK? |
|-----------|---------------|--------|:---:|
| `serial_port` | `/dev/ttyACM0` | | ☐ |
| `serial_baudrate` | `115200` | | ☐ |
| `wheel_radius` | `0.05` | | ☐ |
| `robot_radius` | `0.20` | | ☐ |
| `ticks_per_rev` | `380` | | ☐ |
| `max_rpm` | `600` | | ☐ |
| `watchdog_timeout_s` | `0.5` | | ☐ |
| `max_linear_velocity` | `2.0` (or tuned) | | ☐ |
| `max_angular_velocity` | `10.0` (or tuned) | | ☐ |
| `odom_k_xy` | `0.01` (or tuned) | | ☐ |
| `odom_k_theta` | `0.02` (or tuned) | | ☐ |
| `serial_reconnect_interval_s` | `5.0` | | ☐ |

### 4.2 `sequencer_params.yaml` — Mission Parameters

| Parameter | Expected Value | Actual | OK? |
|-----------|---------------|--------|:---:|
| `publish_rate_hz` | `25.0` | | ☐ |
| `odom_max_age_ms` | `500.0` | | ☐ |
| `max_linear_speed` | `1.0` | | ☐ |
| `max_angular_speed` | `3.14` | | ☐ |
| `input_linear_unit` | `cm` | | ☐ |
| `input_angular_unit` | `deg` | | ☐ |
| `default_linear_speed` | `20.0` | | ☐ |

### 4.3 `control_config.h` — Firmware Tuning

| Parameter | Expected Value | Actual | OK? |
|-----------|---------------|--------|:---:|
| `CONTROL_TICKS_PER_REV` | `380.0f` | | ☐ |
| `CONTROL_PID_KP` | `0.40f` | | ☐ |
| `CONTROL_PID_KI` | `3.60f` | | ☐ |
| `CONTROL_PID_KD` | `0.04f` | | ☐ |
| `CONTROL_IMU_FILTER_ALPHA` | `0.70f` | | ☐ |
| `CONTROL_MAX_RPM` | `600` | | ☐ |
| `CONTROL_INVERT_ENC3` | `true` | | ☐ |

---

## 5. Validation Tests

### 5.1 Unit Tests (Run on Dev Machine or Pi)

```bash
cd ~/mms-mobile-robot
python3 -m pytest tests/test_*.py -v
```

| Test File | Tests | Result |
|-----------|:-----:|:------:|
| `test_bug_fixes_phase1.py` | 19 | ☐ PASS |
| `test_bug_fixes_phase2.py` | 42 | ☐ PASS |
| `test_rec1_rec2_phase3.py` | 50 | ☐ PASS |
| `test_rec3_rec4_phase4.py` | 49 | ☐ PASS |
| `test_rec5_rec6_phase5.py` | 85 | ☐ PASS |
| **Total** | **245** | ☐ ALL PASS |

### 5.2 Hardware Test Suite 1 — Bug Regression (Robot On, Wheels Up)

```bash
cd ~/mms-mobile-robot/tests
python3 hw_test_suite1_bug_regression.py
```

| Test | Description | Pass Criteria | Result |
|------|-------------|--------------|:------:|
| BUG-1 | robot_radius = 0.20 | Param reads 0.20 | ☐ |
| BUG-2 | Logging level correct | `get_logger().isEnabledFor(10)` = True | ☐ |
| BUG-3 | RPM saturation warning | Log shows warning at RPM > 600 | ☐ |
| BUG-4 | Yaw normalization | `/imu/data` yaw wraps within [-π, π] after 30s spin | ☐ |
| BUG-5 | IMU covariance non-zero | `/imu/data` covariance fields > 0 | ☐ |

### 5.3 Hardware Test Suite 2 — Hardening Validation

```bash
cd ~/mms-mobile-robot/tests
python3 hw_test_suite2_hardening.py
```

| Test | Description | Pass Criteria | Result |
|------|-------------|--------------|:------:|
| REC-1 | Odom covariance growth | Covariance increases after 1 m travel | ☐ |
| REC-2 | Velocity clamping | cmd_vel vx=10 → clamped to max_linear_velocity | ☐ |
| REC-3 | IMU filter α=0.70 | Step response settles < 500 ms | ☐ |
| REC-4 | Serial reconnect | Unplug/replug USB → node reconnects | ☐ |
| REC-5 | Encoder overflow docs | Source code contains safety analysis | ☐ |
| REC-6 | Diagnostic publisher | `/diagnostics` topic publishes at 1 Hz | ☐ |

### 5.4 Hardware Test Suite 3 — End-to-End (Clear Floor, Robot Down)

```bash
cd ~/mms-mobile-robot/tests

# Test 1: Square trajectory (45s, position error < 10 cm)
python3 hw_test_suite3_e2e.py --test 1

# Test 2: Rotation stability (drift < 10 cm after 5 rotations)
python3 hw_test_suite3_e2e.py --test 2

# Test 3: Mission dry-run (sequencer completes 5-state mission)
python3 hw_test_suite3_e2e.py --test 3

# Test 4: Long duration (default 10 min, extend with --duration 120)
python3 hw_test_suite3_e2e.py --test 4 --duration 120
```

| Test | Description | Pass Criteria | Result |
|------|-------------|--------------|:------:|
| E2E-1 | Square trajectory | Return-to-origin error < 10 cm | ☐ |
| E2E-2 | Rotation stability | Lateral drift < 10 cm over 5 rotations | ☐ |
| E2E-3 | Mission dry-run | All 5 states complete, no ERROR state | ☐ |
| E2E-4 | Long duration | No crashes for full duration | ☐ |

---

## 6. Mission Upload

### 6.1 Mission Table Preparation

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 1 | Mission YAML/JSON valid (no syntax errors) | ☐ | Validate with `python3 -c "import json; json.load(open('mission.json'))"` |
| 2 | All step commands are valid CMD types | ☐ | See `mission_sequencer.py` COMMAND_MAP |
| 3 | Exit conditions have valid comparators (`>=`, `<=`, `>`, `<`, `==`) | ☐ | |
| 4 | Linear speeds ≤ `max_linear_speed` (100 cm/s = 1.0 m/s) | ☐ | Sequencer clamps but verify intent |
| 5 | Angular speeds ≤ `max_angular_speed` (180 °/s = 3.14 rad/s) | ☐ | |
| 6 | Total mission duration estimated | ☐ | _________ seconds |

### 6.2 Mission Dry-Run

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 7 | Run in safe area (no obstacles, wheels up or clear floor) | ☐ | |
| 8 | All steps execute in order (watch `/mission/status`) | ☐ | |
| 9 | Robot returns to expected final pose | ☐ | Measure position error |
| 10 | No ERROR or ABORTED states during execution | ☐ | |
| 11 | Timing matches expected duration (±20%) | ☐ | |

### 6.3 Foxglove Dashboard Monitoring

| # | Item | Check | Notes |
|---|------|:-----:|-------|
| 12 | 3D panel shows robot pose + path trail | ☐ | |
| 13 | Mission status panel shows step progression | ☐ | |
| 14 | Telemetry plots showing encoder RPMs, velocities | ☐ | |
| 15 | Diagnostics panel shows OK (no WARN/ERROR) | ☐ | |

---

## 7. Emergency Procedures

### 7.1 Emergency Stop

| Priority | Method | Latency | Command |
|:--------:|--------|:-------:|---------|
| **1** | Kill power (battery disconnect) | Instant | Physical switch/unplug |
| **2** | STM32 watchdog auto-stop | 250 ms | Stop sending /cmd_vel |
| **3** | Publish zero velocity | 20 ms | `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'` |
| **4** | Kill mms_bridge node | 500 ms | `Ctrl+C` in terminal or `ros2 lifecycle set /motor_bridge_node shutdown` |
| **5** | Kill mission sequencer | 500 ms | `Ctrl+C` in mission terminal |

**Note:** The STM32 firmware watchdog (250 ms) provides hardware-level safety. If the Pi crashes or USB disconnects, motors stop automatically.

### 7.2 Serial Reconnect Recovery

If USB cable disconnects during operation:

1. Motors stop automatically (STM32 watchdog, 250 ms)
2. `mms_bridge` detects serial error → logs WARNING
3. Auto-reconnect attempts every 5.0 s (Rec 4)
4. Reconnect plug if physically disconnected
5. Verify: `ros2 topic hz /odom` → should resume 50 Hz

**If auto-reconnect fails:**
```bash
# Kill and restart bridge
Ctrl+C
ros2 run robot_mms mms_bridge --ros-args --params-file \
  ~/mms-mobile-robot/robot_mms/config/mms_params.yaml
```

### 7.3 Battery Swap Procedure

**Target time: < 60 seconds**

1. Abort mission: Ctrl+C on sequencer terminal
2. Confirm motors stopped (wheels not spinning)
3. Disconnect old battery (XT60)
4. Connect new battery (verify polarity!)
5. Check voltage > 11.7 V
6. STM32 reboots automatically (~2 s)
7. `mms_bridge` auto-reconnects serial (Rec 4)
8. Verify: `ros2 topic hz /odom` → 50 Hz within 10 s

### 7.4 IMU Failure Recovery

If MPU6050 I2C fails (10 consecutive errors → auto-disabled):

1. `/imu/data` stops publishing (OK — odometry still works via encoders)
2. `/diagnostics` → "IMU Health" status changes to WARN
3. **Fix:** Power-cycle STM32 (unplug/replug USB) → MPU6050 re-inits
4. Odometry yaw from encoders only (accuracy reduced but functional)

### 7.5 Node Crash Recovery

```bash
# Check which nodes are alive
ros2 node list

# If mms_bridge crashed, restart:
ros2 run robot_mms mms_bridge --ros-args --params-file \
  ~/mms-mobile-robot/robot_mms/config/mms_params.yaml

# If mission_sequencer crashed, restart:
ros2 run robot_mission mission_sequencer --ros-args --params-file \
  ~/mms-mobile-robot/robot_mission/config/sequencer_params.yaml

# Reset odometry origin (if robot moved during crash):
ros2 service call /reset_odom std_srvs/srv/Empty
```

---

## 8. Final GO / NO-GO

### 8.1 GO Criteria (ALL must be checked)

| # | Criterion | Verified By | Check |
|---|-----------|------------|:-----:|
| 1 | Battery voltage > 11.7 V | Multimeter | ☐ |
| 2 | All 3 motors respond to test command | `test_motor_diag.py` TEST 3 | ☐ |
| 3 | All 3 encoders read ≈ 380 ticks/rev | `test_encoder_read.py` | ☐ |
| 4 | `/odom` publishing at 50 Hz | `ros2 topic hz /odom` | ☐ |
| 5 | `/imu/data` publishing at 50 Hz | `ros2 topic hz /imu/data` | ☐ |
| 6 | `/diagnostics` shows no ERROR states | `ros2 topic echo /diagnostics` | ☐ |
| 7 | cmd_vel watchdog works (stop → motors stop in 500 ms) | Stop publishing, observe | ☐ |
| 8 | 245/245 unit tests pass | `pytest tests/test_*.py` | ☐ |
| 9 | HW Suite 1 bug regression: ALL PASS | `hw_test_suite1_bug_regression.py` | ☐ |
| 10 | HW Suite 2 hardening: ALL PASS | `hw_test_suite2_hardening.py` | ☐ |
| 11 | E2E square trajectory error < 10 cm | `hw_test_suite3_e2e.py --test 1` | ☐ |
| 12 | Mission dry-run completes successfully | `hw_test_suite3_e2e.py --test 3` | ☐ |
| 13 | Serial cable secured (strain relief) | Visual inspection | ☐ |
| 14 | Team confident in tuning | Test runs confirm accuracy | ☐ |

### 8.2 NO-GO Conditions (Any one = DO NOT COMPETE)

| # | Condition | Risk |
|---|-----------|------|
| 1 | Any motor not spinning | Robot cannot drive straight |
| 2 | Encoder reads 0 ticks (one or more) | No PID feedback → motor runaway |
| 3 | `/odom` not publishing | Mission sequencer cannot navigate |
| 4 | Unit test failures | Known bugs present in code |
| 5 | Square trajectory error > 20 cm | Odometry too inaccurate for missions |
| 6 | Battery voltage < 11.0 V | Motor brownout during high-current turns |
| 7 | Serial cable not secured | Disconnect during match = instant stop |

### 8.3 Decision

| Field | Value |
|-------|-------|
| **Decision** | ☐ GO / ☐ NO-GO |
| **Date/Time** | _______________________ |
| **Operator** | _______________________ |
| **Firmware Git Hash** | _______________________ |
| **ROS2 Package Git Hash** | _______________________ |
| **Notes** | _______________________ |

---

## 9. Quick Reference Card

> **Print this page and keep on the team desk during competition.**

### Key Commands

```bash
# Start motor bridge
ros2 run robot_mms mms_bridge --ros-args --params-file \
  ~/mms-mobile-robot/robot_mms/config/mms_params.yaml

# Start mission sequencer
ros2 run robot_mission mission_sequencer --ros-args --params-file \
  ~/mms-mobile-robot/robot_mission/config/sequencer_params.yaml

# Emergency stop (publish zero velocity)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'

# Check topic rates
ros2 topic hz /odom          # expect 50 Hz
ros2 topic hz /imu/data      # expect 50 Hz
ros2 topic hz /diagnostics   # expect 1 Hz

# Reset odometry to origin
ros2 service call /reset_odom std_srvs/srv/Empty

# Check active nodes
ros2 node list

# Quick motor test (drive forward 0.1 m/s for 2 seconds)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
# Ctrl+C to stop (watchdog auto-stops motors within 500 ms)
```

### Key Numbers

| Parameter | Value | Why It Matters |
|-----------|-------|---------------|
| Ticks/rev | 380 | Encoder calibration |
| Max RPM | 600 | PWM saturation limit |
| Wheel radius | 0.05 m | RPM → m/s conversion |
| Robot radius | 0.20 m | Rotation kinematics |
| Watchdog (firmware) | 250 ms | Motor auto-stop |
| Watchdog (bridge) | 500 ms | /cmd_vel timeout |
| Serial port | /dev/ttyACM0 | USB-to-STM32 |
| Baud rate | 115200 | 8N1 |
| IMU filter α | 0.70 | 70% gyro / 30% accel |
| PID gains | 0.40 / 3.60 / 0.04 | Kp / Ki / Kd |
| Feedback rate | 50 Hz | STM32 → Pi |
| PID rate | 200 Hz | STM32 internal loop |
| Serial reconnect | 5.0 s | Auto-retry interval |
| Foxglove port | 8765 | WebSocket bridge |

### Emergency Contacts

| Role | Name | Notes |
|------|------|-------|
| Software Lead | _____________ | ROS2 / Python issues |
| Hardware Lead | _____________ | Wiring / mechanical issues |
| Team Captain | _____________ | Competition decisions |

---

*End of Deployment Checklist — Last updated 2026-03-05*
