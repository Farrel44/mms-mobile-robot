#!/usr/bin/env python3
"""
Test sederhana: verify kinematika untuk command MAJU.
Expected: ketiga roda berputar dengan RPM yang sama untuk maju lurus.
"""

import sys
import os

# Import kinematics module
REPO_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOT_MMS_SRC = os.path.join(REPO_DIR, 'robot_mms')
if ROBOT_MMS_SRC not in sys.path:
    sys.path.insert(0, ROBOT_MMS_SRC)

from robot_mms import kinematics as kin

# Parameter robot (sesuai config)
WHEEL_RADIUS = 0.05   # 5 cm
ROBOT_RADIUS = 0.20   # 20 cm
MAX_RPM = 600

print('='*60)
print('TEST KINEMATIKA: Command MAJU')
print('='*60)
print(f'wheel_radius = {WHEEL_RADIUS} m')
print(f'robot_radius = {ROBOT_RADIUS} m')
print(f'max_rpm      = {MAX_RPM}')
print()

# Test case: MAJU dengan vx = 0.2 m/s
vx, vy, wz = 0.2, 0.0, 0.0

print(f'Input cmd_vel:')
print(f'  vx = {vx} m/s (maju)')
print(f'  vy = {vy} m/s')
print(f'  wz = {wz} rad/s')
print()

# Inverse kinematics
v1, v2, v3 = kin.cmd_vel_to_wheel_velocities(vx, vy, wz, ROBOT_RADIUS)
print(f'Wheel velocities (m/s):')
print(f'  v1 = {v1:.4f}')
print(f'  v2 = {v2:.4f}')
print(f'  v3 = {v3:.4f}')
print()

# Convert to RPM
rpm1, rpm2, rpm3 = kin.cmd_vel_to_rpm(vx, vy, wz, WHEEL_RADIUS, ROBOT_RADIUS)
print(f'Wheel RPM:')
print(f'  M1 = {rpm1:.1f} RPM')
print(f'  M2 = {rpm2:.1f} RPM')
print(f'  M3 = {rpm3:.1f} RPM')
print()

# Validation
print('='*60)
print('EXPECTED untuk MAJU LURUS (kiwi drive 120°):')
print('  - Roda 1 (120°): harus berputar MUNDUR (negatif)')
print('  - Roda 2 (240°): harus berputar MUNDUR (negatif)')
print('  - Roda 3 (0°):   harus berputar MAJU (positif)')
print()
print('Untuk vx=0.2 m/s:')
print('  v1 = -0.5*vx = -0.1 m/s')
print('  v2 = -0.5*vx = -0.1 m/s')
print('  v3 = vx = 0.2 m/s')
print('='*60)

# Check
if abs(v1 - (-0.1)) < 0.001 and abs(v2 - (-0.1)) < 0.001 and abs(v3 - 0.2) < 0.001:
    print('✅ KINEMATIKA BENAR!')
else:
    print('❌ KINEMATIKA SALAH! Ada bug di rumus inverse kinematics.')
    print(f'   Expected: v1=-0.1, v2=-0.1, v3=0.2')
    print(f'   Actual:   v1={v1:.4f}, v2={v2:.4f}, v3={v3:.4f}')
