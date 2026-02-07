#!/usr/bin/env python3
"""
Modul kinematika untuk robot kiwi drive (omni 3 roda, 120°).

Berisi:
- Inverse kinematics : cmd_vel (vx, vy, wz) -> kecepatan roda / RPM
- Forward kinematics : delta ticks encoder -> twist body (vx, vy, wz)
"""
import math

# =====================================================================
# Konstanta
# =====================================================================
SQRT3_OVER_2 = math.sqrt(3.0) / 2.0
TWO_PI = 2.0 * math.pi


# =====================================================================
# Inverse Kinematics (command path: cmd_vel -> wheel RPM)
# =====================================================================

def cmd_vel_to_wheel_velocities(vx, vy, wz, robot_radius):
    """Hitung kecepatan linear tiap roda dari cmd_vel.

    Kiwi drive 3 roda di 120°:
      v1 = -0.5*vx - (√3/2)*vy + L*wz
      v2 = -0.5*vx + (√3/2)*vy + L*wz
      v3 =      vx              + L*wz

    Args:
        vx: kecepatan linear x (m/s), positif = maju
        vy: kecepatan linear y (m/s), positif = kiri (REP-103)
        wz: kecepatan angular z (rad/s), positif = CCW
        robot_radius: jarak pusat robot ke roda (m)

    Returns:
        (v1, v2, v3) dalam m/s
    """
    rotasi = robot_radius * wz

    v1 = -0.5 * vx - SQRT3_OVER_2 * vy + rotasi
    v2 = -0.5 * vx + SQRT3_OVER_2 * vy + rotasi
    v3 = vx + rotasi

    return v1, v2, v3


def velocity_to_rpm(wheel_velocity, wheel_radius):
    """Konversi kecepatan linear roda (m/s) ke RPM."""
    return (wheel_velocity * 60.0) / (TWO_PI * wheel_radius)


def cmd_vel_to_rpm(vx, vy, wz, wheel_radius, robot_radius):
    """Shortcut: cmd_vel langsung ke RPM tiap roda."""
    v1, v2, v3 = cmd_vel_to_wheel_velocities(vx, vy, wz, robot_radius)
    rpm1 = velocity_to_rpm(v1, wheel_radius)
    rpm2 = velocity_to_rpm(v2, wheel_radius)
    rpm3 = velocity_to_rpm(v3, wheel_radius)
    return rpm1, rpm2, rpm3


# =====================================================================
# Forward Kinematics (feedback path: delta ticks -> body twist)
# =====================================================================

def delta_ticks_to_wheel_velocities(delta_ticks, ticks_per_rev, wheel_radius, dt):
    """Konversi delta ticks encoder ke kecepatan linear roda (m/s).

    Args:
        delta_ticks: [dt1, dt2, dt3] delta ticks tiap roda
        ticks_per_rev: jumlah ticks per putaran penuh
        wheel_radius: radius roda (m)
        dt: selang waktu (s)

    Returns:
        (v1, v2, v3) dalam m/s
    """
    if dt <= 0.0:
        return 0.0, 0.0, 0.0

    tick_to_rad = TWO_PI / float(ticks_per_rev)
    dt1, dt2, dt3 = delta_ticks

    # ticks -> jarak tempuh roda (m)
    ds1 = wheel_radius * dt1 * tick_to_rad
    ds2 = wheel_radius * dt2 * tick_to_rad
    ds3 = wheel_radius * dt3 * tick_to_rad

    return ds1 / dt, ds2 / dt, ds3 / dt


def wheel_velocities_to_body_twist(v1, v2, v3, robot_radius):
    """Forward kinematics: kecepatan roda -> twist di frame body.

    Kebalikan dari inverse kinematics kiwi 3 roda:
      wz  = (v1 + v2 + v3) / (3*L)
      vx  = v3 - L*wz
      vy  = (v2 - v1) / (2 * √3/2)

    Args:
        v1, v2, v3: kecepatan linear roda (m/s)
        robot_radius: jarak pusat ke roda (m)

    Returns:
        (vx, vy, wz_enc) — vx/vy (m/s), wz_enc (rad/s dari encoder)
    """
    L = float(robot_radius)
    if L == 0.0:
        return 0.0, 0.0, 0.0

    wz_enc = (v1 + v2 + v3) / (3.0 * L)
    vx = v3 - L * wz_enc
    vy = (v2 - v1) / (2.0 * SQRT3_OVER_2)

    return vx, vy, wz_enc


def delta_ticks_to_body_twist(delta_ticks, ticks_per_rev, wheel_radius, robot_radius, dt):
    """Shortcut: delta ticks -> kecepatan roda -> twist body."""
    v1, v2, v3 = delta_ticks_to_wheel_velocities(
        delta_ticks, ticks_per_rev, wheel_radius, dt
    )
    return wheel_velocities_to_body_twist(v1, v2, v3, robot_radius)
