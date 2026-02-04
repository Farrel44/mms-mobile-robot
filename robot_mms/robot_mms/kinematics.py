#!/usr/bin/env python3
import math

def velocity_to_rpm(wheel_velocity, wheel_radius):
    rpm = (wheel_velocity * 60.0) / (2.0 * math.pi * wheel_radius)
    return rpm

def cmd_vel_to_wheel_velocities(vx, vy, wz, robot_radius):
    rotation_vel = robot_radius * wz
    
    sqrt3_over_2 = math.sqrt(3) / 2
    v1 = -0.5 * vx - sqrt3_over_2 * vy + rotation_vel
    v2 = -0.5 * vx + sqrt3_over_2 * vy + rotation_vel
    v3 = vx + rotation_vel
    
    
    return v1, v2, v3

def cmd_vel_to_rpm(vx, vy, wz, wheel_radius, robot_radius):
    v1, v2, v3 = cmd_vel_to_wheel_velocities(vx, vy, wz, robot_radius)
    
    rpm1 = velocity_to_rpm(v1, wheel_radius)
    rpm2 = velocity_to_rpm(v2, wheel_radius)
    rpm3 = velocity_to_rpm(v3, wheel_radius)
    
    return rpm1, rpm2, rpm3


def delta_ticks_to_wheel_linear_velocities(delta_ticks, ticks_per_rev, wheel_radius, dt):
    """Convert delta encoder ticks to wheel linear velocities (m/s).

    Args:
        delta_ticks: Iterable of 3 int values [dt1, dt2, dt3]
        ticks_per_rev: Encoder ticks per revolution
        wheel_radius: Wheel radius (m)
        dt: Time delta (s)
    """
    if dt <= 0.0:
        return 0.0, 0.0, 0.0

    dt1, dt2, dt3 = delta_ticks
    two_pi = 2.0 * math.pi
    tick_to_rad = two_pi / float(ticks_per_rev)

    dtheta1 = dt1 * tick_to_rad
    dtheta2 = dt2 * tick_to_rad
    dtheta3 = dt3 * tick_to_rad

    ds1 = wheel_radius * dtheta1
    ds2 = wheel_radius * dtheta2
    ds3 = wheel_radius * dtheta3

    v1 = ds1 / dt
    v2 = ds2 / dt
    v3 = ds3 / dt
    return v1, v2, v3


def wheel_linear_velocities_to_body_twist(v1, v2, v3, robot_radius):
    """Forward kinematics for kiwi/omni 3-wheel drive.

    Returns body-frame twist (vx, vy, wz_enc), where:
    - vx, vy in m/s
    - wz_enc in rad/s (encoder-derived)
    """
    L = float(robot_radius)
    if L == 0.0:
        return 0.0, 0.0, 0.0

    wz_enc = (v1 + v2 + v3) / (3.0 * L)
    vx = v3 - L * wz_enc

    sqrt3_over_2 = math.sqrt(3.0) / 2.0
    vy = (v2 - v1) / (2.0 * sqrt3_over_2)

    return vx, vy, wz_enc


def delta_ticks_to_body_twist(delta_ticks, ticks_per_rev, wheel_radius, robot_radius, dt):
    """Convenience: delta ticks -> wheel v -> body twist."""
    v1, v2, v3 = delta_ticks_to_wheel_linear_velocities(
        delta_ticks, ticks_per_rev, wheel_radius, dt
    )
    return wheel_linear_velocities_to_body_twist(v1, v2, v3, robot_radius)
