#!/usr/bin/env python3
import math

def velocity_to_rpm(wheel_velocity, wheel_radius):
    rpm = (wheel_velocity * 60.0) / (2.0 * math.pi * wheel_radius)
    return rpm

def cmd_vel_to_wheel_velocities(vx, vy, wz, robot_radius):
    rotation_vel = robot_radius * wz
    
    v1 = -0.5 * vx - 0.866 * vy + rotation_vel
    v2 = -0.5 * vx + 0.866 * vy + rotation_vel
    v3 = vx + rotation_vel
    
    return v1, v2, v3

def cmd_vel_to_rpm(vx, vy, wz, wheel_radius, robot_radius):
    v1, v2, v3 = cmd_vel_to_wheel_velocities(vx, vy, wz, robot_radius)
    
    rpm1 = velocity_to_rpm(v1, wheel_radius)
    rpm2 = velocity_to_rpm(v2, wheel_radius)
    rpm3 = velocity_to_rpm(v3, wheel_radius)
    
    return rpm1, rpm2, rpm3
