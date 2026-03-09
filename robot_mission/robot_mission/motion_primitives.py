#!/usr/bin/env python3
"""
motion_primitives.py — Helper class untuk gerakan dasar robot dengan profil trapesoid.

Bukan ROS2 Node sendiri — di-inject ke node yang sudah ada (MissionSequencer).
Menghindari duplikasi subscriber /odom dan publisher /cmd_vel.

Setiap gerakan menggunakan velocity profil trapesoid:
    [ramp_up] → [cruise] → [ramp_down]
Exit: displacement dari /odom ≥ target - tolerance, atau timeout.

Package: robot_mission
"""

# ─── Standard Library ────────────────────────────────────────────
import math
import time

# ─── Third Party ─────────────────────────────────────────────────
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# =====================================================================
# Default parameter values (fallback jika YAML tidak menyediakan)
# =====================================================================
_DEFAULTS = {
    'position_tolerance_m': 0.02,
    'angle_tolerance_deg': 2.0,
    'default_linear_speed_m_s': 0.10,
    'default_angular_speed_rad_s': 0.30,
    'max_linear_speed_m_s': 0.20,
    'ramp_accel_m_s2': 0.20,
    'ramp_decel_dist_m': 0.05,
    'move_timeout_s': 10.0,
    'cmd_vel_rate_hz': 20.0,
}

# Minimum speed — mencegah stall saat ramp mendekati nol
_MIN_SPEED_M_S = 0.02
_MIN_SPEED_RAD_S = 0.05


class MotionPrimitives:
    """Helper class untuk gerakan dasar robot omni 3 roda.

    Arsitektur:
        - Bukan Node sendiri, di-inject ke MissionSequencer
        - Menerima data /odom via update_odom()
        - Publish /cmd_vel via referensi publisher dari host node

    Velocity profile: trapesoid (ramp_up → cruise → ramp_down).

    Args:
        node: ROS2 Node yang sudah ada (untuk logger dan clock)
        cmd_vel_pub: Publisher untuk /cmd_vel (geometry_msgs/Twist)
        params: dict parameter dari YAML section 'motion'
    """

    def __init__(
        self,
        node: Node,
        cmd_vel_pub,
        params: dict | None = None,
    ) -> None:
        """Inisialisasi MotionPrimitives.

        Args:
            node: ROS2 Node host (MissionSequencer)
            cmd_vel_pub: rclpy Publisher untuk /cmd_vel
            params: dict parameter motion dari YAML. Gunakan _DEFAULTS jika None.
        """
        self._node = node
        self._cmd_vel_pub = cmd_vel_pub
        self._logger = node.get_logger()

        # Load parameters dengan fallback ke defaults
        p = params or {}
        self._pos_tol = float(p.get('position_tolerance_m',
                                     _DEFAULTS['position_tolerance_m']))
        self._angle_tol_deg = float(p.get('angle_tolerance_deg',
                                           _DEFAULTS['angle_tolerance_deg']))
        self._angle_tol_rad = math.radians(self._angle_tol_deg)
        self._default_linear_speed = float(p.get('default_linear_speed_m_s',
                                                  _DEFAULTS['default_linear_speed_m_s']))
        self._default_angular_speed = float(p.get('default_angular_speed_rad_s',
                                                   _DEFAULTS['default_angular_speed_rad_s']))
        self._max_linear_speed = float(p.get('max_linear_speed_m_s',
                                              _DEFAULTS['max_linear_speed_m_s']))
        self._ramp_accel = float(p.get('ramp_accel_m_s2',
                                        _DEFAULTS['ramp_accel_m_s2']))
        self._ramp_decel_dist = float(p.get('ramp_decel_dist_m',
                                             _DEFAULTS['ramp_decel_dist_m']))
        self._timeout = float(p.get('move_timeout_s',
                                     _DEFAULTS['move_timeout_s']))
        self._rate_hz = float(p.get('cmd_vel_rate_hz',
                                     _DEFAULTS['cmd_vel_rate_hz']))
        self._dt = 1.0 / self._rate_hz

        # State odom terbaru (di-update dari luar via update_odom)
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_yaw: float = 0.0
        self._odom_received: bool = False

    # =================================================================
    # Odom Interface
    # =================================================================

    def update_odom(self, msg: Odometry) -> None:
        """Update state odom dari message /odom.

        Dipanggil oleh host node setiap kali menerima /odom callback.

        Args:
            msg: nav_msgs/Odometry message
        """
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self._odom_received = True

    def _get_current_pose(self) -> tuple[float, float, float]:
        """Ambil pose terbaru dari odom cache.

        Returns:
            (x, y, yaw_rad) — posisi dan orientasi robot saat ini
        """
        return self._odom_x, self._odom_y, self._odom_yaw

    # =================================================================
    # Quaternion & Angle Utilities
    # =================================================================

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """Extract yaw (radian) dari geometry_msgs/Quaternion.

        Hanya menggunakan komponen z dan w (rotasi 2D).

        Args:
            q: geometry_msgs/Quaternion

        Returns:
            yaw dalam radian, range (-π, π]
        """
        return 2.0 * math.atan2(q.z, q.w)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize sudut ke range (-π, π].

        Args:
            angle: sudut dalam radian

        Returns:
            sudut ternormalisasi (-π, π]
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle

    # =================================================================
    # Trapezoidal Velocity Profile
    # =================================================================

    def _compute_trapezoidal_speed(
        self,
        distance_traveled: float,
        total_distance: float,
        max_speed: float,
        current_speed: float,
    ) -> float:
        """Hitung kecepatan saat ini berdasarkan profil trapesoid.

        Zones:
            ramp_up:   speed += accel × dt (sampai max_speed)
            cruise:    speed = max_speed
            ramp_down: speed -= accel × dt (saat sisa_jarak < ramp_decel_dist)

        Jika total_distance terlalu pendek untuk full trapezoid,
        otomatis menjadi triangular profile (no cruise phase).

        Args:
            distance_traveled: jarak yang sudah ditempuh (m)
            total_distance: target jarak total (m)
            max_speed: kecepatan maksimum (m/s)
            current_speed: kecepatan saat ini (m/s)

        Returns:
            kecepatan baru (m/s), clamped ke [_MIN_SPEED, max_speed]
        """
        remaining = total_distance - distance_traveled

        # Tentukan decel distance — bisa lebih pendek dari default jika
        # total distance terlalu kecil (triangular profile)
        decel_start_dist = total_distance - self._ramp_decel_dist
        if decel_start_dist < total_distance / 2.0:
            # Distance terlalu pendek → triangular: decel mulai dari setengah
            decel_start_dist = total_distance / 2.0

        if remaining <= 0.0:
            # Sudah sampai
            return 0.0
        elif remaining < self._ramp_decel_dist or distance_traveled >= decel_start_dist:
            # Ramp down zone — deselerasi proporsional terhadap sisa jarak
            # Speed berkurang linear dari max_speed ke _MIN_SPEED
            ratio = remaining / self._ramp_decel_dist
            ratio = max(0.0, min(1.0, ratio))
            target_speed = max_speed * ratio
            # Apply deceleration rate limiter
            new_speed = current_speed - self._ramp_accel * self._dt
            # Jangan lebih cepat dari target proporsional
            new_speed = min(new_speed, target_speed)
        else:
            # Ramp up atau cruise zone
            new_speed = current_speed + self._ramp_accel * self._dt
            new_speed = min(new_speed, max_speed)

        # Clamp ke minimum dan maximum
        new_speed = max(_MIN_SPEED_M_S, min(max_speed, new_speed))
        return new_speed

    # =================================================================
    # Cmd_vel Publishing
    # =================================================================

    def _publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """Publish Twist ke /cmd_vel.

        Args:
            vx: kecepatan linear x (m/s)
            vy: kecepatan linear y (m/s)
            wz: kecepatan angular z (rad/s)
        """
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._cmd_vel_pub.publish(msg)

    # =================================================================
    # Linear Move (Generic)
    # =================================================================

    def _move_linear(
        self,
        distance_m: float,
        vx_sign: float,
        vy_sign: float,
        speed_m_s: float | None = None,
    ) -> bool:
        """Gerakan linear generik dengan profil trapesoid.

        Args:
            distance_m: jarak target (m), harus positif
            vx_sign: arah vx (+1.0 = maju, -1.0 = mundur, 0.0 = tidak ada)
            vy_sign: arah vy (+1.0 = kiri, -1.0 = kanan, 0.0 = tidak ada)
            speed_m_s: kecepatan maksimum, None = default

        Returns:
            True jika target tercapai, False jika timeout
        """
        if not self._odom_received:
            self._logger.warn('Motion aborted: odom belum diterima')
            self.stop()
            return False

        distance_m = abs(distance_m)
        if distance_m < self._pos_tol:
            self._logger.info('Move skipped: distance < tolerance')
            return True

        max_speed = speed_m_s if speed_m_s is not None else self._default_linear_speed
        max_speed = min(abs(max_speed), self._max_linear_speed)

        # Record start pose
        start_x, start_y, _ = self._get_current_pose()

        # Direction label untuk logging
        if vx_sign > 0:
            direction = "forward"
        elif vx_sign < 0:
            direction = "backward"
        elif vy_sign > 0:
            direction = "left"
        else:
            direction = "right"

        self._logger.info(
            f'Moving {direction} {distance_m * 1000:.0f}mm '
            f'at {max_speed:.2f} m/s')

        current_speed = 0.0
        t_start = time.monotonic()

        while True:
            # Check timeout
            elapsed = time.monotonic() - t_start
            if elapsed > self._timeout:
                x_now, y_now, _ = self._get_current_pose()
                traveled = math.hypot(x_now - start_x, y_now - start_y)
                self._logger.warn(
                    f'Move timeout after {elapsed:.1f}s '
                    f'(traveled {traveled * 1000:.0f}mm / '
                    f'{distance_m * 1000:.0f}mm)')
                self.stop()
                return False

            # Compute displacement
            x_now, y_now, _ = self._get_current_pose()
            traveled = math.hypot(x_now - start_x, y_now - start_y)

            # Check success
            if traveled >= distance_m - self._pos_tol:
                self.stop()
                self._logger.info(
                    f'Move complete: traveled {traveled * 1000:.0f}mm '
                    f'in {elapsed:.1f}s')
                return True

            # Compute trapezoidal speed
            current_speed = self._compute_trapezoidal_speed(
                traveled, distance_m, max_speed, current_speed)

            # Publish cmd_vel
            self._publish_cmd_vel(
                vx_sign * current_speed,
                vy_sign * current_speed,
                0.0,
            )

            # Rate control
            time.sleep(self._dt)

    # =================================================================
    # Public Move Methods
    # =================================================================

    def move_forward(
        self,
        distance_m: float,
        speed_m_s: float | None = None,
    ) -> bool:
        """Maju lurus ke depan (+X base_link).

        Args:
            distance_m: jarak target (m)
            speed_m_s: kecepatan maksimum (m/s), None = default

        Returns:
            True jika target tercapai, False jika timeout
        """
        return self._move_linear(distance_m, vx_sign=1.0, vy_sign=0.0,
                                  speed_m_s=speed_m_s)

    def move_backward(
        self,
        distance_m: float,
        speed_m_s: float | None = None,
    ) -> bool:
        """Mundur lurus ke belakang (-X base_link).

        Args:
            distance_m: jarak target (m)
            speed_m_s: kecepatan maksimum (m/s), None = default

        Returns:
            True jika target tercapai, False jika timeout
        """
        return self._move_linear(distance_m, vx_sign=-1.0, vy_sign=0.0,
                                  speed_m_s=speed_m_s)

    def move_left(
        self,
        distance_m: float,
        speed_m_s: float | None = None,
    ) -> bool:
        """Geser ke kiri (+Y base_link).

        Args:
            distance_m: jarak target (m)
            speed_m_s: kecepatan maksimum (m/s), None = default

        Returns:
            True jika target tercapai, False jika timeout
        """
        return self._move_linear(distance_m, vx_sign=0.0, vy_sign=1.0,
                                  speed_m_s=speed_m_s)

    def move_right(
        self,
        distance_m: float,
        speed_m_s: float | None = None,
    ) -> bool:
        """Geser ke kanan (-Y base_link).

        Args:
            distance_m: jarak target (m)
            speed_m_s: kecepatan maksimum (m/s), None = default

        Returns:
            True jika target tercapai, False jika timeout
        """
        return self._move_linear(distance_m, vx_sign=0.0, vy_sign=-1.0,
                                  speed_m_s=speed_m_s)

    # =================================================================
    # Rotate
    # =================================================================

    def rotate(
        self,
        angle_deg: float,
        speed_rad_s: float | None = None,
    ) -> bool:
        """Rotasi in-place.

        Positif = CCW, negatif = CW (REP-103 convention).

        Menggunakan profil trapesoid angular:
            ramp_up → cruise → ramp_down

        Args:
            angle_deg: sudut target (derajat), positif=CCW, negatif=CW
            speed_rad_s: kecepatan angular maksimum (rad/s), None = default

        Returns:
            True jika target tercapai, False jika timeout
        """
        if not self._odom_received:
            self._logger.warn('Rotate aborted: odom belum diterima')
            self.stop()
            return False

        angle_rad = math.radians(angle_deg)
        if abs(angle_rad) < self._angle_tol_rad:
            self._logger.info('Rotate skipped: angle < tolerance')
            return True

        max_omega = speed_rad_s if speed_rad_s is not None else self._default_angular_speed
        max_omega = abs(max_omega)

        # Direction: +1 = CCW, -1 = CW
        direction = 1.0 if angle_rad > 0 else -1.0
        target_angle = abs(angle_rad)

        # Decel zone angular: 15° (≈0.26 rad), cukup untuk smooth stop
        angular_decel_rad = math.radians(15.0)

        _, _, start_yaw = self._get_current_pose()

        self._logger.info(
            f'Rotating {"CCW" if direction > 0 else "CW"} '
            f'{abs(angle_deg):.1f}\u00b0 at {max_omega:.2f} rad/s')

        current_omega = 0.0
        accumulated_angle = 0.0
        prev_yaw = start_yaw
        t_start = time.monotonic()

        # Angular acceleration (rad/s²) — reach max_omega dalam 0.5s
        angular_accel = max_omega / 0.5

        while True:
            # Check timeout
            elapsed = time.monotonic() - t_start
            if elapsed > self._timeout:
                self._logger.warn(
                    f'Rotate timeout after {elapsed:.1f}s '
                    f'(rotated {math.degrees(accumulated_angle):.1f}\u00b0 / '
                    f'{abs(angle_deg):.1f}\u00b0)')
                self.stop()
                return False

            # Compute angle traveled (accumulated delta, handles wrap)
            _, _, current_yaw = self._get_current_pose()
            delta_yaw = self._normalize_angle(current_yaw - prev_yaw)
            accumulated_angle += abs(delta_yaw)
            prev_yaw = current_yaw

            # Check success
            if accumulated_angle >= target_angle - self._angle_tol_rad:
                self.stop()
                self._logger.info(
                    f'Rotate complete: {math.degrees(accumulated_angle):.1f}\u00b0 '
                    f'in {elapsed:.1f}s')
                return True

            # Trapezoidal angular profile
            remaining = target_angle - accumulated_angle

            decel_start = target_angle - angular_decel_rad
            if decel_start < target_angle / 2.0:
                decel_start = target_angle / 2.0

            if remaining < angular_decel_rad or accumulated_angle >= decel_start:
                # Ramp down
                ratio = remaining / angular_decel_rad
                ratio = max(0.0, min(1.0, ratio))
                target_omega = max_omega * ratio
                current_omega = current_omega - angular_accel * self._dt
                current_omega = min(current_omega, target_omega)
            else:
                # Ramp up / cruise
                current_omega = current_omega + angular_accel * self._dt
                current_omega = min(current_omega, max_omega)

            # Clamp minimum
            current_omega = max(_MIN_SPEED_RAD_S, min(max_omega, current_omega))

            # Publish cmd_vel
            self._publish_cmd_vel(0.0, 0.0, direction * current_omega)

            # Rate control
            time.sleep(self._dt)

    # =================================================================
    # Stop
    # =================================================================

    def stop(self) -> None:
        """Kirim cmd_vel = 0 beberapa kali untuk memastikan motor berhenti.

        Publish 3 kali dengan interval dt untuk memastikan message diterima
        oleh motor bridge (guard terhadap packet loss).
        """
        for _ in range(3):
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            time.sleep(self._dt)
