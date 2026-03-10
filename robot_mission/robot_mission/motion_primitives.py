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

# ─── Project ─────────────────────────────────────────────────────
from robot_mms.constants import OBSTACLE_STOP_DIST_M  # ADDED(phase2-sensor)
from robot_mms.constants import (  # ADDED(phase2-balance)
    BALANCE_STOP_DIST_M,
    BALANCE_APPROACH_SPEED_M_S,
    BALANCE_TIMEOUT_S,
)


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
        self._params = p  # ADDED(phase2-d3-linefollower): store for follow_line_u
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

        # ADDED(phase2-sensor): Obstacle awareness
        self._obstacle_distances: dict[str, float] = {
            'front': -1.0, 'right': -1.0, 'rear': -1.0, 'left': -1.0,
        }
        self._obstacle_check_enabled: bool = False

        # REVISED(phase2-line-marker): Line sensor sebagai position marker
        self._line_d1: int = 0
        self._line_d2: int = 0
        self._line_received: bool = False

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
    # Obstacle Awareness — ADDED(phase2-sensor)
    # =================================================================

    def set_obstacle_distances(self, distances: dict[str, float]) -> None:
        """Update jarak obstacle per arah dari /obstacle/distances.

        Dipanggil oleh host node setiap kali menerima data obstacle.

        Args:
            distances: dict {'front': m, 'right': m, 'rear': m, 'left': m}.
                       Nilai -1.0 = invalid/tidak ada data.
        """
        self._obstacle_distances = dict(distances)

    def enable_obstacle_check(self, enabled: bool) -> None:
        """Aktifkan/nonaktifkan pengecekan obstacle saat _move_linear.

        Args:
            enabled: True untuk mengaktifkan obstacle stop.
        """
        self._obstacle_check_enabled = enabled
        self._logger.info(f'Obstacle check {"enabled" if enabled else "disabled"}')

    # =================================================================
    # Line Sensor Interface — REVISED(phase2-line-marker)
    # =================================================================

    def update_line_state(self, d1: int, d2: int) -> None:
        """Update state line sensor.

        Dipanggil dari mission_sequencer setiap kali
        menerima /line_sensor/state.

        Args:
            d1: state sensor kiri D1 (0=off, 1=on/garis terdeteksi)
            d2: state sensor kanan D2 (0=off, 1=on/garis terdeteksi)
        """
        self._line_d1 = d1
        self._line_d2 = d2
        self._line_received = True

    def _line_marker_detected(self) -> bool:
        """Cek apakah line sensor mendeteksi marker posisi.

        Returns True jika D1 ATAU D2 mendeteksi garis hitam.
        Digunakan sebagai stop condition untuk move_until_marker().

        Returns:
            True jika marker terdeteksi, False jika tidak.
        """
        return bool(self._line_d1 or self._line_d2)

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

            # ADDED(phase2-sensor): Obstacle check — stop jika obstacle di arah gerak
            if self._obstacle_check_enabled:
                obs_dist = self._obstacle_distances.get(direction, -1.0)
                if 0.0 <= obs_dist < OBSTACLE_STOP_DIST_M:
                    self.stop()
                    self._logger.warn(
                        f'Obstacle detected {direction} at '
                        f'{obs_dist * 1000:.0f}mm — stopping')
                    return False

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

    # =================================================================
    # Move Until Marker — REVISED(phase2-line-marker)
    # =================================================================

    def move_until_marker(
        self,
        vx: float = 0.10,
        vy: float = 0.0,
        wz: float = 0.0,
        timeout_s: float | None = None,
    ) -> bool:
        """Gerakkan robot dengan kecepatan tetap sampai line sensor
        mendeteksi marker posisi (garis hitam di lantai).

        Digunakan untuk:
          - Maju sampai posisi rak
          - Maju sampai posisi standbox
          - Bergerak ke titik referensi manapun

        Stop condition:
          D1=1 ATAU D2=1 → marker terdeteksi → stop, return True
          timeout_s terlewat → stop, return False (tidak ketemu marker)

        Args:
            vx: kecepatan linear x (m/s), default 0.10 maju
            vy: kecepatan linear y (m/s), default 0.0
            wz: kecepatan angular z (rad/s), default 0.0
            timeout_s: timeout, None = pakai move_timeout_s dari params

        Returns:
            True jika marker terdeteksi dan robot berhenti
            False jika timeout (marker tidak ditemukan)

        Notes:
            Tidak menggunakan trapezoidal profile —
            kecepatan konstan karena jarak tidak diketahui.
            Gunakan kecepatan rendah (≤0.10 m/s) untuk akurasi stop.
        """
        if timeout_s is None:
            timeout_s = self._timeout

        if not self._line_received:
            self._logger.warn(
                'move_until_marker: /line_sensor/state belum diterima')

        self._logger.info(
            f'Moving until marker: '
            f'vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}')

        t_start = time.monotonic()

        while True:
            elapsed = time.monotonic() - t_start

            if elapsed > timeout_s:
                self.stop()
                self._logger.warn(
                    f'move_until_marker timeout {elapsed:.1f}s '
                    f'— marker not found')
                return False

            # Stop condition: marker terdeteksi
            if self._line_marker_detected():
                self.stop()
                self._logger.info(
                    f'Marker detected! '
                    f'D1={self._line_d1} D2={self._line_d2} '
                    f'after {elapsed:.1f}s')
                return True

            # Publish kecepatan konstan
            self._publish_cmd_vel(vx, vy, wz)
            time.sleep(self._dt)

    # =================================================================
    # Approach Wall — ADDED(phase2-sensor)
    # =================================================================

    def approach_wall(
        self,
        direction: str,
        target_dist_m: float | None = None,
        speed_m_s: float | None = None,
    ) -> bool:
        """Maju ke arah dinding sampai jarak ultrasonik mencapai target.

        Modul D2 LKS: robot mendekati dinding sampai jarak tertentu, lalu berhenti.

        Robot bergerak ke arah `direction` dengan kecepatan rendah,
        terus monitoring jarak ultrasonik arah tersebut.
        Berhenti saat jarak ≤ target_dist_m.

        Args:
            direction: arah dinding ('front', 'right', 'rear', 'left')
            target_dist_m: jarak target berhenti (m). None = OBSTACLE_STOP_DIST_M
            speed_m_s: kecepatan approach (m/s). None = default_linear_speed / 2

        Returns:
            True jika target jarak tercapai, False jika timeout atau sensor invalid
        """
        if not self._odom_received:
            self._logger.warn('Approach wall aborted: odom belum diterima')
            self.stop()
            return False

        target = target_dist_m if target_dist_m is not None else OBSTACLE_STOP_DIST_M
        speed = speed_m_s if speed_m_s is not None else self._default_linear_speed / 2.0
        speed = min(abs(speed), self._max_linear_speed)

        # Map direction ke vx/vy sign
        dir_map: dict[str, tuple[float, float]] = {
            'front':  ( 1.0,  0.0),
            'rear':   (-1.0,  0.0),
            'left':   ( 0.0,  1.0),
            'right':  ( 0.0, -1.0),
        }
        if direction not in dir_map:
            self._logger.error(f'Approach wall: invalid direction "{direction}"')
            return False

        vx_sign, vy_sign = dir_map[direction]

        self._logger.info(
            f'Approach wall {direction}: target {target * 1000:.0f}mm '
            f'at {speed:.2f} m/s')

        t_start = time.monotonic()

        while True:
            elapsed = time.monotonic() - t_start
            if elapsed > self._timeout:
                self._logger.warn(
                    f'Approach wall timeout after {elapsed:.1f}s')
                self.stop()
                return False

            # Cek jarak ultrasonik di arah gerak
            obs_dist = self._obstacle_distances.get(direction, -1.0)

            if obs_dist < 0.0:
                # Sensor invalid — terus bergerak (best effort)
                pass
            elif obs_dist <= target:
                # Target tercapai
                self.stop()
                self._logger.info(
                    f'Approach wall {direction} complete: '
                    f'distance {obs_dist * 1000:.0f}mm in {elapsed:.1f}s')
                return True

            # Publish cmd_vel (kecepatan konstan, tanpa trapesoid)
            self._publish_cmd_vel(
                vx_sign * speed,
                vy_sign * speed,
                0.0,
            )

            time.sleep(self._dt)

    # =================================================================
    # Balance (Wall Alignment) — ADDED(phase2-balance)
    # =================================================================

    def balance(
        self,
        direction: str,
        stop_distance_m: float | None = None,
        speed_m_s: float | None = None,
        timeout_s: float | None = None,
    ) -> bool:
        """Rapatkan robot ke dinding di arah tertentu (wall alignment).

        Robot bergerak pelan ke arah yang ditentukan sampai sensor
        ultrasonik di arah tersebut mendeteksi jarak <= stop_distance_m.

        Digunakan sebagai self-correction posisi sebelum task presisi:
          balance('left')  → rapatkan ke dinding kiri
          balance('front') → rapatkan ke dinding depan
          dll.

        Berbeda dari approach_wall():
          - approach_wall: berhenti di OBSTACLE_STOP_DIST_M (75mm, safety)
          - balance: berhenti di BALANCE_STOP_DIST_M (50mm, merapat)

        Args:
            direction: arah balance
                       'front'/'depan'   → maju ke dinding depan
                       'rear'/'belakang' → mundur ke dinding belakang
                       'left'/'kiri'     → geser ke dinding kiri
                       'right'/'kanan'   → geser ke dinding kanan
            stop_distance_m: jarak berhenti (None = BALANCE_STOP_DIST_M)
            speed_m_s: kecepatan (None = BALANCE_APPROACH_SPEED_M_S)
            timeout_s: timeout (None = BALANCE_TIMEOUT_S)

        Returns:
            True jika berhasil merapat ke dinding
            False jika timeout (dinding tidak ditemukan)

        Notes:
            Mendukung nama arah dalam Bahasa Indonesia dan Inggris
            untuk kemudahan operator saat input di Foxglove.
        """
        # Normalize direction — support Bahasa Indonesia & English
        dir_aliases: dict[str, str] = {
            'depan':    'front',
            'belakang': 'rear',
            'kiri':     'left',
            'kanan':    'right',
            'front':    'front',
            'rear':     'rear',
            'left':     'left',
            'right':    'right',
        }
        normalized = dir_aliases.get(direction.lower())
        if normalized is None:
            self._logger.error(
                f'balance: invalid direction "{direction}". '
                f'Valid: front/depan, rear/belakang, left/kiri, right/kanan')
            return False

        stop_dist = stop_distance_m if stop_distance_m is not None \
                    else BALANCE_STOP_DIST_M
        speed = speed_m_s if speed_m_s is not None \
                else BALANCE_APPROACH_SPEED_M_S
        timeout = timeout_s if timeout_s is not None \
                  else BALANCE_TIMEOUT_S

        # Map direction ke velocity signs  # ADDED(phase2-balance)
        dir_to_vel: dict[str, tuple[float, float]] = {
            'front': (speed,  0.0),
            'rear':  (-speed, 0.0),
            'left':  (0.0,  speed),
            'right': (0.0, -speed),
        }
        vx, vy = dir_to_vel[normalized]

        self._logger.info(
            f'Balance {direction} ({normalized}): '
            f'moving until ultrasonik < {stop_dist * 1000:.0f}mm')

        t_start = time.monotonic()
        _warned_no_data = False  # ADDED(phase2-balance): warn once flag

        while True:
            elapsed = time.monotonic() - t_start

            if elapsed > timeout:
                self.stop()
                self._logger.warn(
                    f'Balance {direction} timeout {elapsed:.1f}s '
                    f'— wall not found')
                return False

            # Cek ultrasonik di arah yang dituju  # ADDED(phase2-balance)
            obs_dist = self._obstacle_distances.get(normalized, -1.0)

            if obs_dist >= 0.0 and obs_dist <= stop_dist:
                self.stop()
                self._logger.info(
                    f'Balance {direction} complete: '
                    f'{obs_dist * 1000:.0f}mm in {elapsed:.1f}s')
                return True

            if obs_dist < 0.0 and not _warned_no_data:
                self._logger.warn(
                    'balance: obstacle_distances belum tersedia, '
                    'pastikan sensor_node berjalan')
                _warned_no_data = True

            self._publish_cmd_vel(vx, vy, 0.0)
            time.sleep(self._dt)

    # =================================================================
    # Line Follow U — ADDED(phase2-d3-linefollower)
    # =================================================================

    def follow_line_u(
        self,
        speed_m_s: float | None = None,
        timeout_s: float = 30.0,
    ) -> bool:
        """Ikuti garis hitam berbentuk U untuk Modul D3 LKS.

        Line sensor sebagai line tracer:
          D1=kiri, D2=kanan
          D1=0, D2=0 → lost → recovery: putar ke arah last_error
          D1=1, D2=0 → garis di kiri → koreksi belok kiri (wz positif)
          D1=0, D2=1 → garis di kanan → koreksi belok kanan (wz negatif)
          D1=1, D2=1 → on center → jalan lurus

        PD control:
          error = D1 - D2  → range: -1, 0, +1
          correction = Kp*error + Kd*(error - prev_error)/dt
          vx = base_speed
          wz = correction

        Stop condition (selesai U):
          Lost terus menerus > lost_timeout_s → return True

        Args:
            speed_m_s: kecepatan maju (None = dari YAML base_speed)
            timeout_s: batas waktu total (s)

        Returns:
            True jika selesai (garis U habis, lost > lost_timeout_s)
            False jika timeout total
        """
        # Load params dari self._params atau defaults  # ADDED(phase2-d3-linefollower)
        base_speed = speed_m_s if speed_m_s is not None else float(
            self._params.get('line_follower_base_speed_m_s', 0.08))
        kp = float(self._params.get('line_follower_kp', 0.30))
        kd = float(self._params.get('line_follower_kd', 0.05))
        lost_timeout = float(self._params.get(
            'line_follower_lost_timeout_s', 1.5))
        recovery_wz = float(self._params.get(
            'line_follower_recovery_wz_rad_s', 0.20))

        self._logger.info(
            f'follow_line_u: speed={base_speed} kp={kp} kd={kd}')

        if not self._line_received:
            self._logger.warn(
                'follow_line_u: /line_sensor/state belum diterima')

        prev_error = 0
        last_error = 0        # untuk recovery direction
        lost_start = None     # timestamp mulai lost
        t_start = time.monotonic()

        while True:
            elapsed = time.monotonic() - t_start

            if elapsed > timeout_s:
                self.stop()
                self._logger.warn(f'follow_line_u timeout {elapsed:.1f}s')
                return False

            d1 = self._line_d1
            d2 = self._line_d2

            # LOST state: kedua sensor tidak mendeteksi garis
            if d1 == 0 and d2 == 0:
                if lost_start is None:
                    lost_start = time.monotonic()

                lost_duration = time.monotonic() - lost_start
                if lost_duration >= lost_timeout:
                    # Selesai U — garis benar-benar habis
                    self.stop()
                    self._logger.info(
                        f'follow_line_u complete: '
                        f'line lost {lost_duration:.1f}s → U done')
                    return True

                # Recovery: putar ke arah last error
                wz = recovery_wz if last_error >= 0 else -recovery_wz
                self._publish_cmd_vel(0.0, 0.0, wz)

            else:
                # Garis terdeteksi → reset lost timer
                lost_start = None

                error = d1 - d2   # -1, 0, atau +1
                dt = self._dt
                correction = kp * error + kd * (error - prev_error) / dt

                prev_error = error
                last_error = error

                self._publish_cmd_vel(base_speed, 0.0, correction)

            time.sleep(self._dt)
