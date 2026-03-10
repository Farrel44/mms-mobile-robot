#!/usr/bin/env python3
"""
sensor_node.py — ROS2 Node untuk pemrosesan data sensor ultrasonik.

Subscribe ke topic /ultrasonic/* dari mms_bridge,
lalu publish status obstacle dan jarak per arah.

Topics subscribed:
    /ultrasonic/front  (sensor_msgs/Range)
    /ultrasonic/right  (sensor_msgs/Range)
    /ultrasonic/rear   (sensor_msgs/Range)
    /ultrasonic/left   (sensor_msgs/Range)

Topics published:
    /obstacle/detected    (std_msgs/Bool)       — True jika ada obstacle < OBSTACLE_STOP_DIST_M
    /obstacle/distances   (std_msgs/Float32MultiArray) — [front, right, rear, left] meter
    /line_sensor/state    (std_msgs/Int8MultiArray)    — [D1, D2] state (belum aktif, placeholder)

Package: robot_mms
"""

# ─── Standard Library ────────────────────────────────────────────
import time

# ─── Third Party ─────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32MultiArray, Int8MultiArray

# ─── Project ─────────────────────────────────────────────────────
from .constants import (
    ULTRASONIC_DIRECTION_NAMES,
    ULTRASONIC_MIN_DIST_M,
    ULTRASONIC_MAX_DIST_M,
    OBSTACLE_STOP_DIST_M,
    LINE_SENSOR_COUNT,
    LINE_STATE_OFF,
)

# Timeout: jika ultrasonik tidak update > 1 detik, anggap stale
_STALENESS_TIMEOUT_S = 1.0


class ObstacleSensorNode(Node):
    """ROS2 Node untuk pemrosesan sensor ultrasonik dan obstacle detection.

    Subscribe ke 4 topic Range dari mms_bridge, lalu publish:
      - /obstacle/detected  (Bool)  — True jika ada arah dengan jarak < threshold
      - /obstacle/distances (Float32MultiArray) — [front, right, rear, left] meter
      - /line_sensor/state  (Int8MultiArray) — placeholder untuk line sensor

    Timer 10 Hz melakukan pengecekan staleness dan publish status.
    """

    def __init__(self) -> None:
        """Inisialisasi ObstacleSensorNode."""
        super().__init__('obstacle_sensor_node')

        # ─── Internal state ──────────────────────────────────────
        # Jarak per arah (meter). -1.0 = invalid/belum diterima
        self._distances: dict[str, float] = {
            d: -1.0 for d in ULTRASONIC_DIRECTION_NAMES
        }
        # Timestamp terakhir update per arah (monotonic)
        self._last_update: dict[str, float] = {
            d: 0.0 for d in ULTRASONIC_DIRECTION_NAMES
        }
        # Line sensor state (placeholder, belum ada hardware)
        self._line_state: list[int] = [LINE_STATE_OFF] * LINE_SENSOR_COUNT

        # ─── Subscribers: /ultrasonic/<direction> ────────────────
        for direction in ULTRASONIC_DIRECTION_NAMES:
            self.create_subscription(
                Range,
                f'/ultrasonic/{direction}',
                lambda msg, d=direction: self._on_ultrasonic(msg, d),
                qos_profile_sensor_data,
            )

        # ─── Publishers ─────────────────────────────────────────
        self._obstacle_detected_pub = self.create_publisher(
            Bool, '/obstacle/detected', 10)
        self._obstacle_dist_pub = self.create_publisher(
            Float32MultiArray, '/obstacle/distances', 10)
        self._line_state_pub = self.create_publisher(
            Int8MultiArray, '/line_sensor/state', 10)

        # ─── Timer 10 Hz ─────────────────────────────────────────
        self.create_timer(0.1, self._timer_callback)

        self.get_logger().info(
            f'ObstacleSensorNode siap — threshold {OBSTACLE_STOP_DIST_M * 1000:.0f}mm')

    # =================================================================
    # Callback: ultrasonic Range
    # =================================================================

    def _on_ultrasonic(self, msg: Range, direction: str) -> None:
        """Callback untuk setiap topic /ultrasonic/<direction>.

        Menyimpan jarak valid ke internal state.
        Jarak -1.0 dari mms_bridge berarti sensor invalid.

        Args:
            msg: sensor_msgs/Range dari mms_bridge
            direction: nama arah ('front', 'right', 'rear', 'left')
        """
        if msg.range < 0.0:
            # Sensor invalid — tandai -1.0
            self._distances[direction] = -1.0
        else:
            self._distances[direction] = msg.range
        self._last_update[direction] = time.monotonic()

    # =================================================================
    # Timer: staleness check + publish
    # =================================================================

    def _timer_callback(self) -> None:
        """Timer 10 Hz: cek staleness, publish obstacle status dan line sensor."""
        self._check_staleness()
        self._publish_obstacle_status()
        self._publish_line_state()

    def _check_staleness(self) -> None:
        """Tandai sensor sebagai invalid jika data terlalu lama (> 1 detik).

        Mencegah keputusan obstacle berdasarkan data basi.
        """
        now = time.monotonic()
        for direction in ULTRASONIC_DIRECTION_NAMES:
            last = self._last_update[direction]
            if last > 0.0 and (now - last) > _STALENESS_TIMEOUT_S:
                if self._distances[direction] >= 0.0:
                    self.get_logger().warn(
                        f'Ultrasonic {direction} stale — marking invalid')
                self._distances[direction] = -1.0

    # =================================================================
    # Publish: obstacle status
    # =================================================================

    def _publish_obstacle_status(self) -> None:
        """Publish /obstacle/detected dan /obstacle/distances.

        /obstacle/detected = True jika ada arah dengan jarak valid
        yang kurang dari OBSTACLE_STOP_DIST_M.

        /obstacle/distances = [front, right, rear, left] dalam meter.
        Sensor invalid → -1.0.
        """
        # Build distances array (urutan: front, right, rear, left)
        dist_array: list[float] = [
            self._distances[d] for d in ULTRASONIC_DIRECTION_NAMES
        ]

        # Cek obstacle: ada arah valid dengan jarak < threshold
        detected = any(
            0.0 <= d < OBSTACLE_STOP_DIST_M
            for d in dist_array
        )

        # Publish Bool
        bool_msg = Bool()
        bool_msg.data = detected
        self._obstacle_detected_pub.publish(bool_msg)

        # Publish Float32MultiArray
        dist_msg = Float32MultiArray()
        dist_msg.data = [float(d) for d in dist_array]
        self._obstacle_dist_pub.publish(dist_msg)

    # =================================================================
    # Publish: line sensor (placeholder)
    # =================================================================

    def _publish_line_state(self) -> None:
        """Publish /line_sensor/state.

        Saat ini placeholder — selalu [0, 0] (LINE_STATE_OFF).
        Akan diupdate ketika line sensor diintegrasikan ke serial protocol.
        """
        msg = Int8MultiArray()
        msg.data = list(self._line_state)
        self._line_state_pub.publish(msg)


# =====================================================================
# Entry point
# =====================================================================
def main(args=None) -> None:
    """Entry point untuk ros2 run robot_mms sensor_node."""
    rclpy.init(args=args)
    node = ObstacleSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ObstacleSensorNode...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
