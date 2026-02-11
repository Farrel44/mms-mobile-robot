#!/usr/bin/env python3
"""Estimate wheel RPM from /encoder_ticks (delta ticks per packet).

Use this to sanity-check that the firmware PID + ramp is behaving smoothly.
It converts delta ticks to RPM using dt between messages.

Usage:
  python3 check_wheel_rpm.py

Notes:
- /encoder_ticks from robot_mms is delta ticks (not absolute).
- For stable estimates, keep robot lifted or on low friction.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray


class WheelRpmEstimator(Node):
    def __init__(self):
        super().__init__("wheel_rpm_estimator")

        self.declare_parameter("ticks_per_rev", 380)
        self.ticks_per_rev = int(self.get_parameter("ticks_per_rev").value or 380)

        self._last_stamp_ns = None
        self._latest_rpm = [0.0, 0.0, 0.0]
        self._last_print_ns = 0

        self.create_subscription(
            Float32MultiArray,
            "/encoder_ticks",
            self._on_encoder,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Listening /encoder_ticks; ticks_per_rev={self.ticks_per_rev}"
        )

    def _on_encoder(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_stamp_ns is None:
            self._last_stamp_ns = now_ns
            return

        dt = (now_ns - self._last_stamp_ns) / 1e9
        self._last_stamp_ns = now_ns
        if dt <= 0.0 or dt > 0.5:
            return

        # ticks/s -> rev/s -> rpm
        scale = 60.0 / float(self.ticks_per_rev)
        self._latest_rpm = [
            float(msg.data[0]) / dt * scale,
            float(msg.data[1]) / dt * scale,
            float(msg.data[2]) / dt * scale,
        ]

        # Print at ~2 Hz
        if (now_ns - self._last_print_ns) >= int(0.5 * 1e9):
            self._last_print_ns = now_ns
            r1, r2, r3 = self._latest_rpm
            self.get_logger().info(
                f"Est RPM: M1={r1:+7.1f}  M2={r2:+7.1f}  M3={r3:+7.1f}"
            )


def main():
    rclpy.init()
    node = WheelRpmEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
