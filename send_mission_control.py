#!/usr/bin/env python3
"""Send a /mission/control JSON command and (optionally) watch /mission/status.

This is a helper for testing the mission_sequencer without fighting shell quoting.

Examples:
  python3 send_mission_control.py --example rotate_cw
  python3 send_mission_control.py --file missions/demo.json

Notes:
- Topics:
  - Publishes: /mission/control (std_msgs/String)
  - Subscribes: /mission/status (std_msgs/String)
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


EXAMPLES = {
    "rotate_cw": {
        "cmd": "EXECUTE_RAW",
        "steps": [
            {"cmd": "PUTAR_KANAN", "x": 0, "y": 0, "w": 0.0, "exit": {"mode": "TIME", "op": ">=", "val": 200}},
            {"cmd": "PUTAR_KANAN", "x": 0, "y": 0, "w": 0.6, "exit": {"mode": "TIME", "op": ">=", "val": 2000}},
            {"cmd": "STOP", "x": 0, "y": 0, "w": 0, "exit": {"mode": "TIME", "op": ">=", "val": 800}},
        ],
    },
    "maju_stop": {
        "cmd": "EXECUTE_RAW",
        "steps": [
            {"cmd": "MAJU", "x": 0.2, "y": 0, "w": 0, "exit": {"mode": "TIME", "op": ">=", "val": 1500}},
            {"cmd": "STOP", "x": 0, "y": 0, "w": 0, "exit": {"mode": "TIME", "op": ">=", "val": 1000}},
        ],
    },
}


class MissionControlSender(Node):
    def __init__(self, watch_status: bool, watch_seconds: float):
        super().__init__("mission_control_sender")
        self.pub = self.create_publisher(String, "/mission/control", 10)
        self.watch_status = watch_status
        self.watch_seconds = watch_seconds
        self.last_status_msg = None

        if self.watch_status:
            self.create_subscription(String, "/mission/status", self._on_status, 10)

    def _on_status(self, msg: String):
        self.last_status_msg = msg.data
        self.get_logger().info(f"/mission/status: {msg.data}")

    def publish_control(self, payload: dict, repeats: int = 3):
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))

        # Publish a few times to reduce chance of missing on startup.
        repeats = int(repeats)
        if repeats < 1:
            repeats = 1
        for _ in range(repeats):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def watch_status_for(self, seconds: float):
        if not self.watch_status:
            return

        end_t = time.time() + float(seconds)
        while time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.2)

    def send(self, payload: dict):
        self.publish_control(payload)

        self.get_logger().info("Sent /mission/control")
        self.watch_status_for(self.watch_seconds)


def _load_payload_from_file(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main() -> int:
    parser = argparse.ArgumentParser()
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--example", choices=sorted(EXAMPLES.keys()))
    src.add_argument("--file", help="Path to a JSON file containing the /mission/control payload")
    src.add_argument("--json", help="Inline JSON payload string")

    run_group = parser.add_mutually_exclusive_group()
    run_group.add_argument(
        "--run",
        action="store_true",
        help="After sending an EXECUTE_RAW payload, also send {cmd: RUN} to start execution",
    )
    run_group.add_argument(
        "--no-run",
        action="store_true",
        help="Do not auto-send RUN (only load steps / update state)",
    )
    parser.add_argument(
        "--run-delay",
        type=float,
        default=0.15,
        help="Delay (seconds) between sending EXECUTE_RAW and RUN",
    )

    parser.add_argument("--watch-status", action="store_true", help="Echo /mission/status messages")
    parser.add_argument("--watch-seconds", type=float, default=6.0)

    args = parser.parse_args()

    if args.example:
        payload = EXAMPLES[args.example]
        auto_run = not bool(args.no_run)
    elif args.file:
        payload = _load_payload_from_file(args.file)
        auto_run = bool(args.run)
    else:
        try:
            payload = json.loads(args.json)
        except json.JSONDecodeError as e:
            print(f"Invalid JSON: {e}", file=sys.stderr)
            return 2
        auto_run = bool(args.run)

    rclpy.init()
    node = MissionControlSender(watch_status=bool(args.watch_status), watch_seconds=args.watch_seconds)

    try:
        # Always publish the main payload first.
        node.publish_control(payload)

        cmd = str(payload.get("cmd", "")).upper()
        if auto_run and cmd == "EXECUTE_RAW":
            delay_s = max(0.0, float(args.run_delay))
            if delay_s > 0.0:
                time.sleep(delay_s)
            # RUN should be idempotent; publish once to avoid noisy warnings.
            node.publish_control({"cmd": "RUN"}, repeats=1)

        node.get_logger().info("Sent /mission/control")
        node.watch_status_for(args.watch_seconds)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
