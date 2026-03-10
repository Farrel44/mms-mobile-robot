#!/usr/bin/env python3
"""
shape_detector.py — Node deteksi bentuk+warna untuk Modul D7/D8 LKS.

Pipeline:
    /camera/image_raw → BGR → HSV → mask merah/hijau → contour
    → circularity check → "O-merah" / "O-hijau" / "none"

Output:
    /vision/shape_detection (String): label bentuk+warna
    /vision/shape_debug_image (Image): frame anotasi untuk Foxglove

Semua parameter dari camera_params.yaml dan constants.py.

Package: robot_mms
"""

# ─── Standard Library ────────────────────────────────────────────
import math
import os
import time
from typing import Any

# ─── Third Party ─────────────────────────────────────────────────
import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# ─── Local ───────────────────────────────────────────────────────
from robot_mms.constants import (
    VISION_LABEL_CIRCLE_RED,
    VISION_LABEL_CIRCLE_GREEN,
    VISION_LABEL_NONE,
    VISION_CIRCULARITY_THRESHOLD,
    VISION_CONSECUTIVE_FRAMES,
    VISION_MIN_AREA_PX,
)


class ShapeDetectorNode(Node):
    """Node ROS2 untuk deteksi bentuk+warna (D7: O-merah, D8: O-hijau).

    Subscribe /camera/image_raw, deteksi lingkaran merah/hijau,
    publish label ke /vision/shape_detection dan debug image
    ke /vision/shape_debug_image.
    """

    # Warna yang relevan untuk D7-D8 (merah dan hijau saja)
    _RELEVANT_COLORS: tuple[str, ...] = ('red', 'green')

    # Mapping color key → shape+color label dari constants.py
    _KEY_TO_SHAPE_LABEL: dict[str, str] = {
        'red':   VISION_LABEL_CIRCLE_RED,
        'green': VISION_LABEL_CIRCLE_GREEN,
    }

    # BGR colors untuk drawing anotasi
    _BGR_COLORS: dict[str, tuple[int, int, int]] = {
        'red':   (0, 0, 255),
        'green': (0, 255, 0),
    }

    def __init__(self) -> None:
        """Inisialisasi node, load params, setup pub/sub."""
        super().__init__('shape_detector_node')

        # Load camera_params.yaml
        yaml_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'camera_params.yaml'
        )
        self._params = self._load_params(yaml_path)

        # Extract config sections
        self._color_cfg: dict[str, Any] = self._params['color_detection']
        shape_cfg: dict[str, Any] = self._params.get('shape_detection', {})

        # Shape detection thresholds dari YAML (fallback ke constants.py)
        self._circularity_threshold: float = shape_cfg.get(
            'circularity_threshold', VISION_CIRCULARITY_THRESHOLD)
        self._min_contour_area: int = shape_cfg.get(
            'min_contour_area', 800)

        # Detection params
        det_cfg = self._params.get('detection', {})
        self._consecutive_needed: int = det_cfg.get(
            'consecutive_frames', VISION_CONSECUTIVE_FRAMES)
        camera_topic: str = det_cfg.get(
            'camera_topic', '/camera/image_raw')

        # CvBridge
        self._bridge = CvBridge()

        # Consecutive frame counter state
        self._candidate_label: str = VISION_LABEL_NONE
        self._candidate_count: int = 0
        self._confirmed_label: str = VISION_LABEL_NONE
        self._last_published: str = VISION_LABEL_NONE

        # FPS tracking
        self._prev_time: float = time.monotonic()
        self._fps: float = 0.0

        # Morphological kernel — 3×3 untuk opening
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (3, 3))

        # Last detection state untuk debug drawing
        self._last_contour: np.ndarray | None = None
        self._last_shape_type: str = 'unknown'
        self._last_circularity: float = 0.0

        # Subscriber: /camera/image_raw
        self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        # Publisher: /vision/shape_detection (label string)
        self._pub_shape = self.create_publisher(
            String, '/vision/shape_detection', 10)

        # Publisher: /vision/shape_debug_image (anotasi frame)
        self._pub_debug = self.create_publisher(
            Image, '/vision/shape_debug_image', 10)

        self.get_logger().info(
            f'ShapeDetectorNode started — topic={camera_topic}, '
            f'circularity_threshold={self._circularity_threshold}')

    # =================================================================
    # Parameter Loading
    # =================================================================

    @staticmethod
    def _load_params(yaml_path: str) -> dict:
        """Load camera_params.yaml dan return dict.

        Args:
            yaml_path: path absolut ke camera_params.yaml

        Returns:
            Dict isi YAML

        Raises:
            RuntimeError: jika file tidak ditemukan
        """
        resolved = os.path.abspath(yaml_path)
        if not os.path.isfile(resolved):
            raise RuntimeError(
                f'camera_params.yaml not found: {resolved}')

        with open(resolved, 'r') as f:
            data = yaml.safe_load(f)

        return data

    # =================================================================
    # HSV Mask Building (reuse logika dari color_detector)
    # =================================================================

    def _build_color_mask(
        self, hsv_frame: np.ndarray, color_key: str
    ) -> np.ndarray:
        """Buat binary mask untuk satu warna dari HSV ranges di YAML.

        Args:
            hsv_frame: frame HSV (H=0-179, S=0-255, V=0-255)
            color_key: 'red' atau 'green'

        Returns:
            Binary mask (uint8, 0/255)
        """
        cfg = self._color_cfg[color_key]
        hsv_ranges: list[dict] = cfg['hsv_ranges']

        # Mulai dengan mask kosong
        combined = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)

        # OR semua range (merah punya 2 range, hijau punya 1)
        for rng in hsv_ranges:
            lower = np.array(rng['lower'], dtype=np.uint8)
            upper = np.array(rng['upper'], dtype=np.uint8)
            mask = cv2.inRange(hsv_frame, lower, upper)
            combined = cv2.bitwise_or(combined, mask)

        return combined

    # =================================================================
    # Shape Detection in Mask
    # =================================================================

    def _detect_shape_in_mask(
        self, mask: np.ndarray, min_area: int
    ) -> str:
        """Deteksi bentuk dominan dalam binary mask.

        Algoritma:
            1. findContours dari mask
            2. Filter contour dengan area < min_area
            3. Untuk contour terbesar:
               a. Circularity: C = (4*pi*area) / (perimeter^2)
                  C > threshold → "circle"
               b. Fallback: approxPolyDP
                  4 vertex → "square", 3 vertex → "triangle"
            4. Return: "circle" / "square" / "triangle" / "unknown"

        Args:
            mask: binary mask (uint8)
            min_area: area minimum contour untuk diproses

        Returns:
            String bentuk: "circle" / "square" / "triangle" / "unknown"
        """
        # Step 1: findContours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return 'unknown'

        # Step 2: Filter area minimum → ambil contour terbesar
        valid = [c for c in contours if cv2.contourArea(c) >= min_area]
        if not valid:
            return 'unknown'

        largest = max(valid, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        perimeter = cv2.arcLength(largest, closed=True)

        # Simpan contour untuk debug drawing
        self._last_contour = largest

        # Step 3a: Circularity check — paling reliable untuk lingkaran
        if perimeter > 0:
            circularity = (4.0 * math.pi * area) / (perimeter * perimeter)
            self._last_circularity = circularity

            if circularity > self._circularity_threshold:
                self._last_shape_type = 'circle'
                return 'circle'

        # Step 3b: Fallback — approxPolyDP untuk kotak/segitiga
        epsilon = 0.04 * perimeter
        approx = cv2.approxPolyDP(largest, epsilon, closed=True)
        num_vertices = len(approx)

        if num_vertices == 3:
            self._last_shape_type = 'triangle'
            return 'triangle'
        elif num_vertices == 4:
            self._last_shape_type = 'square'
            return 'square'

        # Step 4: Tidak teridentifikasi
        self._last_shape_type = 'unknown'
        return 'unknown'

    # =================================================================
    # Shape + Color Detection Pipeline
    # =================================================================

    def _detect_shape_color(
        self, frame_bgr: np.ndarray
    ) -> tuple[str, float]:
        """Deteksi kombinasi bentuk+warna untuk D7-D8.

        Pipeline:
            1. BGR → HSV → mask merah dan hijau
            2. Untuk setiap warna (merah dulu, lalu hijau):
               a. Morphological opening → hapus noise
               b. Cari contour terbesar
               c. Jika area > min_contour_area:
                  → _detect_shape_in_mask()
                  → Jika shape == "circle" → return label+confidence
            3. Tidak ada lingkaran → return ("none", 0.0)

        Args:
            frame_bgr: frame kamera dalam format BGR

        Returns:
            (label, confidence) dimana:
                label: VISION_LABEL_CIRCLE_RED / VISION_LABEL_CIRCLE_GREEN
                       / VISION_LABEL_NONE
                confidence: float area ratio
        """
        h, w = frame_bgr.shape[:2]
        frame_area = h * w

        # Step 1: BGR → HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Reset last detection state
        self._last_contour = None
        self._last_shape_type = 'unknown'
        self._last_circularity = 0.0

        # Step 2: Iterate merah lalu hijau
        for color_key in self._RELEVANT_COLORS:
            # Build mask untuk warna ini
            mask = self._build_color_mask(hsv, color_key)

            # Morphological opening — hapus noise kecil
            opened = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self._morph_kernel)

            # Cari contour terbesar
            contours, _ = cv2.findContours(
                opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            # Filter area minimum dari YAML shape_detection
            if area < self._min_contour_area:
                continue

            # Deteksi bentuk di dalam mask warna
            shape = self._detect_shape_in_mask(opened, self._min_contour_area)

            if shape == 'circle':
                # Confidence = area ratio
                confidence = area / frame_area
                # Label dari constants.py
                label = self._KEY_TO_SHAPE_LABEL[color_key]
                return label, confidence

        # Step 3: Tidak ada lingkaran terdeteksi
        return VISION_LABEL_NONE, 0.0

    # =================================================================
    # Consecutive Frame Confirmation
    # =================================================================

    def _confirm_shape(self, raw_label: str) -> str:
        """Konfirmasi deteksi bentuk dengan consecutive frame counter.

        Increment counter jika label sama dengan candidate.
        Reset jika label berubah.
        Return confirmed label setelah CONSECUTIVE_FRAMES berturut-turut.

        Args:
            raw_label: label mentah dari _detect_shape_color()

        Returns:
            Label yang sudah dikonfirmasi, atau "none" jika belum cukup
        """
        if raw_label == self._candidate_label:
            # Label sama — increment counter
            self._candidate_count += 1
        else:
            # Label berubah — reset counter
            self._candidate_label = raw_label
            self._candidate_count = 1

        # Cek apakah sudah cukup berturut-turut
        if self._candidate_count >= self._consecutive_needed:
            self._confirmed_label = self._candidate_label
            return self._confirmed_label

        # Belum cukup — return label terakhir yang confirmed
        return self._confirmed_label

    # =================================================================
    # Debug Image Drawing
    # =================================================================

    def _draw_shape_debug(
        self,
        frame_bgr: np.ndarray,
        label: str,
        contour: np.ndarray | None,
        shape_type: str,
    ) -> np.ndarray:
        """Anotasi frame untuk debug bentuk di Foxglove.

        Elemen:
            - Contour lingkaran yang terdeteksi (hijau terang)
            - Bounding circle (minEnclosingCircle) overlay
            - Text kiri atas: "SHAPE: O-merah (circle)" atau "SHAPE: none"
            - Circularity value: "C=0.89"
            - FPS kanan atas

        Args:
            frame_bgr: frame asli BGR
            label: label confirmed ("O-merah"/"O-hijau"/"none")
            contour: contour terbesar yang terdeteksi (atau None)
            shape_type: "circle"/"square"/"triangle"/"unknown"

        Returns:
            Frame BGR dengan anotasi
        """
        out = frame_bgr.copy()
        h, w = out.shape[:2]

        # Draw contour dan bounding circle jika ada
        if contour is not None:
            # Draw contour outline (hijau terang)
            cv2.drawContours(out, [contour], -1, (0, 255, 128), 2)

            # Draw bounding circle overlay
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            center = (int(cx), int(cy))
            cv2.circle(out, center, int(radius), (255, 255, 0), 2)

        # Text kiri atas: status deteksi
        if label != VISION_LABEL_NONE:
            text = f'SHAPE: {label} ({shape_type})'
            text_color = (0, 255, 0)
        else:
            text = 'SHAPE: none'
            text_color = (0, 255, 255)

        cv2.putText(out, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

        # Circularity value
        circ_text = f'C={self._last_circularity:.2f}'
        cv2.putText(out, circ_text, (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # FPS counter — kanan atas
        now = time.monotonic()
        dt = now - self._prev_time
        if dt > 0:
            self._fps = 1.0 / dt
        self._prev_time = now

        fps_text = f'FPS: {self._fps:.0f}'
        cv2.putText(out, fps_text, (w - 120, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return out

    # =================================================================
    # Image Callback
    # =================================================================

    def image_callback(self, msg: Image) -> None:
        """Callback untuk /camera/image_raw.

        Pipeline:
            1. CvBridge → BGR numpy
            2. _detect_shape_color()
            3. _confirm_shape()
            4. Jika confirmed label berubah → publish /vision/shape_detection
            5. _draw_shape_debug()
            6. Publish /vision/shape_debug_image

        Args:
            msg: ROS2 sensor_msgs/Image
        """
        try:
            # Step 1: Konversi ROS Image → BGR numpy
            frame_bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        try:
            # Step 2: Deteksi bentuk+warna
            raw_label, confidence = self._detect_shape_color(frame_bgr)

            # Step 3: Konfirmasi dengan consecutive frames
            confirmed = self._confirm_shape(raw_label)

            # Step 4: Publish jika label berubah
            if confirmed != self._last_published:
                shape_msg = String()
                shape_msg.data = confirmed
                self._pub_shape.publish(shape_msg)
                self.get_logger().info(
                    f'Shape: {confirmed} '
                    f'(C={self._last_circularity:.2f})')
                self._last_published = confirmed

            # Step 5: Draw debug anotasi
            debug_frame = self._draw_shape_debug(
                frame_bgr, confirmed,
                self._last_contour, self._last_shape_type)

            # Step 6: Publish debug image
            debug_msg = self._bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
            debug_msg.header = msg.header
            self._pub_debug.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Shape detection error: {e}')


def main(args: list[str] | None = None) -> None:
    """Entry point — init, spin, shutdown."""
    rclpy.init(args=args)
    node = ShapeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
