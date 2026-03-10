#!/usr/bin/env python3
"""
color_detector.py — Node deteksi warna untuk Modul D4/D5/D6 LKS.

Pipeline:
    /camera/image_raw → BGR → HSV → mask per warna → contour → label
    Output: /vision/color_detection (String: "merah"/"hijau"/"biru"/"none")
           /vision/debug_image (Image: frame dengan anotasi untuk Foxglove)

Semua parameter dari camera_params.yaml dan constants.py.

Package: robot_mms
"""

# ─── Standard Library ────────────────────────────────────────────
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
    VISION_LABEL_RED,
    VISION_LABEL_GREEN,
    VISION_LABEL_BLUE,
    VISION_LABEL_NONE,
    VISION_MIN_AREA_PX,
    VISION_CONSECUTIVE_FRAMES,
)


class ColorDetectorNode(Node):
    """Node ROS2 untuk deteksi warna dominan dari kamera.

    Subscribe /camera/image_raw, publish label warna ke
    /vision/color_detection dan debug image ke /vision/debug_image.
    """

    # BGR colors untuk drawing anotasi
    _BGR_COLORS: dict[str, tuple[int, int, int]] = {
        'red':   (0, 0, 255),
        'green': (0, 255, 0),
        'blue':  (255, 0, 0),
    }

    # Mapping internal color key → VISION_LABEL_* dari constants.py
    _KEY_TO_LABEL: dict[str, str] = {
        'red':   VISION_LABEL_RED,
        'green': VISION_LABEL_GREEN,
        'blue':  VISION_LABEL_BLUE,
    }

    def __init__(self) -> None:
        """Inisialisasi node, load params, setup pub/sub."""
        super().__init__('color_detector_node')

        # Load camera_params.yaml
        yaml_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'camera_params.yaml'
        )
        self._params = self._load_params(yaml_path)

        # Extract color detection config dari YAML
        self._color_cfg: dict[str, Any] = self._params['color_detection']

        # Detection params dari YAML (fallback ke constants.py)
        det_cfg = self._params.get('detection', {})
        self._consecutive_needed: int = det_cfg.get(
            'consecutive_frames', VISION_CONSECUTIVE_FRAMES)
        camera_topic: str = det_cfg.get(
            'camera_topic', '/camera/image_raw')

        # CvBridge untuk konversi ROS Image ↔ OpenCV
        self._bridge = CvBridge()

        # Consecutive frame counter state
        self._candidate_label: str = VISION_LABEL_NONE
        self._candidate_count: int = 0
        self._confirmed_label: str = VISION_LABEL_NONE
        self._last_published: str = VISION_LABEL_NONE

        # FPS tracking
        self._prev_time: float = time.monotonic()
        self._fps: float = 0.0

        # Morphological kernel — 3×3 untuk opening (hapus noise)
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (3, 3))

        # Subscriber: /camera/image_raw
        self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        # Publisher: /vision/color_detection (label string)
        self._pub_color = self.create_publisher(
            String, '/vision/color_detection', 10)

        # Publisher: /vision/debug_image (anotasi frame)
        self._pub_debug = self.create_publisher(
            Image, '/vision/debug_image', 10)

        self.get_logger().info(
            f'ColorDetectorNode started — topic={camera_topic}, '
            f'consecutive={self._consecutive_needed}')

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
    # HSV Mask Building
    # =================================================================

    def _build_masks(self, hsv_frame: np.ndarray) -> dict[str, np.ndarray]:
        """Buat binary mask per warna dari HSV ranges di YAML.

        Merah: gabung 2 range (OR) karena hue merah di kedua ujung.
        Hijau/biru: 1 range masing-masing.

        Args:
            hsv_frame: frame HSV (H=0-179, S=0-255, V=0-255)

        Returns:
            Dict {'red': mask, 'green': mask, 'blue': mask}
        """
        masks: dict[str, np.ndarray] = {}

        for color_key in ('red', 'green', 'blue'):
            cfg = self._color_cfg[color_key]
            hsv_ranges: list[dict] = cfg['hsv_ranges']

            # Mulai dengan mask kosong
            combined_mask = np.zeros(
                hsv_frame.shape[:2], dtype=np.uint8)

            # OR semua range (merah punya 2, hijau/biru punya 1)
            for rng in hsv_ranges:
                lower = np.array(rng['lower'], dtype=np.uint8)
                upper = np.array(rng['upper'], dtype=np.uint8)
                mask = cv2.inRange(hsv_frame, lower, upper)
                combined_mask = cv2.bitwise_or(combined_mask, mask)

            masks[color_key] = combined_mask

        return masks

    # =================================================================
    # Color Detection Pipeline
    # =================================================================

    def _detect_dominant_color(
        self, frame_bgr: np.ndarray
    ) -> tuple[str, float, dict[str, np.ndarray | None]]:
        """Deteksi warna dominan dari frame BGR.

        Pipeline:
            1. BGR → HSV
            2. _build_masks() per warna
            3. Morphological opening 3×3 untuk hapus noise
            4. findContours per warna → ambil contour terbesar
            5. confidence = area_contour_terbesar / (H × W)
            6. Warna dengan confidence tertinggi = kandidat
            7. Jika confidence > threshold → return label dari YAML
            8. Semua di bawah threshold → return "none"

        Args:
            frame_bgr: frame kamera dalam format BGR

        Returns:
            tuple (label, confidence, contours_dict) dimana:
                label: "merah"/"hijau"/"biru"/"none"
                confidence: float rasio area terbesar / area frame
                contours_dict: dict color_key → contour terbesar (atau None)
        """
        h, w = frame_bgr.shape[:2]
        frame_area = h * w

        # Step 1: BGR → HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Step 2: Build mask per warna
        masks = self._build_masks(hsv)

        best_label: str = VISION_LABEL_NONE
        best_confidence: float = 0.0
        contours_dict: dict[str, np.ndarray | None] = {
            'red': None, 'green': None, 'blue': None}

        for color_key, mask in masks.items():
            # Step 3: Morphological opening — hapus noise kecil
            opened = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self._morph_kernel)

            # Step 4: findContours → ambil contour terbesar
            contours, _ = cv2.findContours(
                opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            # Filter area minimum dari YAML
            min_area = self._color_cfg[color_key].get(
                'min_area_px', VISION_MIN_AREA_PX)
            if area < min_area:
                continue

            # Step 5: confidence = area / frame_area
            confidence = area / frame_area

            contours_dict[color_key] = largest

            # Step 6: Track warna dengan confidence tertinggi
            if confidence > best_confidence:
                best_confidence = confidence

                # Step 7: Cek threshold dari YAML
                threshold = self._color_cfg[color_key].get(
                    'confidence_threshold', 0.02)
                if confidence >= threshold:
                    # Gunakan label dari YAML (merah/hijau/biru)
                    best_label = self._color_cfg[color_key]['label']

        # Step 8: Jika tidak ada yang lolos threshold → "none"
        return best_label, best_confidence, contours_dict

    # =================================================================
    # Consecutive Frame Confirmation
    # =================================================================

    def _confirm_detection(self, raw_label: str) -> str:
        """Konfirmasi deteksi dengan consecutive frame counter.

        Increment counter jika label sama dengan candidate.
        Reset jika label berubah.
        Return confirmed label setelah CONSECUTIVE_FRAMES berturut-turut.

        Args:
            raw_label: label mentah dari _detect_dominant_color()

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

    def _draw_debug(
        self,
        frame_bgr: np.ndarray,
        label: str,
        confidence: float,
        contours_dict: dict[str, np.ndarray | None],
    ) -> np.ndarray:
        """Anotasi frame untuk debug di Foxglove.

        Elemen:
            - Bounding box contour terbesar (warna BGR sesuai deteksi)
            - Text kiri atas: "DETECTED: merah (0.142)" atau "DETECTING..."
            - Confidence bar horizontal di bawah text
            - FPS counter kanan atas

        Args:
            frame_bgr: frame asli BGR
            label: label confirmed ("merah"/"hijau"/"biru"/"none")
            confidence: float confidence value
            contours_dict: dict color_key → contour terbesar

        Returns:
            Frame BGR dengan anotasi
        """
        out = frame_bgr.copy()
        h, w = out.shape[:2]

        # Draw bounding box untuk setiap contour yang terdeteksi
        for color_key, contour in contours_dict.items():
            if contour is None:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            bgr = self._BGR_COLORS.get(color_key, (255, 255, 255))
            cv2.rectangle(out, (x, y), (x + bw, y + bh), bgr, 2)

        # Text kiri atas: status deteksi
        if label != VISION_LABEL_NONE:
            text = f'DETECTED: {label} ({confidence:.3f})'
            text_color = (0, 255, 0)
        else:
            text = 'DETECTING...'
            text_color = (0, 255, 255)

        cv2.putText(out, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

        # Confidence bar — di bawah text
        bar_width = int(min(confidence, 1.0) * (w - 20))
        cv2.rectangle(out, (10, 40), (10 + bar_width, 55),
                      text_color, -1)
        cv2.rectangle(out, (10, 40), (w - 10, 55),
                      (128, 128, 128), 1)

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
            2. _detect_dominant_color()
            3. _confirm_detection()
            4. Jika confirmed label berubah → publish /vision/color_detection
            5. _draw_debug()
            6. Publish /vision/debug_image

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
            # Step 2: Deteksi warna dominan
            raw_label, confidence, contours_dict = (
                self._detect_dominant_color(frame_bgr))

            # Step 3: Konfirmasi dengan consecutive frames
            confirmed = self._confirm_detection(raw_label)

            # Step 4: Publish jika label berubah
            if confirmed != self._last_published:
                color_msg = String()
                color_msg.data = confirmed
                self._pub_color.publish(color_msg)
                self.get_logger().info(
                    f'Color: {confirmed} (conf={confidence:.3f})')
                self._last_published = confirmed

            # Step 5: Draw debug anotasi
            debug_frame = self._draw_debug(
                frame_bgr, confirmed, confidence, contours_dict)

            # Step 6: Publish debug image
            debug_msg = self._bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
            debug_msg.header = msg.header
            self._pub_debug.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')


def main(args: list[str] | None = None) -> None:
    """Entry point — init, spin, shutdown."""
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
