#!/usr/bin/env python3
"""
color_calibration_tool.py — Tool kalibrasi HSV standalone untuk LKS.

Standalone tool (TANPA ROS2). Hanya butuh: cv2, numpy, yaml, argparse.

Penggunaan:
    python3 tools/color_calibration_tool.py
    python3 tools/color_calibration_tool.py --camera 0 --yaml robot_mms/config/camera_params.yaml

Kontrol:
    Trackbar 'Color' : 0=Red_Low, 1=Red_High, 2=Green, 3=Blue
    S                : Simpan ke YAML
    R                : Reset dari YAML
    Q / ESC          : Keluar
"""

import argparse
import os
import sys
from typing import Any

import cv2
import numpy as np
import yaml

# ── Color slot definitions ──────────────────────────────────────
_COLOR_SLOTS = {
    0: ('red',   0, 'Red Low  (H 0-10)'),
    1: ('red',   1, 'Red High (H 165-179)'),
    2: ('green', 0, 'Green'),
    3: ('blue',  0, 'Blue'),
}


def load_hsv_from_yaml(yaml_path: str) -> dict[str, list[dict[str, list[int]]]]:
    """Load HSV ranges dari camera_params.yaml.

    Args:
        yaml_path: path ke camera_params.yaml

    Returns:
        dict color_name → list of {lower, upper}
    """
    with open(yaml_path) as f:
        params = yaml.safe_load(f)

    color_cfg = params.get('color_detection', {})
    hsv_data: dict[str, list[dict[str, list[int]]]] = {}

    for color_name in ('red', 'green', 'blue'):
        hsv_data[color_name] = []
        for r in color_cfg.get(color_name, {}).get('hsv_ranges', []):
            hsv_data[color_name].append({
                'lower': list(r['lower']),
                'upper': list(r['upper']),
            })

    return hsv_data


def save_hsv_to_yaml(
    yaml_path: str,
    hsv_data: dict[str, list[dict[str, list[int]]]],
) -> None:
    """Simpan HSV ranges kembali ke camera_params.yaml.

    Args:
        yaml_path: path ke camera_params.yaml
        hsv_data: dict color_name → list of {lower, upper}
    """
    with open(yaml_path) as f:
        params = yaml.safe_load(f)

    for color_name in ('red', 'green', 'blue'):
        if color_name in params.get('color_detection', {}):
            params['color_detection'][color_name]['hsv_ranges'] = \
                hsv_data[color_name]

    with open(yaml_path, 'w') as f:
        yaml.dump(params, f, default_flow_style=False, sort_keys=False)

    print(f'[SAVED] HSV ranges ditulis ke {yaml_path}')


def _noop(_: Any) -> None:
    """Callback kosong untuk trackbar."""


def run_calibration(camera_id: int, yaml_path: str) -> None:
    """Jalankan loop kalibrasi HSV interaktif.

    Args:
        camera_id: ID kamera (biasanya 0)
        yaml_path: path ke camera_params.yaml
    """
    # Load existing values
    hsv_data = load_hsv_from_yaml(yaml_path)

    # Create windows
    cv2.namedWindow('Original', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('HSV Mask', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Trackbars', 400, 300)

    # Create trackbars
    cv2.createTrackbar('Color', 'Trackbars', 0, 3, _noop)
    cv2.createTrackbar('H min', 'Trackbars', 0, 179, _noop)
    cv2.createTrackbar('H max', 'Trackbars', 179, 179, _noop)
    cv2.createTrackbar('S min', 'Trackbars', 0, 255, _noop)
    cv2.createTrackbar('S max', 'Trackbars', 255, 255, _noop)
    cv2.createTrackbar('V min', 'Trackbars', 0, 255, _noop)
    cv2.createTrackbar('V max', 'Trackbars', 255, 255, _noop)

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f'[ERROR] Kamera {camera_id} tidak bisa dibuka!')
        return

    prev_slot = -1

    print('\n=== Color Calibration Tool ===')
    print('Trackbar Color: 0=Red_Low  1=Red_High  2=Green  3=Blue')
    print('S=Save  R=Reset  Q/ESC=Quit')
    print('=' * 35)

    while True:
        ret, frame = cap.read()
        if not ret:
            print('[WARN] Frame tidak terbaca, skip...')
            continue

        # Read trackbar values
        slot = cv2.getTrackbarPos('Color', 'Trackbars')
        color_name, range_idx, slot_label = _COLOR_SLOTS.get(
            slot, ('red', 0, 'Red Low'))

        # When slot changes, load values from hsv_data into trackbars
        if slot != prev_slot:
            prev_slot = slot
            if range_idx < len(hsv_data.get(color_name, [])):
                r = hsv_data[color_name][range_idx]
                cv2.setTrackbarPos('H min', 'Trackbars', r['lower'][0])
                cv2.setTrackbarPos('H max', 'Trackbars', r['upper'][0])
                cv2.setTrackbarPos('S min', 'Trackbars', r['lower'][1])
                cv2.setTrackbarPos('S max', 'Trackbars', r['upper'][1])
                cv2.setTrackbarPos('V min', 'Trackbars', r['lower'][2])
                cv2.setTrackbarPos('V max', 'Trackbars', r['upper'][2])
            print(f'[SLOT] {slot_label}')

        h_min = cv2.getTrackbarPos('H min', 'Trackbars')
        h_max = cv2.getTrackbarPos('H max', 'Trackbars')
        s_min = cv2.getTrackbarPos('S min', 'Trackbars')
        s_max = cv2.getTrackbarPos('S max', 'Trackbars')
        v_min = cv2.getTrackbarPos('V min', 'Trackbars')
        v_max = cv2.getTrackbarPos('V max', 'Trackbars')

        lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper = np.array([h_max, s_max, v_max], dtype=np.uint8)

        # Update hsv_data live
        if range_idx < len(hsv_data.get(color_name, [])):
            hsv_data[color_name][range_idx]['lower'] = [h_min, s_min, v_min]
            hsv_data[color_name][range_idx]['upper'] = [h_max, s_max, v_max]

        # Create mask
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower, upper)

        # Apply mask on original
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Draw info text
        info = f'{slot_label}  H:[{h_min}-{h_max}] S:[{s_min}-{s_max}] V:[{v_min}-{v_max}]'
        cv2.putText(frame, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Show windows
        cv2.imshow('Original', frame)
        cv2.imshow('HSV Mask', result)

        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):  # Q or ESC
            break
        elif key == ord('s'):
            save_hsv_to_yaml(yaml_path, hsv_data)
        elif key == ord('r'):
            hsv_data = load_hsv_from_yaml(yaml_path)
            prev_slot = -1  # force reload trackbars
            print('[RESET] HSV values dimuat ulang dari YAML')

    cap.release()
    cv2.destroyAllWindows()


def main() -> None:
    """Entry point color calibration tool."""
    default_yaml = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'robot_mms', 'config', 'camera_params.yaml')

    parser = argparse.ArgumentParser(
        description='HSV Color Calibration Tool untuk LKS')
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera device ID (default: 0)')
    parser.add_argument('--yaml', type=str, default=default_yaml,
                        help='Path ke camera_params.yaml')

    args = parser.parse_args()

    yaml_path = os.path.abspath(args.yaml)
    if not os.path.isfile(yaml_path):
        print(f'[ERROR] File tidak ditemukan: {yaml_path}')
        sys.exit(1)

    print(f'Camera : {args.camera}')
    print(f'YAML   : {yaml_path}')

    run_calibration(args.camera, yaml_path)


if __name__ == '__main__':
    main()
