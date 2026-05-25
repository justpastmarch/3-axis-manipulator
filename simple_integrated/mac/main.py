"""simple_integrated 메인 — YOLO + pixel_to_robot + manipulator_ik + serial_sender."""

from __future__ import annotations

import importlib
import sys
import time
from typing import Any

# ============================================================
# 0. 설정
# ============================================================

TARGET_Z_MM = 50.0          # 물체 위 팔 끝 높이 (mm)
MIN_CONFIDENCE = 0.6        # YOLO 최소 신뢰도
MIN_ANGLE_CHANGE = 2        # 명령 중복 방지 (degree)
MIN_COMMAND_INTERVAL = 0.1  # 명령 최소 간격 (초)
from serial_sender import SERIAL_PORT
TARGET_CLASSES = {"cup", "bottle"}
NO_DETECTION_IDLE_TIMEOUT = 5.0  # 초 — 미감지 시 HOME 복귀
HOME_ANGLES = (90, 90, 90)

# ============================================================
# 1. 의존성 검사
# ============================================================

_REQUIRED_PACKAGES = {
    "cv2": ("opencv-python", "pip install opencv-python"),
    "ultralytics": ("ultralytics", "pip install ultralytics"),
    "serial": ("pyserial", "pip install pyserial"),
}


def _check_dependencies() -> None:
    missing = []
    for import_name, (pkg_name, install_cmd) in _REQUIRED_PACKAGES.items():
        try:
            importlib.import_module(import_name)
        except ImportError:
            missing.append((pkg_name, install_cmd))
    if missing:
        print("=" * 60)
        print("  Missing Python packages — 설치가 필요합니다")
        print("=" * 60)
        for pkg_name, install_cmd in missing:
            print(f"  ❌ {pkg_name} → {install_cmd}")
        print("\n  → pip install -r requirements.txt\n")
        sys.exit(1)


# ============================================================
# 2. 메인 루프
# ============================================================

def main() -> None:
    _check_dependencies()

    import cv2

    # --- 카메라 ---
    from yolo_detect import open_camera, load_model, detect_target

    camera = open_camera(cv2)
    if camera is None:
        print("No camera found.")
        sys.exit(1)

    model = load_model(cv2)

    # --- Serial ---
    from serial_sender import connect_serial, send_angles

    ser = connect_serial()
    if ser:
        print(f"[INFO] Serial connected: {SERIAL_PORT}")
    else:
        print("[WARN] Serial not connected (simulation mode)")

    # --- 변환 ---
    from pixel_to_robot_simple import pixel_to_robot

    # --- IK ---
    from manipulator_ik import compute_angles

    last_command: tuple[int, int, int] | None = None
    last_command_time: float = 0.0
    no_detection_start: float | None = None

    print()
    print("=" * 50)
    print("  System ready! Press 'q' in camera window to quit.")
    print(f"  Targets: {', '.join(sorted(TARGET_CLASSES))}")
    print("=" * 50)
    print()

    while True:
        ok, frame = camera.read()
        if not ok:
            time.sleep(0.5)
            continue

        # ---- (1) YOLO 탐지 ----
        detection = detect_target(frame, model)

        if detection is not None and detection["confidence"] >= MIN_CONFIDENCE:
            no_detection_start = None
            cx, cy = detection["center"]

            # ---- (2) 픽셀 → mm ----
            try:
                x_mm, y_mm = pixel_to_robot(cx, cy)
            except ValueError as e:
                print(f"⚠ Coord conversion: {e}")
                cv2.imshow("Robot Arm Control", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            # ---- (3) 역기구학 ----
            result = compute_angles(x_mm, y_mm, TARGET_Z_MM)
            if result is None:
                print(f"⚠ IK unreachable: ({x_mm:.0f}, {y_mm:.0f}, {TARGET_Z_MM})")
                cv2.imshow("Robot Arm Control", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            a1, a2, a3 = result["m1"], result["m2"], result["m3"]
            cmd = (int(round(a1)), int(round(a2)), int(round(a3)))

            now = time.time()
            should_send = True
            if last_command is not None:
                max_diff = max(abs(cmd[i] - last_command[i]) for i in range(3))
                if max_diff < MIN_ANGLE_CHANGE:
                    should_send = False
            if now - last_command_time < MIN_COMMAND_INTERVAL:
                should_send = False

            if should_send:
                cmd_str = f"MOVE {cmd[0]} {cmd[1]} {cmd[2]}"
                if ser:
                    send_angles(cmd[0], cmd[1], cmd[2], ser)
                    print(f"{cmd_str}  ({detection['target']} @ ({cx},{cy})→({x_mm:.0f},{y_mm:.0f}))")
                else:
                    print(f"[SIMULATE] {cmd_str}")
                last_command = cmd
                last_command_time = now

        else:
            # 미감지 → idle 타이머
            if no_detection_start is None:
                no_detection_start = time.time()
            elif (
                no_detection_start is not None
                and time.time() - no_detection_start > NO_DETECTION_IDLE_TIMEOUT
            ):
                if ser:
                    send_angles(*HOME_ANGLES, ser)
                    print(f"🏠 HOME ({HOME_ANGLES[0]}, {HOME_ANGLES[1]}, {HOME_ANGLES[2]})")
                no_detection_start = time.time()  # 리셋 (반복 전송 방지)

        cv2.imshow("Robot Arm Control", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()


if __name__ == "__main__":
    main()
