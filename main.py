"""통합 메인 — YOLO 감지 → 좌표변환 → IK → Serial → Arduino.

파이프라인 (1 cycle):
  카메라 프레임
  → yolo_detect.detect_target(frame) → center (cx, cy)
  → pixel_to_robot_simple.pixel_to_robot(cx, cy) → (x_mm, y_mm)
  → ik_control.inverse_kinematics(x, y, z) → (servo1, servo2, servo3)
  → serial_comm.send_angles(a1, a2, a3) → Arduino → PCA9685 → 서보
"""

from __future__ import annotations

import importlib
import sys
import time
from typing import Any

# ============================================================
# 0. 설정
# ============================================================

# Z 높이: 물체 위 안전 높이 (mm)
# 로봇 베이스 기준, 위쪽(+). 테이블 위 물체라면 보통 0~50mm.
TARGET_Z_MM = 50.0

# 최소 신뢰도 (0~1)
MIN_CONFIDENCE = 0.6

# 동일 명령 반복 전송 방지 최소 변화량 (degree)
MIN_ANGLE_CHANGE = 2

# 명령 최소 간격 (초)
MIN_COMMAND_INTERVAL = 0.1

# YOLO 모델 경로
YOLO_MODEL_PATH = "yolov8n.pt"

# 시리얼 포트 설정
SERIAL_PORT = "COM3"

# ============================================================
# 1. 의존성 검사
# ============================================================

_REQUIRED_PACKAGES = {
    "cv2": ("opencv-python", "pip install opencv-python"),
    "ultralytics": ("ultralytics", "pip install ultralytics"),
    "serial": ("pyserial", "pip install pyserial"),
}


def _check_dependencies() -> None:
    """필요한 패키지가 모두 설치되었는지 확인하고, 없으면 안내 후 종료."""
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
        print()
        for pkg_name, install_cmd in missing:
            print(f"  ❌ {pkg_name}")
            print(f"     → {install_cmd}")
        print()
        print("  또는 다음 명령어로 한 번에 설치:")
        print()
        print("      pip install -r requirements.txt")
        print()
        print("=" * 60)
        sys.exit(1)


# ============================================================
# 2. 모듈 임포트 (의존성 통과 후)
# ============================================================

def _import_module(import_name: str):
    return importlib.import_module(import_name)


def _load_yolo() -> tuple[Any, Any]:
    cv2 = _import_module("cv2")
    ultralytics = _import_module("ultralytics")
    YOLO = ultralytics.YOLO
    return cv2, YOLO


# ============================================================
# 3. 카메라 자동 탐색
# ============================================================

def _open_camera(cv2: Any, scan_limit: int = 5) -> tuple[Any, int]:
    for index in range(scan_limit):
        cam = cv2.VideoCapture(index)
        if cam.isOpened():
            ok, _ = cam.read()
            if ok:
                return cam, index
        cam.release()
    raise SystemExit(
        f"Could not find a working camera (scanned 0-{scan_limit - 1})"
    )


# ============================================================
# 4. 메인 루프
# ============================================================

def main() -> None:
    # ---- 의존성 검사 (가장 먼저 실행) ----
    _check_dependencies()

    # ---- 모듈 로드 ----
    print("Loading modules...")
    cv2, YOLO = _load_yolo()
    from yolo_detect import detect_target, draw_detection, TARGET_CLASSES
    from pixel_to_robot_simple import pixel_to_robot
    from ik_control import inverse_kinematics
    from serial_comm import SerialController

    # ---- 초기화 ----
    print("Loading YOLO model...")
    model = YOLO(YOLO_MODEL_PATH)

    print("Opening camera...")
    camera, cam_index = _open_camera(cv2)
    print(f"  → Camera index {cam_index} opened")

    print("Connecting to Arduino...")
    serial = SerialController(port=SERIAL_PORT)
    try:
        serial.connect()
        print(f"  → Connected on {SERIAL_PORT}")
    except Exception as e:
        print(f"  ⚠ Could not connect to Arduino on {SERIAL_PORT}: {e}")
        print("  Run without serial (display only).")
        serial = None

    last_command: tuple[int, int, int] | None = None
    last_command_time = 0.0
    no_detection_start: float | None = None
    NO_DETECTION_IDLE_TIMEOUT = 5.0  # 초

    print()
    print("=" * 50)
    print("  System ready! Press 'q' in camera window to quit.")
    print(f"  Targets: {', '.join(sorted(TARGET_CLASSES))}")
    print("=" * 50)
    print()

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("⚠ Camera read failed, retrying...")
                time.sleep(0.5)
                continue

            # ---- (1) YOLO 탐지 ----
            detection = detect_target(frame, model)

            if detection is not None and detection["confidence"] >= MIN_CONFIDENCE:
                no_detection_start = None
                cx, cy = detection["center"]

                # ---- (2) 픽셀 → mm 변환 ----
                try:
                    x_mm, y_mm = pixel_to_robot(cx, cy)
                except ValueError as e:
                    print(f"⚠ Coordinate conversion failed: {e}")
                    cv2.imshow("Robot Arm Control", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    continue

                # ---- (3) 역기구학 ----
                try:
                    a1, a2, a3 = inverse_kinematics(x_mm, y_mm, TARGET_Z_MM)
                except ValueError as e:
                    print(f"⚠ IK failed: {e}")
                    cv2.imshow("Robot Arm Control", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    continue

                # ---- (4) 반올림 & 중복 방지 & 간격 유지 ----
                cmd = (int(round(a1)), int(round(a2)), int(round(a3)))
                now = time.time()

                should_send = True
                if last_command is not None:
                    max_diff = max(abs(cmd[i] - last_command[i]) for i in range(3))
                    if max_diff < MIN_ANGLE_CHANGE:
                        should_send = False
                if now - last_command_time < MIN_COMMAND_INTERVAL:
                    should_send = False

                if should_send and serial is not None:
                    try:
                        response = serial.send_angles(*cmd)
                        print(
                            f"MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d} "
                            f"→ {response}  "
                            f"(target: {detection['target']} "
                            f"@ ({cx:3d}, {cy:3d}) → "
                            f"({x_mm:6.1f}, {y_mm:6.1f}, {TARGET_Z_MM:.1f})"
                        )
                        last_command = cmd
                        last_command_time = now
                    except Exception as e:
                        print(f"⚠ Serial send failed: {e}")
                elif should_send and serial is None:
                    print(
                        f"[SIMULATE] MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}  "
                        f"(target: {detection['target']} "
                        f"@ ({cx:3d}, {cy:3d})"
                    )

                # ---- 화면 표시 ----
                draw_detection(frame, detection, cv2)

            else:
                # 탐지 없음
                if no_detection_start is None:
                    no_detection_start = time.time()
                elif (
                    no_detection_start is not None
                    and time.time() - no_detection_start > NO_DETECTION_IDLE_TIMEOUT
                    and serial is not None
                    and last_command != (90, 90, 90)
                ):
                    print("No detection for a while → moving to HOME (90, 90, 90)")
                    try:
                        serial.send_angles(90, 90, 90)
                        last_command = (90, 90, 90)
                        last_command_time = time.time()
                    except Exception:
                        pass

            cv2.imshow("Robot Arm Control", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("\nShutting down...")
        camera.release()
        cv2.destroyAllWindows()
        if serial is not None:
            serial.close()
        print("Done.")


if __name__ == "__main__":
    main()
