"""픽셀 좌표 → 로봇 좌표(mm) 변환 (비례식 + FLIP_Y 지원)."""
from __future__ import annotations

# --- 카메라 해상도 ---
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# --- 작업판 실제 크기 (mm) ---
TABLE_WIDTH_MM = 300.0
TABLE_HEIGHT_MM = 200.0

# --- 로봇 베이스 위치 (mm, 작업판 좌상단 기준) ---
BASE_X_MM = 150.0
BASE_Y_MM = 100.0

# --- Y축 반전 ---
# True: 카메라 위쪽 = 로봍 y+ (일반적인 테이블 설치)
# False: 카메라 위쪽 = 로봇 y- (카메라가 로봇 위에 있을 때)
FLIP_Y = False


def pixel_to_robot(cx: int, cy: int) -> tuple[float, float]:
    """카메라 픽셀 (cx, cy) → 로봇 베이스 기준 mm."""
    if not (0 <= cx <= CAMERA_WIDTH):
        raise ValueError(f"cx ({cx}) out of range 0~{CAMERA_WIDTH}")
    if not (0 <= cy <= CAMERA_HEIGHT):
        raise ValueError(f"cy ({cy}) out of range 0~{CAMERA_HEIGHT}")

    x_table = cx / CAMERA_WIDTH * TABLE_WIDTH_MM

    if FLIP_Y:
        y_table = TABLE_HEIGHT_MM - (cy / CAMERA_HEIGHT * TABLE_HEIGHT_MM)
    else:
        y_table = cy / CAMERA_HEIGHT * TABLE_HEIGHT_MM

    x_robot = x_table - BASE_X_MM
    y_robot = y_table - BASE_Y_MM

    return (x_robot, y_robot)
