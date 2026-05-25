"""픽셀 좌표 → 로봇 좌표(mm) 변환 (단순 비율 방식)."""

# 카메라 해상도 (pixel)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# 작업판 실제 크기 (mm)
TABLE_WIDTH_MM = 300.0
TABLE_HEIGHT_MM = 200.0

# 작업판에서 로봇 베이스 위치 (mm)
# 작업판 좌상단이 (0,0), 우측이 x+, 아래가 y+
BASE_X_MM = 150.0  # 작업판 가로 중앙
BASE_Y_MM = 100.0  # 작업판 세로 중앙


def pixel_to_robot(cx: int, cy: int) -> tuple[float, float]:
    """카메라 픽셀 좌표 (cx, cy) → 로봇 베이스 기준 mm 좌표 (x_mm, y_mm).

    Args:
        cx: 이미지 상 x 픽셀 (0 ~ CAMERA_WIDTH)
        cy: 이미지 상 y 픽셀 (0 ~ CAMERA_HEIGHT)

    Returns:
        (x_robot, y_robot): 로봇 베이스 기준 mm 좌표.
            x: 로봇 정면 방향 (+)
            y: 로봇 왼쪽 방향 (+)

    Raises:
        ValueError: 픽셀 좌표가 카메라 해상도 범위를 벗어난 경우
    """
    if not (0 <= cx <= CAMERA_WIDTH):
        raise ValueError(f"cx ({cx}) is outside camera width (0~{CAMERA_WIDTH})")
    if not (0 <= cy <= CAMERA_HEIGHT):
        raise ValueError(f"cy ({cy}) is outside camera height (0~{CAMERA_HEIGHT})")

    # 픽셀 → 작업판 좌표 (비례 변환)
    x_table = cx / CAMERA_WIDTH * TABLE_WIDTH_MM
    y_table = cy / CAMERA_HEIGHT * TABLE_HEIGHT_MM

    # 작업판 좌표 → 로봇 베이스 기준 좌표
    x_robot = x_table - BASE_X_MM
    y_robot = y_table - BASE_Y_MM

    return (x_robot, y_robot)
