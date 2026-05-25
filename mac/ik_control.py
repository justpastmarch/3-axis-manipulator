"""3축 매니퓰레이터 역기구학 (Inverse Kinematics).

로봇 형상:
  - Base 회전 (yaw): servo1 (PCA9685 CH0)
  - Shoulder (pitch): servo2 (PCA9685 CH1)
  - Elbow (pitch): servo3 (PCA9685 CH2)

좌표계 (오른손 법칙):
  x: 로봇 정면 (+)
  y: 로봇 왼쪽 (+)
  z: 위쪽 (+)
  원점: base 회전축 중심

FK 검증:
  r = L1*sin(θ2) + L2*sin(θ2+θ3)
  z_rel = L1*cos(θ2) + L2*cos(θ2+θ3)
  x = r*cos(θ1), y = r*sin(θ1), z = H + z_rel
"""

from __future__ import annotations

import math

# --- 링크 길이 (mm) ---
L1 = 120.0  # Upper arm
L2 = 100.0  # Forearm
H = 80.0    # Shoulder joint 높이 (base 기준)

# --- 서보 오프셋 & 방향 ---
# servo_angle = offset + dir * math_angle
#
# Elbow (θ3 = acos(…)):
#   0°  = arm fully extended (forearm inline with upper arm)
#   90° = forearm at right angle to upper arm
#   180°= arm fully folded
# servo3 = 180 - θ3  →  θ3=0→servo3=180, θ3=90→servo3=90, θ3=180→servo3=0
# 이렇게 하면 0~180 전 범위를 θ3에 쓸 수 있어 테이블 위 물체 도달 가능.
#
# Shoulder (θ2 = 0 → arm straight up, positive = forward tilt):
#   servo2 = 90 + θ2  →  θ2=0→90, θ2=90→180 (horizontal)
#
# Base (θ1 = atan2(y,x), 0 = forward):
#   servo1 = 90 + θ1  →  range [-90, 90] → [0, 180] (전방 반구)
BASE_OFFSET = 90.0
SHOULDER_OFFSET = 90.0
ELBOW_OFFSET = 180.0

BASE_DIR = 1
SHOULDER_DIR = 1
ELBOW_DIR = -1


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    """(x, y, z) mm → 서보 각도 (servo1, servo2, servo3) degree.

    Args:
        x: 로봇 정방향 mm
        y: 로봇 좌측 mm
        z: 높이 mm (위쪽 +)

    Returns:
        (servo1, servo2, servo3): 각도 (degree), 0~180 범위 보장

    Raises:
        ValueError: 도달 불가능한 좌표일 때
    """
    # --- 1. Base 회전 ---
    theta1 = math.atan2(y, x)

    # --- 2. 수직 평면 투영 ---
    r = math.sqrt(x * x + y * y)
    z_rel = z - H  # shoulder 기준 높이

    d_sq = r * r + z_rel * z_rel
    d = math.sqrt(d_sq)

    # --- 3. 도달 가능성 검사 ---
    if d > L1 + L2 + 1e-6:
        raise ValueError(
            f"Target ({x:.1f}, {y:.1f}, {z:.1f}) is too far: "
            f"d={d:.1f} > L1+L2={L1 + L2:.1f}"
        )
    if d < abs(L1 - L2) - 1e-6:
        raise ValueError(
            f"Target ({x:.1f}, {y:.1f}, {z:.1f}) is too close: "
            f"d={d:.1f} < |L1-L2|={abs(L1 - L2):.1f}"
        )

    # --- 4. Elbow 각도 (θ3) ---
    cos_theta3 = (d_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # 수치 안정화
    theta3 = math.acos(cos_theta3)  # 0 ~ π, elbow-down solution

    # --- 5. Shoulder 각도 (θ2) ---
    theta2 = math.atan2(r, z_rel) - math.atan2(
        L2 * math.sin(theta3), L1 + L2 * math.cos(theta3)
    )

    # --- 6. Degree 변환 ---
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)

    # --- 7. 서보 오프셋/방향 적용 ---
    servo1 = BASE_OFFSET + BASE_DIR * theta1_deg
    servo2 = SHOULDER_OFFSET + SHOULDER_DIR * theta2_deg
    servo3 = ELBOW_OFFSET + ELBOW_DIR * theta3_deg

    # --- 8. 0~180 범위 검증 ---
    if not all(0.0 <= a <= 180.0 for a in (servo1, servo2, servo3)):
        raise ValueError(
            f"Servo angle out of bounds after offset: "
            f"({servo1:.1f}, {servo2:.1f}, {servo3:.1f}) "
            f"for target ({x:.1f}, {y:.1f}, {z:.1f})"
        )

    return (servo1, servo2, servo3)
