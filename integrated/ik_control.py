"""3축 매니퓰레이터 역기구학 (Inverse Kinematics).
로봇 형상: Base(yaw) → Shoulder(pitch) → Elbow(pitch)
좌표계: x=정면, y=왼쪽, z=위쪽, 원점=base 회전축 중심
"""
from __future__ import annotations

import math

# --- 링크 길이 (mm) --- 실제 하드웨어 측정 후 변경
L1 = 120.0   # Upper arm
L2 = 100.0   # Forearm
H = 80.0     # Shoulder 높이 (base 기준)

# --- 서보 오프셋 & 방향 ---
BASE_OFFSET = 90.0
SHOULDER_OFFSET = 90.0
ELBOW_OFFSET = 180.0    # 0°=fully extended → servo=180°
BASE_DIR = 1
SHOULDER_DIR = 1
ELBOW_DIR = -1


def inverse_kinematics(x: float, y: float, z: float) -> tuple[float, float, float]:
    """(x, y, z) mm → (servo1, servo2, servo3) degree."""
    theta1 = math.atan2(y, x)

    r = math.sqrt(x * x + y * y)
    z_rel = z - H
    d = math.sqrt(r * r + z_rel * z_rel)

    if d > L1 + L2 + 1e-6:
        raise ValueError(f"Target too far: d={d:.1f} > L1+L2={L1 + L2:.1f}")
    if d < abs(L1 - L2) - 1e-6:
        raise ValueError(f"Target too close: d={d:.1f} < |L1-L2|={abs(L1 - L2):.1f}")

    cos_t3 = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_t3 = max(-1.0, min(1.0, cos_t3))
    theta3 = math.acos(cos_t3)

    theta2 = math.atan2(r, z_rel) - math.atan2(L2 * math.sin(theta3), L1 + L2 * math.cos(theta3))

    servo1 = BASE_OFFSET + BASE_DIR * math.degrees(theta1)
    servo2 = SHOULDER_OFFSET + SHOULDER_DIR * math.degrees(theta2)
    servo3 = ELBOW_OFFSET + ELBOW_DIR * math.degrees(theta3)

    if not all(0.0 <= a <= 180.0 for a in (servo1, servo2, servo3)):
        raise ValueError(f"Servo angle out of bounds: ({servo1:.1f}, {servo2:.1f}, {servo3:.1f})")

    return (servo1, servo2, servo3)
