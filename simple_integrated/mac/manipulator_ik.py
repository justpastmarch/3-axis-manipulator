import math

L1 = 150.0
L2 = 120.0
H  = 100.0

M1_OFFSET = 0.0
M2_OFFSET = 0.0
M3_OFFSET = 0.0

M1_DIR = 1
M2_DIR = 1
M3_DIR = 1

SERVO_MIN =   0
SERVO_MAX = 180


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def compute_angles(x: float, y: float, z: float):
    z_rel = z - H

    m1 = M1_DIR * math.degrees(math.atan2(y, x)) + M1_OFFSET

    r = math.sqrt(x**2 + y**2)
    D = math.sqrt(x**2 + y**2 + z_rel**2)

    m2 = M2_DIR * math.degrees(math.atan2(z_rel, r)) + M2_OFFSET

    if D > (L1 + L2):
        print(f"[IK 오류] 도달 불가: 거리 {D:.1f}mm > 최대 {L1+L2:.1f}mm")
        return None
    if D < abs(L1 - L2):
        print(f"[IK 오류] 너무 가까움: 거리 {D:.1f}mm < 최소 {abs(L1-L2):.1f}mm")
        return None

    cos_angle = clamp((L1**2 + L2**2 - D**2) / (2 * L1 * L2), -1.0, 1.0)
    m3 = M3_DIR * clamp(180.0 - math.degrees(math.acos(cos_angle)), SERVO_MIN, SERVO_MAX) + M3_OFFSET

    return {
        'm1': round(m1, 2),
        'm2': round(m2, 2),
        'm3': round(m3, 2),
        'distance': round(D, 2),
        'z_rel': round(z_rel, 2)
    }
