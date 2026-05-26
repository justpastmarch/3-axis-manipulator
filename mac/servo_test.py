"""서보 모터 연결 확인용 간단 테스트.
각 서보를 순서대로 90° → 135° → 90°로 움직여 봄.
"""
import sys
import time

try:
    from serial_comm import SerialController, DEFAULT_PORT
except ImportError:
    sys.path.insert(0, ".")
    from serial_comm import SerialController, DEFAULT_PORT

print("=" * 45)
print("  서보 모터 연결 테스트")
print("=" * 45)
print(f"  Port: {DEFAULT_PORT}")
print()

# 연결
try:
    sc = SerialController(port=DEFAULT_PORT)
    sc.connect()
    print("  ✓ Serial connected")
except Exception as e:
    print(f"  ✗ Serial connect failed: {e}")
    print()
    input("Press Enter to exit...")
    sys.exit(1)

time.sleep(0.5)

# 각 서보를 90→135→90 으로 테스트
for name, ch in [("Base    (PWM 0)", 0), ("Shoulder(PWM 1)", 1), ("Elbow   (PWM 2)", 2)]:
    print(f"\n  --- {name} ---")

    for angle in [90, 135, 90]:
        try:
            resp = sc.send_angles(*[
                angle if c == ch else 90 for c in range(3)
            ])
            print(f"    MOVE → {angle}° → {resp}")
        except Exception as e:
            print(f"    ✗ Error: {e}")
        time.sleep(0.8)

print()
print("  ✓ 테스트 완료")
print()
input("Press Enter to exit...")
