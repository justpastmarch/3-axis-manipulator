from manipulator_ik import compute_angles
from serial_sender import connect_serial, send_angles

def main():
    ser = connect_serial()
    if not ser:
        print("[오류] 아두이노 연결 실패, 종료합니다.")
        return

    try:
        while True:
            raw = input("좌표 입력 (x y z) 또는 'q' 종료: ").strip()
            if raw.lower() == 'q':
                break
            try:
                x, y, z = map(float, raw.split())
            except ValueError:
                print("[입력 오류] 숫자 3개를 입력하세요. 예) 100 140 120")
                continue

            result = compute_angles(x, y, z)
            if result:
                print(f"M1={result['m1']}°  M2={result['m2']}°  M3={result['m3']}°  거리={result['distance']}mm")
                send_angles(result['m1'], result['m2'], result['m3'], ser)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
