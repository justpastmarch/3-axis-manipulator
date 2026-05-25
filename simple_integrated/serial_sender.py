import serial
import time

SERIAL_PORT = "COM3"
BAUD_RATE   = 9600

def connect_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        return ser
    except Exception as e:
        print(f"[SERIAL 오류] 연결 실패: {e}")
        return None

def send_angles(a1: float, a2: float, a3: float, ser: serial.Serial):
    cmd = f"MOVE {a1} {a2} {a3}\n"
    ser.write(cmd.encode())
