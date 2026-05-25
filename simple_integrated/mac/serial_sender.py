"""Arduino Serial 통신 — MOVE 명령 전송 (macOS 포트 자동감지)."""

import glob
import sys
import time


def _detect_port() -> str:
    """macOS에서 Arduino 포트 자동 감지, 실패/Windows면 기본값 반환."""
    if sys.platform == "darwin":
        patterns = [
            "/dev/cu.usbmodem*",
            "/dev/cu.usbserial*",
            "/dev/tty.usbmodem*",
            "/dev/tty.usbserial*",
            "/dev/tty.wchusbserial*",
        ]
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                return ports[0]
    return "COM3"  # Windows fallback / macOS 감지 실패


# 연결할 포트 (macOS는 자동 감지)
SERIAL_PORT = _detect_port()
BAUD_RATE = 9600


def connect_serial():
    try:
        import serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        return ser
    except Exception as e:
        print(f"[SERIAL] 연결 실패 ({SERIAL_PORT}): {e}")
        return None


def send_angles(a1: float, a2: float, a3: float, ser):
    cmd = f"MOVE {a1} {a2} {a3}\n"
    ser.write(cmd.encode())
