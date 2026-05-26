"""Arduino Serial 통신 — MOVE 명령 전송 (Win/macOS 자동감지)."""
from __future__ import annotations

import glob
import os
import sys
import time

# --- 포트 설정 ---
# None=자동감지, 또는 직접 지정 예: "COM3", "/dev/cu.usbserial-2120"
SERIAL_PORT: str | None = None
SERIAL_BAUDRATE = 9600
SERIAL_TIMEOUT = 1.0


def _detect_port() -> str:
    """플랫폼별 Arduino 포트 자동 감지."""
    env_port = os.environ.get("SERIAL_PORT")
    if env_port:
        return env_port

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
        return "COM3"  # fallback

    # Windows: 환경변수 SERIAL_PORT 없으면 COM3 기본
    return "COM3"


def _import_serial():
    try:
        import serial as _serial
        return _serial
    except ImportError:
        raise SystemExit("Missing pyserial. Install with: pip install pyserial")


class SerialController:
    """Arduino Serial 연결 및 MOVE 명령 전송."""

    def __init__(self, port: str | None = None, baudrate: int = SERIAL_BAUDRATE, timeout: float = SERIAL_TIMEOUT):
        self.port = port or _detect_port()
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None

    def connect(self) -> None:
        serial = _import_serial()
        self._ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        time.sleep(2.0)
        self._ser.write(b"PING\n")
        time.sleep(0.5)
        while self._ser.in_waiting:
            self._ser.readline()

    def send_angles(self, a1: int, a2: int, a3: int) -> str:
        if self._ser is None or not self._ser.is_open:
            raise ConnectionError("Serial port is not open. Call connect() first.")
        if not all(0 <= a <= 180 for a in (a1, a2, a3)):
            raise ValueError(f"Servo angles out of 0~180: ({a1}, {a2}, {a3})")
        cmd = f"MOVE {a1} {a2} {a3}\n"
        self._ser.write(cmd.encode("ascii"))
        response = self._ser.readline().decode("ascii", errors="replace").strip()
        return response

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            self._ser = None
