"""Arduino Serial 통신 — MOVE 명령 전송 (macOS 포트 자동감지)."""

from __future__ import annotations

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
DEFAULT_PORT = _detect_port()
DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT = 1.0


def _import_serial():
    """필요할 때 serial 모듈을 import (의존성 지연 로딩)."""
    try:
        import serial as _serial

        return _serial
    except ImportError:
        raise SystemExit(
            "Missing pyserial. Install with: pip install pyserial"
        )


class SerialController:
    """Arduino Serial 연결 및 MOVE 명령 전송."""

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None

    def connect(self) -> None:
        """시리얼 포트 열기 (Arduino 리셋 대기 포함)."""
        serial = _import_serial()
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )
        # Arduino 리셋 후 안정화 대기
        time.sleep(2.0)

        # 웨이크업 신호 전송
        self._ser.write(b"PING\n")
        time.sleep(0.5)
        # READY 신호까지 읽어 버리기
        while self._ser.in_waiting:
            self._ser.readline()

    def send_angles(self, a1: int, a2: int, a3: int) -> str:
        """서보 각도 3개를 MOVE 명령으로 Arduino에 전송.

        Args:
            a1: base servo 각도 (0~180, 정수)
            a2: shoulder servo 각도 (0~180, 정수)
            a3: elbow servo 각도 (0~180, 정수)

        Returns:
            Arduino 응답 문자열 ("OK" / "ERROR")

        Raises:
            ConnectionError: 시리얼 연결이 끊긴 경우
            ValueError: 각도 범위를 벗어난 경우
        """
        if self._ser is None or not self._ser.is_open:
            raise ConnectionError("Serial port is not open. Call connect() first.")

        if not all(0 <= a <= 180 for a in (a1, a2, a3)):
            raise ValueError(f"Servo angles out of 0~180 range: ({a1}, {a2}, {a3})")

        cmd = f"MOVE {a1} {a2} {a3}\n"
        self._ser.write(cmd.encode("ascii"))

        # 응답 대기
        response = self._ser.readline().decode("ascii", errors="replace").strip()
        return response

    def close(self) -> None:
        """시리얼 포트 정리."""
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            self._ser = None
