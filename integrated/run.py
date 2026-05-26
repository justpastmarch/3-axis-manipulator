#!/usr/bin/env python3
"""3축 매니퓰레이터 — 통합 실행기 (Windows/macOS 공용).

사용법:
  python run.py              # 일반 실행
  python run.py --serial COM5  # 시리얼 포트 직접 지정
  python run.py --no-camera   # 카메라 없이 실행 (디버깅용)

환경변수:
  SERIAL_PORT=COM5  # 시리얼 포트 직접 지정
"""
from __future__ import annotations

import argparse
import importlib
import os
import subprocess
import sys


def _check_deps(auto_install: bool = False) -> bool:
    required = [
        ("cv2", "opencv-python"),
        ("PIL", "pillow"),
        ("ultralytics", "ultralytics"),
        ("serial", "pyserial"),
    ]
    missing = []
    for import_name, pkg_name in required:
        try:
            importlib.import_module(import_name)
        except ImportError:
            missing.append(pkg_name)

    if not missing:
        return True

    print("=" * 50)
    print("  Missing Python packages")
    print("=" * 50)
    for pkg in missing:
        print(f"  ❌ {pkg}")

    if auto_install:
        print("\n  → Installing...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install"] + missing)
            print("  ✓ Done\n")
            return True
        except subprocess.CalledProcessError:
            print("  ✗ Install failed\n")
            return False
    else:
        print(f"\n  → pip install -r requirements.txt\n")
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description="3-axis Manipulator — Integrated")
    parser.add_argument("--serial", type=str, default=None, help="Serial port (e.g. COM3, /dev/cu.usbserial-2120)")
    parser.add_argument("--no-camera", action="store_true", help="Skip camera init")
    parser.add_argument("--install", action="store_true", help="Auto-install missing packages")
    args = parser.parse_args()

    # 의존성 검사
    if not _check_deps(auto_install=args.install):
        sys.exit(1)

    # 시리얼 포트 환경변수 설정 (CLI 인자 우선)
    if args.serial:
        os.environ["SERIAL_PORT"] = args.serial

    # GUI 실행
    from gui_main import RobotGUI

    app = RobotGUI(skip_hardware=args.no_camera)
    app.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        print("=" * 60)
        print("  ERROR")
        print("=" * 60)
        traceback.print_exc()
        print()
        input("Press Enter to exit...")
