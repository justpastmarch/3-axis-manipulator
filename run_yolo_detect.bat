@echo off
cd /d "%~dp0"
python yolo_detect.py
echo.
echo YOLO detector closed. Press any key to exit.
pause >nul
