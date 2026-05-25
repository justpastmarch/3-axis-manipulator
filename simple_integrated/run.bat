@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ========================================
echo  3-axis Manipulator — Simple Integrated
echo ========================================
echo.
python -c "import cv2, PIL, ultralytics, serial" 2>nul
if %errorlevel% neq 0 (
    echo [INFO] Installing dependencies...
    pip install -r requirements.txt
    if %errorlevel% neq 0 (
        echo [ERROR] pip install failed.
        pause
        exit /b 1
    )
    echo.
)

echo [INFO] Starting gui_main.py ...
echo [INFO] Close the GUI window to quit.
echo.
python gui_main.py

echo.
echo Robot arm GUI closed.
pause
