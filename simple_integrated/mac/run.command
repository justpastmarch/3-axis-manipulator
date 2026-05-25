#!/bin/bash
cd "$(dirname "$0")"

echo "========================================"
echo " 3-axis Manipulator — macOS (simple)"
echo "========================================"
echo ""

# Tkinter 확인 (brew install python-tk 필요할 수 있음)
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[INFO] Tkinter not found. Installing via Homebrew..."
    brew install python-tk 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "[ERROR] Tkinter install failed."
        echo "  Try manually: brew install python-tk"
        read -p "Press Enter to exit..."
        exit 1
    fi
    echo ""
fi

# 의존성 확인
python3 -c "import cv2, PIL, ultralytics, serial" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[INFO] Installing dependencies..."
    pip3 install -r ../requirements.txt
    if [ $? -ne 0 ]; then
        echo "[ERROR] pip install failed."
        read -p "Press Enter to exit..."
        exit 1
    fi
    echo ""
fi

echo "[INFO] Starting gui_main.py ..."
echo "[INFO] Close the GUI window to quit."
echo ""
python3 gui_main.py

echo ""
echo "Robot arm GUI closed."
read -p "Press Enter to exit..."
