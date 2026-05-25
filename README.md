# 3축 매니퓰레이터 — YOLO + IK + Arduino

카메라로 cup/bottle을 감지하고, 마우스 클릭으로 대상을 선택하면 로봇팔이 따라가는 통합 제어 시스템.

## System Architecture

```
Camera → YOLO (pixel cx,cy) → pixel_to_robot (x_mm,y_mm) → IK (a1,a2,a3) → Serial (MOVE a1 a2 a3) → Arduino → PCA9685 → DS3240 Servos
```

## Modules

| Module | File | Description |
|--------|------|-------------|
| A | `servo_control_pca9685.ino` | Arduino firmware — MOVE/PING/HOME serial commands → PWM |
| B | `ik_control.py` | 3-DOF inverse kinematics |
| B | `serial_comm.py` | Serial communication (MOVE a1 a2 a3) |
| C | `yolo_detect.py` | YOLO cup/bottle detection |
| D | `pixel_to_robot_simple.py` | Pixel → mm coordinate transform |
| - | `gui_main.py` | Tkinter GUI with camera, click-to-lock, log panel |
| - | `main.py` | CLI pipeline (alternative entry point) |
| - | `CALIBRATION_VARIABLES.md` | All 33 tunable calibration variables |

## Quick Start

**Windows:** double-click `run.bat`  
**macOS:** double-click `mac/run.command` (or `chmod +x mac/run.command; ./mac/run.sh`)

Or manually:
```bash
pip install -r requirements.txt
python gui_main.py
```

## How to Use

1. Camera shows live feed with YOLO detections (green boxes)
2. Click on an object → red box locks target
3. Robot follows locked target via IK → Serial → Arduino
4. Use 🏠 Home / 📌 Set Home / Manual angle input for calibration

## Folders

| Folder | Description |
|--------|-------------|
| `mac/` | macOS-optimized version (port auto-detect, run.sh) |
| `simple_integrated/ | Version using simpler IK (from early prototype) |
| `코드1/` | Original IK/serial prototype files |
| `robot_arm_project/` | Original pixel_to_robot prototype |
