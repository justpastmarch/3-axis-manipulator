# 캘리브레이션 변수 목록

실제 매니퓰레이터에 맞게 **수정이 필요한 모든 변수**를 파일별로 정리.

---

## 1. `ik_control.py` — 로봇 형상 & 서보 방향

| 변수 | 현재값 | 단위 | 의미 |
|---|---|---|---|
| `L1` | `120.0` | mm | Upper arm 길이 (shoulder 회전축 ~ elbow 회전축) |
| `L2` | `100.0` | mm | Forearm 길이 (elbow 회전축 ~ end effector) |
| `H` | `80.0` | mm | Shoulder 높이 (base 기준면 ~ shoulder 회전축) |
| `BASE_OFFSET` | `90.0` | deg | Base 서보 중립각. θ₁=0→servo1=90 |
| `SHOULDER_OFFSET` | `90.0` | deg | Shoulder 서보 중립각. θ₂=0(상향)→servo2=90 |
| `ELBOW_OFFSET` | `180.0` | deg | Elbow 서보 중립각. θ₃=0(완전신전)→servo3=180 |
| `BASE_DIR` | `1` | - | Base 회전 방향. +1: θ₁증가=좌회전 |
| `SHOULDER_DIR` | `1` | - | Shoulder 회전 방향. +1: θ₂증가=전방기울기 |
| `ELBOW_DIR` | `-1` | - | Elbow 회전 방향. -1: θ₃증가=굽힘→servo3감소 |

### 튜닝 순서
1. `L1`, `L2`, `H` — 줄자로 실제 측정
2. `*_OFFSET` — HOME(90,90,90)에서 팔이直立/정면 보는지 확인 후 조정
3. `*_DIR` — 단일 servo씩 수동으로 움직여서 방향이 수식과 일치하는지 확인

---

## 2. `pixel_to_robot_simple.py` — 카메라-작업판 캘리브레이션

| 변수 | 현재값 | 단위 | 의미 |
|---|---|---|---|
| `CAMERA_WIDTH` | `640` | px | 카메라 출력 가로 해상도 |
| `CAMERA_HEIGHT` | `480` | px | 카메라 출력 세로 해상도 |
| `TABLE_WIDTH_MM` | `300.0` | mm | 카메라가 비추는 작업판 물리적 가로 |
| `TABLE_HEIGHT_MM` | `200.0` | mm | 카메라가 비추는 작업판 물리적 세로 |
| `BASE_X_MM` | `150.0` | mm | 작업판 좌상단 기준 로봇 베이스 X (가로) |
| `BASE_Y_MM` | `100.0` | mm | 작업판 좌상단 기준 로봇 베이스 Y (세로) |

### 튜닝 순서
1. 카메라 해상도를 실제 사용값으로 설정
2. 작업판 물리 크기 측정 (줄자)
3. 작업판에서 로봇 베이스 위치 측정
4. 추후 Homography로 대체 가능

---

## 3. `main.py` — 운영 파라미터

| 변수 | 라인 | 현재값 | 의미 |
|---|---|---|---|
| `TARGET_Z_MM` | 24 | `50.0` | 물체 위 팔 끝 높이 (mm, base 기준 상향+) |
| `MIN_CONFIDENCE` | 27 | `0.6` | YOLO 신뢰도 하한 (0~1) |
| `MIN_ANGLE_CHANGE` | 30 | `2` | 명령 중복 방지 deadband (degree) |
| `MIN_COMMAND_INTERVAL` | 33 | `0.1` | 명령 최소 간격 (초) |
| `YOLO_MODEL_PATH` | 36 | `"yolov8n.pt"` | YOLO 모델 파일 경로 |
| `SERIAL_PORT` | 39 | `"COM3"` | Arduino 연결 포트 |
| `NO_DETECTION_IDLE_TIMEOUT` | 148 | `5.0` | 미탐지 지속 시 HOME 복귀 시간 (초) |

### 튜닝 순서
1. `SERIAL_PORT` — 장치 관리자에서 Arduino 포트 확인
2. `TARGET_Z_MM` — 물체 높이 + 여유 높이로 설정
3. `MIN_CONFIDENCE` — 실제 YOLO 결과 보고 조정
4. 나머지는 고정해도 무방

---

## 4. `serial_comm.py` — 시리얼 통신 설정

| 변수 | 현재값 | 의미 |
|---|---|---|
| `DEFAULT_PORT` | `"COM3"` | 기본 포트 (main.py SERIAL_PORT 우선) |
| `DEFAULT_BAUDRATE` | `9600` | 보레이트 (Arduino와 일치 필수) |
| `DEFAULT_TIMEOUT` | `1.0` | 읽기 타임아웃 (초) |

---

## 5. `yolo_detect.py` — 탐지 설정

| 변수 | 현재값 | 의미 |
|---|---|---|
| `TARGET_CLASSES` | `{"cup", "bottle"}` | 탐지할 COCO 클래스명 집합 |
| `DEFAULT_CAMERA_SCAN_LIMIT` | `5` | 카메라 인덱스 스캔 범위 (0~N-1) |

---

## 6. `servo_control_pca9685.ino` — Arduino 하드웨어 설정

| 변수 | 라인 | 현재값 | 의미 |
|---|---|---|---|
| `SERVO_FREQ` | 6 | `50` | PWM 주파수 (Hz). 서보는 50 고정 |
| `BASE_CH` | 8 | `0` | Base 서보 PCA9685 채널 번호 |
| `SHOULDER_CH` | 9 | `1` | Shoulder 서보 PCA9685 채널 번호 |
| `ELBOW_CH` | 10 | `2` | Elbow 서보 PCA9685 채널 번호 |
| `SERVO_MIN_US` | 12 | `500` | 0° 대응 펄스폭 (μs) |
| `SERVO_MAX_US` | 13 | `2500` | 180° 대응 펄스폭 (μs) |

### 튜닝 순서
1. `*_CH` — 실제 배선과 일치하는지 확인
2. `SERVO_MIN_US` / `SERVO_MAX_US` — DS3240 데이터시트 확인 또는 직접 스윕 테스트
   - 최소값: 서보가 더 이상 안 움직이는 지점 + 약간의 여유
   - 최대값: 동일하게 설정

---

## 요약

| # | 파일 | 변수 개수 |
|---|---|---|
| 1 | `ik_control.py` | 9 |
| 2 | `pixel_to_robot_simple.py` | 6 |
| 3 | `main.py` | 7 |
| 4 | `serial_comm.py` | 3 |
| 5 | `yolo_detect.py` | 2 |
| 6 | `servo_control_pca9685.ino` | 6 |
| | **합계** | **33** |

### 권장 튜닝 순서

```
1. 배선 확인          → servo_control_pca9685.ino (*_CH)
2. 서보 PWM 범위 설정 → servo_control_pca9685.ino (SERVO_MIN_US, SERVO_MAX_US)
3. 링크 길이 측정     → ik_control.py (L1, L2, H)
4. 서보 오프셋/방향   → ik_control.py (*_OFFSET, *_DIR)
5. 포트 설정          → main.py (SERIAL_PORT)
6. 카메라 캘리브레이션 → pixel_to_robot_simple.py (전체)
7. Z 높이 설정        → main.py (TARGET_Z_MM)
8. 신뢰도/탐지 조정   → main.py, yolo_detect.py
```
