"""GUI 통합 메인 — 카메라 + YOLO + IK + 시리얼 (통합 버전).

레이아웃:
  ┌─────────────────────────────┬──────────────────────┐
  │      Camera Feed            │   Event Log          │
  │      (click to select)      │                      │
  ├─────────────────────────────┴──────────────────────┤
  │  Status + Home/SetHome + Manual angle input        │
  └────────────────────────────────────────────────────┘
"""
from __future__ import annotations

import importlib
import json
import os
import sys
import time
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
from typing import Any

# ============================================================
# 설정 (사용자 수정 가능)
# ============================================================

# --- 제어 ---
TARGET_Z_MM = 50.0          # 물체 위 팔 끝 높이 (mm)
MIN_CONFIDENCE = 0.6        # YOLO 최소 신뢰도
MIN_ANGLE_CHANGE = 2        # 명령 중복 방지 (degree)
MIN_COMMAND_INTERVAL = 0.1  # 명령 최소 간격 (초)
YOLO_MODEL_PATH = "yolov8n.pt"
LOCK_TRACK_DIST_THRESHOLD = 80   # locked 대상 추적 최대 픽셀 거리
LOCK_AUTO_UNLOCK_SEC = 3.0       # locked 대상 연속 미감지 시 자동 해제 시간 (초)
LOG_MAX_LINES = 500              # 로그 최대 줄 수

# --- 카메라 ---
CAMERA_INDEX: int | None = None  # None=자동, 0=내장캠, 1=USB웹캠
PROC_W, PROC_H = 640, 480        # YOLO/픽셀매핑 내부 처리 해상도

# --- Home ---
HOME_ANGLES_DEFAULT = (90, 90, 90)
HOME_CONFIG_FILE = "home_position.json"

# --- Serial (여기서 포트 지정 가능, None=자동감지) ---
SERIAL_PORT: str | None = None

# ============================================================
# GUI
# ============================================================

class RobotGUI:
    """Tkinter 기반 로봇팔 제어 GUI."""

    def __init__(self, skip_hardware: bool = False) -> None:
        self.root = tk.Tk()
        self.root.title("Robot Arm Control")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # 상태 변수
        self.running = True
        self.locked: dict[str, Any] | None = None
        self.locked_last_seen: float = 0.0
        self.current_detections: list[dict[str, Any]] = []
        self.latest_frame: Any = None
        self.last_command: tuple[int, int, int] | None = None
        self.last_command_time: float = 0.0
        self.home_angles: tuple[int, int, int] = self._load_home()
        self._cam_index = -1
        self.fps_counter = 0
        self.fps_timer = time.time()
        self.fps = 0

        # cv2
        try:
            import cv2 as _cv2
            self.cv2 = _cv2
        except ImportError:
            self.cv2 = None

        self.camera = None
        self.model = None
        self.serial = None

        if not skip_hardware:
            self._init_hardware()

        self._build_ui()
        self.log("System initialized. Click on an object to lock.", "INFO")

    # ── 하드웨어 초기화 ──────────────────────────────────

    def _init_hardware(self) -> None:
        """카메라, YOLO, 시리얼 초기화."""
        # 카메라
        indices_to_try: list[int] = []
        if CAMERA_INDEX is not None:
            indices_to_try = [CAMERA_INDEX] + [i for i in range(5) if i != CAMERA_INDEX]
        else:
            indices_to_try = list(range(5))
        self.camera = None
        for idx in indices_to_try:
            cam = self.cv2.VideoCapture(idx) if self.cv2 else None
            if cam and cam.isOpened():
                cam.set(self.cv2.CAP_PROP_FRAME_WIDTH, 640)
                cam.set(self.cv2.CAP_PROP_FRAME_HEIGHT, 480)
                ok, _ = cam.read()
                if ok:
                    self.camera = cam
                    self._cam_index = idx
                    break
            if cam:
                cam.release()
        if self.camera:
            print(f"[CAMERA] Opened index {self._cam_index} ✓")
        else:
            print("[CAMERA] No camera found")

        # YOLO
        self.model = None
        try:
            ultralytics = importlib.import_module("ultralytics")
            YOLO = ultralytics.YOLO
            self.model = YOLO(YOLO_MODEL_PATH)
        except Exception:
            pass

        # 시리얼 (통합 serial_comm 사용)
        self.serial = None
        try:
            from serial_comm import SerialController
            sc = SerialController(port=SERIAL_PORT)  # SERIAL_PORT=None → 자동감지
            sc.connect()
            self.serial = sc
        except Exception:
            self.serial = None

    # ── Home ─────────────────────────────────────────────

    @staticmethod
    def _load_home() -> tuple[int, int, int]:
        if os.path.exists(HOME_CONFIG_FILE):
            try:
                with open(HOME_CONFIG_FILE) as f:
                    data = json.load(f)
                return (int(data["a1"]), int(data["a2"]), int(data["a3"]))
            except Exception:
                pass
        return HOME_ANGLES_DEFAULT

    def _save_home(self) -> None:
        with open(HOME_CONFIG_FILE, "w") as f:
            json.dump({"a1": self.home_angles[0], "a2": self.home_angles[1], "a3": self.home_angles[2]}, f)

    def _go_home(self) -> None:
        cmd = self.home_angles
        if self.serial is not None:
            try:
                resp = self.serial.send_angles(*cmd)
                tag = "MOVE" if resp == "OK" else "WARN"
                self.log(f"🏠 HOME → {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d} → {resp}", tag)
            except Exception as e:
                self.log(f"🏠 HOME failed: {e}", "ERROR")
                return
        else:
            self.log(f"[SIMULATE] 🏠 HOME {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}", "MOVE")
        self.last_command = cmd
        self.last_command_time = time.time()

    def _set_home(self) -> None:
        if self.last_command is not None:
            self.home_angles = self.last_command
            self._save_home()
            self.label_home.config(text=f"Home: {self.home_angles[0]}° {self.home_angles[1]}° {self.home_angles[2]}°")
            self.log(f"📌 Home set to {self.home_angles}", "INFO")
        else:
            self.log("📌 No command to set as Home. Send a command first.", "WARN")

    # ── 수동 각도 입력 ──────────────────────────────────

    def _send_manual(self) -> None:
        try:
            a1 = int(self.entry_a1.get())
            a2 = int(self.entry_a2.get())
            a3 = int(self.entry_a3.get())
        except ValueError:
            self.log("Manual: invalid angle (integer only)", "ERROR")
            return
        if not all(0 <= a <= 180 for a in (a1, a2, a3)):
            self.log("Manual: angles must be 0~180", "ERROR")
            return
        cmd = (a1, a2, a3)
        if self.serial is not None:
            try:
                resp = self.serial.send_angles(*cmd)
                tag = "MOVE" if resp == "OK" else "WARN"
                self.log(f"▶ MOVE {a1:3d} {a2:3d} {a3:3d} → {resp}", tag)
            except Exception as e:
                self.log(f"▶ Send failed: {e}", "ERROR")
                return
        else:
            self.log(f"[SIMULATE] ▶ MOVE {a1:3d} {a2:3d} {a3:3d}", "MOVE")
        self.last_command = cmd
        self.last_command_time = time.time()

    # ── UI 구성 ─────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root.minsize(800, 500)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main_frame = tk.Frame(self.root)
        main_frame.grid(row=0, column=0, sticky="nsew")
        main_frame.columnconfigure(0, weight=3)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # 왼쪽: 카메라 캔버스
        self.canvas = tk.Canvas(main_frame, bg="black", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=(5, 2), pady=5)
        self.canvas.bind("<Button-1>", self._on_click)

        # 오른쪽: 로그 패널
        right_frame = tk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=(2, 5), pady=5)
        right_frame.rowconfigure(0, weight=1)
        right_frame.columnconfigure(0, weight=1)

        self.text_log = ScrolledText(right_frame, state=tk.DISABLED, wrap=tk.WORD, font=("Consolas", 9), height=1)
        self.text_log.grid(row=0, column=0, sticky="nsew")

        # 하단: 상태바 + 버튼 + 수동입력
        bottom_frame = tk.Frame(self.root)
        bottom_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=(0, 5))
        bottom_frame.columnconfigure(0, weight=1)

        # 1줄: 상태바
        self.status_bar = tk.Label(bottom_frame, text="● Initializing...", anchor="w", font=("Consolas", 10))
        self.status_bar.grid(row=0, column=0, columnspan=10, sticky="ew", pady=(0, 3))

        # 2줄: Home 관련
        btn_home = tk.Button(bottom_frame, text="🏠 Home", command=self._go_home, width=8)
        btn_home.grid(row=1, column=0, padx=(0, 2))
        btn_sethome = tk.Button(bottom_frame, text="📌 Set Home", command=self._set_home, width=8)
        btn_sethome.grid(row=1, column=1, padx=2)
        self.label_home = tk.Label(bottom_frame, text=f"Home: {self.home_angles[0]}° {self.home_angles[1]}° {self.home_angles[2]}°", font=("Consolas", 9))
        self.label_home.grid(row=1, column=2, padx=(5, 10))

        # 3줄: 수동 각도 입력
        tk.Label(bottom_frame, text="a1:", font=("Consolas", 9)).grid(row=2, column=0, sticky="e")
        self.entry_a1 = tk.Entry(bottom_frame, width=4, font=("Consolas", 9))
        self.entry_a1.insert(0, "90")
        self.entry_a1.grid(row=2, column=1, sticky="w")
        tk.Label(bottom_frame, text="a2:", font=("Consolas", 9)).grid(row=2, column=2, sticky="e")
        self.entry_a2 = tk.Entry(bottom_frame, width=4, font=("Consolas", 9))
        self.entry_a2.insert(0, "90")
        self.entry_a2.grid(row=2, column=3, sticky="w")
        tk.Label(bottom_frame, text="a3:", font=("Consolas", 9)).grid(row=2, column=4, sticky="e")
        self.entry_a3 = tk.Entry(bottom_frame, width=4, font=("Consolas", 9))
        self.entry_a3.insert(0, "90")
        self.entry_a3.grid(row=2, column=5, sticky="w")
        btn_send = tk.Button(bottom_frame, text="▶ Send", command=self._send_manual, width=6)
        btn_send.grid(row=2, column=6, padx=(5, 0))

        # 힌트 라벨
        self.label_hint = tk.Label(bottom_frame, text="🖱 Click on an object to lock", fg="gray", font=("Consolas", 9))
        self.label_hint.grid(row=2, column=7, padx=(10, 0), sticky="w")

    # ── 로그 ────────────────────────────────────────────

    def log(self, msg: str, tag: str = "INFO") -> None:
        tags = {"INFO": "black", "WARN": "orange", "ERROR": "red", "MOVE": "blue", "UNLOCK": "gray"}
        color = tags.get(tag, "black")
        timestamp = time.strftime("%H:%M:%S")
        self.text_log.configure(state=tk.NORMAL)
        self.text_log.insert(tk.END, f"[{timestamp}] {msg}\n", color)
        self.text_log.tag_config(color, foreground=color)
        self.text_log.see(tk.END)

        lines = int(self.text_log.index("end-1c").split(".")[0])
        if lines > LOG_MAX_LINES:
            self.text_log.delete("1.0", f"{lines - LOG_MAX_LINES}.0")

        self.text_log.configure(state=tk.DISABLED)

    # ── 마우스 클릭 ─────────────────────────────────────

    def _on_click(self, event: tk.Event) -> None:
        """캔버스 클릭 → locked 대상 설정."""
        if not self.current_detections:
            self.log("No detections to lock.", "WARN")
            return

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()
        if canvas_w < 1 or canvas_h < 1:
            return

        # 캔버스 좌표 → PROC 좌표
        scale_x = PROC_W / canvas_w
        scale_y = PROC_H / canvas_h
        click_cx = int(event.x * scale_x)
        click_cy = int(event.y * scale_y)

        # 가장 가까운 detection 찾기
        best_dist = float("inf")
        best_det = None
        for det in self.current_detections:
            cx, cy = det["center"]
            d = (cx - click_cx) ** 2 + (cy - click_cy) ** 2
            if d < best_dist:
                best_dist = d
                best_det = det

        if best_det and best_dist < 10000:  # 100픽셀² 이내
            self.locked = best_det
            self.locked_last_seen = time.time()
            self.log(f"🔒 Locked: {best_det['target']} @ ({best_det['center'][0]},{best_det['center'][1]})", "INFO")
            self.label_hint.config(text=f"🔒 {best_det['target']}", fg="red")

    # ── Locked 추적 ─────────────────────────────────────

    def _track_locked(self, detections: list[dict[str, Any]]) -> dict[str, Any] | None:
        if self.locked is None:
            return None

        locked_cx, locked_cy = self.locked["center"]
        best = None
        best_dist = float("inf")

        for det in detections:
            cx, cy = det["center"]
            d = abs(cx - locked_cx) + abs(cy - locked_cy)
            if d < best_dist:
                best_dist = d
                best = det

        if best is not None and best_dist < LOCK_TRACK_DIST_THRESHOLD:
            self.locked = best
            self.locked_last_seen = time.time()
            return best

        if time.time() - self.locked_last_seen > LOCK_AUTO_UNLOCK_SEC:
            prev = self.locked["target"]
            self.locked = None
            self.log(f"🔓 Auto-unlocked: {prev} lost for {LOCK_AUTO_UNLOCK_SEC}s", "UNLOCK")
            self.label_hint.config(text="🖱 Click on an object to lock", fg="gray")

        return None

    # ── 레터박스 ────────────────────────────────────────

    def _letterbox(self, frame: Any, target_w: int, target_h: int) -> Any:
        h, w = frame.shape[:2]
        if w == target_w and h == target_h:
            return frame.copy()
        scale = min(target_w / w, target_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = self.cv2.resize(frame, (new_w, new_h))
        top = (target_h - new_h) // 2
        bottom = target_h - new_h - top
        left = (target_w - new_w) // 2
        right = target_w - new_w - left
        return self.cv2.copyMakeBorder(resized, top, bottom, left, right, self.cv2.BORDER_CONSTANT, value=(0, 0, 0))

    # ── 메인 루프 ───────────────────────────────────────

    def update_frame(self) -> None:
        if not self.running:
            return
        start = time.time()

        ok, frame = self.camera.read() if self.camera else (False, None)
        if not ok or frame is None:
            self._schedule_next(start)
            return

        frame = self._letterbox(frame, PROC_W, PROC_H)
        self.latest_frame = frame.copy()

        # YOLO 추론
        detections: list[dict[str, Any]] = []
        if self.model is not None:
            raw = self.model(frame, verbose=False)
            if raw:
                result = raw[0]
                names = result.names
                for box in result.boxes:
                    cid = int(box.cls[0])
                    target = names[cid]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                    detections.append({
                        "target": target, "confidence": conf,
                        "center": [(x1 + x2) // 2, (y1 + y2) // 2],
                        "bbox": [x1, y1, x2, y2],
                    })
        self.current_detections = detections

        # 대상 선정
        selected: dict[str, Any] | None = None
        if self.locked is not None:
            selected = self._track_locked(detections)
        else:
            filtered = [d for d in detections if d["confidence"] >= MIN_CONFIDENCE]
            selected = max(filtered, key=lambda d: d["confidence"]) if filtered else None

        # 파이프라인
        if selected is not None and selected["confidence"] >= MIN_CONFIDENCE:
            cx, cy = selected["center"]
            try:
                from pixel_to_robot_simple import pixel_to_robot
                x_mm, y_mm = pixel_to_robot(cx, cy)
            except Exception:
                self._draw_frame(frame, detections, selected)
                self._schedule_next(start)
                return

            try:
                from ik_control import inverse_kinematics
                a1, a2, a3 = inverse_kinematics(x_mm, y_mm, TARGET_Z_MM)
            except Exception:
                self._draw_frame(frame, detections, selected)
                self._schedule_next(start)
                return

            cmd = (int(round(a1)), int(round(a2)), int(round(a3)))
            now = time.time()
            should_send = True
            if self.last_command is not None:
                if max(abs(cmd[i] - self.last_command[i]) for i in range(3)) < MIN_ANGLE_CHANGE:
                    should_send = False
            if now - self.last_command_time < MIN_COMMAND_INTERVAL:
                should_send = False

            if should_send and self.serial is not None:
                try:
                    resp = self.serial.send_angles(*cmd)
                    tag = "MOVE" if resp == "OK" else "WARN"
                    self.log(f"MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d} → {resp}  ({selected['target']})", tag)
                    self.last_command = cmd
                    self.last_command_time = now
                except Exception:
                    pass
            elif should_send and self.serial is None:
                self.log(f"[SIMULATE] MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}  ({selected['target']})", "MOVE")
                self.last_command = cmd
                self.last_command_time = now

        # 화면 그리기
        self._draw_frame(frame, detections, selected)

        # FPS
        self.fps_counter += 1
        if time.time() - self.fps_timer >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.fps_timer = time.time()

        # 상태바
        parts = []
        if selected:
            mark = "🔒" if self.locked else "●"
            parts.append(f"{mark} {selected['target']} @ ({selected['center'][0]},{selected['center'][1]})")
        else:
            parts.append("● No target")
        parts.append(f"Detected: {len(detections)}")
        parts.append(f"FPS: {self.fps}")
        self.status_bar.config(text=" | ".join(parts))

        self._schedule_next(start)

    def _schedule_next(self, start: float) -> None:
        elapsed = time.time() - start
        delay = max(1, int((1 / 30 - elapsed) * 1000))
        self.root.after(delay, self.update_frame)

    def _draw_frame(self, frame: Any, detections: list[dict[str, Any]], selected: dict[str, Any] | None) -> None:
        from PIL import Image, ImageTk

        if self.cv2 is None:
            return

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()
        if canvas_w < 10 or canvas_h < 10:
            return

        # BGR → RGB
        display = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        h, w = display.shape[:2]

        # 비율 유지 리사이즈
        scale = min(canvas_w / w, canvas_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = self.cv2.resize(display, (new_w, new_h))
        scale_x = w / new_w  # display→PROC
        scale_y = h / new_h

        # 바운딩박스 그리기
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            color = (0, 255, 0) if det is not selected else (255, 0, 0)
            thickness = 2 if det is not selected else 3
            self.cv2.rectangle(resized,
                (int(x1 / scale_x), int(y1 / scale_y)),
                (int(x2 / scale_x), int(y2 / scale_y)),
                color, thickness)

        # PIL → ImageTk
        img = Image.fromarray(resized)
        imgtk = ImageTk.PhotoImage(image=img)
        self.canvas.image = imgtk
        self.canvas.create_image(0, 0, anchor="nw", image=imgtk)

    # ── 종료 ────────────────────────────────────────────

    def _on_close(self) -> None:
        self.running = False
        if self.camera:
            self.camera.release()
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.root.destroy()

    def run(self) -> None:
        if self.camera:
            self.log(f"Camera index {self._cam_index} opened ✓", "INFO")
        if self.model:
            self.log(f"YOLO model loaded: {YOLO_MODEL_PATH} ✓", "INFO")
        if self.serial:
            port = getattr(self.serial, "port", "?")
            self.log(f"Serial connected: {port} ✓", "INFO")
        else:
            self.log("Serial not connected (simulation mode)", "WARN")
        self.log(f"Target Z: {TARGET_Z_MM}mm, Min confidence: {MIN_CONFIDENCE}", "INFO")
        self.log("-" * 40, "INFO")

        self.root.after(100, self.update_frame)
        self.root.mainloop()


# ============================================================
# 메인
# ============================================================

def main() -> None:
    import importlib
    missing = []
    for import_name, pkg in [("cv2", "opencv-python"), ("PIL", "pillow"),
                              ("ultralytics", "ultralytics"), ("serial", "pyserial")]:
        try:
            importlib.import_module(import_name)
        except ImportError:
            missing.append(pkg)
    if missing:
        print("=" * 60)
        print("  Missing Python packages — 설치가 필요합니다")
        print("=" * 60)
        for pkg in missing:
            print(f"  ❌ {pkg}")
        print(f"\n  → pip install -r requirements.txt\n")
        sys.exit(1)

    app = RobotGUI()
    app.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        print("=" * 60)
        print("  ERROR — 프로그램 실행 중 오류가 발생했습니다")
        print("=" * 60)
        traceback.print_exc()
        print()
        input("Press Enter to exit...")
