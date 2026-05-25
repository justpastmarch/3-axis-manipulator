"""GUI 통합 메인 (simple_integrated 버전 — 코드1/robot_arm_project 모듈 사용)."""

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
# 0. 설정
# ============================================================

TARGET_Z_MM = 50.0          # 물체 위 팔 끝 높이 (mm)
MIN_CONFIDENCE = 0.6        # YOLO 최소 신뢰도
MIN_ANGLE_CHANGE = 2        # 명령 중복 방지 (degree)
MIN_COMMAND_INTERVAL = 0.1  # 명령 최소 간격 (초)
from serial_sender import SERIAL_PORT

LOCK_TRACK_DIST_THRESHOLD = 80   # locked 대상 추적 최대 픽셀 거리
LOCK_AUTO_UNLOCK_SEC = 3.0       # locked 대상 연속 미감지 시 자동 해제 시간 (초)
LOG_MAX_LINES = 500              # 로그 최대 줄 수

PROC_W, PROC_H = 640, 480        # YOLO/픽셀매핑 내부 처리 해상도

CAMERA_INDEX: int | None = None  # 특정 카메라 인덱스 (None=자동 검색, 0=내장캠, 1=USB웹캠)

HOME_ANGLES_DEFAULT = (90, 90, 90)
HOME_CONFIG_FILE = "home_position.json"


# ============================================================
# 1. 의존성 검사
# ============================================================

_REQUIRED_PACKAGES = {
    "cv2": ("opencv-python", "pip install opencv-python"),
    "PIL": ("pillow", "pip install pillow"),
    "ultralytics": ("ultralytics", "pip install ultralytics"),
    "serial": ("pyserial", "pip install pyserial"),
}


def _check_dependencies() -> None:
    missing = []
    for import_name, (pkg_name, install_cmd) in _REQUIRED_PACKAGES.items():
        try:
            importlib.import_module(import_name)
        except ImportError:
            missing.append((pkg_name, install_cmd))
    if missing:
        print("=" * 60)
        print("  Missing Python packages — 설치가 필요합니다")
        print("=" * 60)
        for pkg_name, install_cmd in missing:
            print(f"  ❌ {pkg_name} → {install_cmd}")
        print("\n  → pip install -r requirements.txt\n")
        sys.exit(1)


# ============================================================
# 2. GUI 클래스
# ============================================================

class RobotGUI:
    """Tkinter 기반 로봇팔 제어 GUI (simple_integrated)."""

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Robot Arm Control (simple)")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # --- 상태 변수 ---
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

        # --- 하드웨어 초기화 ---
        self._init_hardware()

        # --- UI 구성 ---
        self._build_ui()
        self.log("System initialized (simple_integrated). Click on an object to lock.", "INFO")

    # ── 하드웨어 초기화 ──────────────────────────────────

    def _init_hardware(self) -> None:
        try:
            import cv2 as _cv2
            self.cv2 = _cv2
        except ImportError:
            self.cv2 = None

        # 카메라
        self.camera = None
        indices_to_try: list[int] = []
        if CAMERA_INDEX is not None:
            indices_to_try = [CAMERA_INDEX] + [i for i in range(5) if i != CAMERA_INDEX]
        else:
            indices_to_try = list(range(5))
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
            print(f"[CAMERA] No camera found (tried indices: {indices_to_try})")

        # YOLO 모델
        self.model = None
        try:
            ultralytics = importlib.import_module("ultralytics")
            YOLO = ultralytics.YOLO
            self.model = YOLO("yolov8n.pt")
        except Exception:
            pass

        # 시리얼 (simple_integrated: connect_serial 함수 사용)
        self.serial = None
        try:
            from serial_sender import connect_serial
            self.serial = connect_serial()
        except Exception:
            pass

    # ── Home 자세 저장/불러오기 ─────────────────────────

    @staticmethod
    def _load_home() -> tuple[int, int, int]:
        if os.path.exists(HOME_CONFIG_FILE):
            try:
                with open(HOME_CONFIG_FILE, "r") as f:
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
        try:
            from serial_sender import send_angles
            if self.serial is not None:
                send_angles(cmd[0], cmd[1], cmd[2], self.serial)
                self.log(f"🏠 HOME → {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}", "MOVE")
            else:
                self.log(f"[SIMULATE] 🏠 HOME {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}", "MOVE")
        except Exception as e:
            self.log(f"🏠 HOME failed: {e}", "ERROR")
            return
        self.last_command = cmd
        self.last_command_time = time.time()

    def _set_home(self) -> None:
        if self.last_command is not None:
            self.home_angles = self.last_command
            self._save_home()
            self.label_home_angles.config(text=f"Home: {self.home_angles[0]}° {self.home_angles[1]}° {self.home_angles[2]}°")
            self.log(f"📌 Home saved: {self.home_angles[0]}° {self.home_angles[1]}° {self.home_angles[2]}°", "LOCK")
        else:
            self.log("⚠ No command sent yet — move robot first", "WARN")

    def _send_manual(self) -> None:
        try:
            a1 = int(self.entry_a1.get())
            a2 = int(self.entry_a2.get())
            a3 = int(self.entry_a3.get())
        except ValueError:
            self.log("⚠ Invalid angle — enter integers only", "ERROR")
            return
        cmd = (a1, a2, a3)
        try:
            from serial_sender import send_angles
            if self.serial is not None:
                send_angles(cmd[0], cmd[1], cmd[2], self.serial)
                self.log(f"▶ MANUAL MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}", "MOVE")
            else:
                self.log(f"[SIMULATE] ▶ MANUAL MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}", "MOVE")
        except Exception as e:
            self.log(f"▶ MANUAL send failed: {e}", "ERROR")
            return
        self.last_command = cmd
        self.last_command_time = time.time()

    # ── 레터박스 ────────────────────────────────────────

    def _letterbox(self, frame: Any, target_w: int, target_h: int) -> Any:
        h, w = frame.shape[:2]
        if w == target_w and h == target_h:
            return frame.copy()
        scale = min(target_w / w, target_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        resized = self.cv2.resize(frame, (new_w, new_h))
        top = (target_h - new_h) // 2
        bottom = target_h - new_h - top
        left = (target_w - new_w) // 2
        right = target_w - new_w - left
        return self.cv2.copyMakeBorder(resized, top, bottom, left, right, self.cv2.BORDER_CONSTANT, value=(0, 0, 0))

    # ── UI 구성 ──────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root.minsize(800, 500)

        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ── 우측: 로그 패널 ──
        right_frame = tk.Frame(main_frame, width=300)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=False)
        right_frame.pack_propagate(False)

        tk.Label(right_frame, text="📋 Event Log", font=("Arial", 11, "bold")).pack(fill=tk.X, padx=5, pady=(5, 0))

        self.text_log = ScrolledText(
            right_frame, width=38, height=30,
            font=("Consolas", 9), state=tk.DISABLED, wrap=tk.WORD,
        )
        self.text_log.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.text_log.tag_config("INFO", foreground="black")
        self.text_log.tag_config("MOVE", foreground="#1a56db")
        self.text_log.tag_config("LOCK", foreground="#0d7c3f")
        self.text_log.tag_config("UNLOCK", foreground="#6b7280")
        self.text_log.tag_config("WARN", foreground="#b45309")
        self.text_log.tag_config("ERROR", foreground="#dc2626")

        # ── 좌측: 카메라 + 버튼 ──
        left_frame = tk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        left_frame.pack_propagate(False)

        self.canvas = tk.Canvas(left_frame, bg="black", cursor="crosshair", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=(5, 0))
        self.canvas.bind("<Button-1>", self._on_click)

        self.label_hint = tk.Label(left_frame, text="🖱 Click on an object to lock", fg="gray")
        self.label_hint.pack(pady=(2, 0))

        btn_frame = tk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, padx=5, pady=(2, 5))

        self.btn_home = tk.Button(btn_frame, text="🏠 Home", command=self._go_home, font=("Arial", 10, "bold"), bg="#e5e7eb", relief=tk.RAISED, padx=10)
        self.btn_home.pack(side=tk.LEFT, padx=2)

        self.btn_set_home = tk.Button(btn_frame, text="📌 Set Home", command=self._set_home, font=("Arial", 10), bg="#e5e7eb", relief=tk.RAISED, padx=8)
        self.btn_set_home.pack(side=tk.LEFT, padx=2)

        self.label_home_angles = tk.Label(btn_frame, text=f"Home: {self.home_angles[0]}° {self.home_angles[1]}° {self.home_angles[2]}°", font=("Consolas", 10), fg="#4b5563")
        self.label_home_angles.pack(side=tk.RIGHT, padx=5)

        # ── 수동 각도 입력 ──
        manual_frame = tk.Frame(left_frame)
        manual_frame.pack(fill=tk.X, padx=5, pady=(0, 5))

        tk.Label(manual_frame, text="a1:", font=("Consolas", 10)).pack(side=tk.LEFT)
        self.entry_a1 = tk.Entry(manual_frame, width=5, font=("Consolas", 10))
        self.entry_a1.pack(side=tk.LEFT, padx=(0, 8))
        self.entry_a1.insert(0, "90")

        tk.Label(manual_frame, text="a2:", font=("Consolas", 10)).pack(side=tk.LEFT)
        self.entry_a2 = tk.Entry(manual_frame, width=5, font=("Consolas", 10))
        self.entry_a2.pack(side=tk.LEFT, padx=(0, 8))
        self.entry_a2.insert(0, "90")

        tk.Label(manual_frame, text="a3:", font=("Consolas", 10)).pack(side=tk.LEFT)
        self.entry_a3 = tk.Entry(manual_frame, width=5, font=("Consolas", 10))
        self.entry_a3.pack(side=tk.LEFT, padx=(0, 8))
        self.entry_a3.insert(0, "90")

        self.btn_send = tk.Button(manual_frame, text="▶ Send", command=self._send_manual, font=("Arial", 10, "bold"), bg="#dbeafe", relief=tk.RAISED, padx=10)
        self.btn_send.pack(side=tk.LEFT, padx=2)

        # ── 하단: 상태바 ──
        self.status_bar = tk.Label(self.root, text="Initializing...", bd=1, relief=tk.SUNKEN, anchor=tk.W, font=("Consolas", 10))
        self.status_bar.pack(fill=tk.X, side=tk.BOTTOM)

    # ── 마우스 클릭 ──────────────────────────────────────

    def _on_click(self, event: tk.Event) -> None:
        x, y = event.x, event.y

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()
        if canvas_w < 10 or canvas_h < 10:
            canvas_w, canvas_h = PROC_W, PROC_H
        scale = min(canvas_w / PROC_W, canvas_h / PROC_H)
        disp_w = int(PROC_W * scale)
        disp_h = int(PROC_H * scale)
        x_off = (canvas_w - disp_w) // 2
        y_off = (canvas_h - disp_h) // 2
        proc_x = (x - x_off) / scale
        proc_y = (y - y_off) / scale

        for det in self.current_detections:
            x1, y1, x2, y2 = det["bbox"]
            margin = 10
            if (x1 - margin) <= proc_x <= (x2 + margin) and (y1 - margin) <= proc_y <= (y2 + margin):
                self.locked = det.copy()
                self.locked_last_seen = time.time()
                self.log(f"🖱 Locked onto: {det['target']} (conf={det['confidence']:.2f}) @ ({det['center'][0]}, {det['center'][1]})", "LOCK")
                self.label_hint.config(text=f"🔒 Locked: {det['target']}", fg="#0d7c3f")
                return

        if self.locked is not None:
            self.locked = None
            self.log("🖱 Unlocked (clicked empty space)", "UNLOCK")
            self.label_hint.config(text="🖱 Click on an object to lock", fg="gray")

    # ── Locked 대상 추적 ─────────────────────────────────

    def _track_locked(self, detections: list[dict[str, Any]]) -> dict[str, Any] | None:
        if self.locked is None:
            return None
        locked_cx, locked_cy = self.locked["center"]
        best: dict[str, Any] | None = None
        best_dist = LOCK_TRACK_DIST_THRESHOLD
        for det in detections:
            if det["target"] != self.locked["target"]:
                continue
            cx, cy = det["center"]
            dist = ((cx - locked_cx) ** 2 + (cy - locked_cy) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best = det
        if best is not None:
            self.locked = best.copy()
            self.locked_last_seen = time.time()
            return best
        if time.time() - self.locked_last_seen > LOCK_AUTO_UNLOCK_SEC:
            prev = self.locked["target"]
            self.locked = None
            self.log(f"🔓 Auto-unlocked: {prev} lost for {LOCK_AUTO_UNLOCK_SEC}s", "UNLOCK")
            self.label_hint.config(text="🖱 Click on an object to lock", fg="gray")
        return None

    # ── 메인 루프 ────────────────────────────────────────

    def update_frame(self) -> None:
        if not self.running:
            return

        start_time = time.time()

        # (1) 카메라 캡처
        ok, frame = self.camera.read() if self.camera else (False, None)
        if not ok or frame is None:
            self.root.after(50, self.update_frame)
            return

        # 내부 처리 해상도로 통일
        frame = self._letterbox(frame, PROC_W, PROC_H)
        self.latest_frame = frame.copy()

        # (2) YOLO 추론
        detections: list[dict[str, Any]] = []
        if self.model is not None:
            raw_results = self.model(frame, verbose=False)
            if raw_results:
                result = raw_results[0]
                names = result.names
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    target = names[class_id]
                    confidence = float(box.conf[0])
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    detections.append({
                        "target": target, "confidence": confidence,
                        "center": [cx, cy], "bbox": [x1, y1, x2, y2],
                    })
        self.current_detections = detections

        # (3) 대상 선정
        selected: dict[str, Any] | None
        if self.locked is not None:
            selected = self._track_locked(detections)
        else:
            filtered = [d for d in detections if d["confidence"] >= MIN_CONFIDENCE]
            selected = max(filtered, key=lambda d: d["confidence"]) if filtered else None

        # (4) 파이프라인 실행 (simple_integrated 모듈 사용)
        if selected is not None and selected["confidence"] >= MIN_CONFIDENCE:
            cx, cy = selected["center"]

            try:
                from pixel_to_robot_simple import pixel_to_robot
                x_mm, y_mm = pixel_to_robot(cx, cy)
            except Exception:
                self._draw_frame(frame, detections, selected)
                self._schedule_next(start_time)
                return

            try:
                from manipulator_ik import compute_angles
                result = compute_angles(x_mm, y_mm, TARGET_Z_MM)
                if result is None:
                    self._draw_frame(frame, detections, selected)
                    self._schedule_next(start_time)
                    return
                a1, a2, a3 = result["m1"], result["m2"], result["m3"]
            except Exception:
                self._draw_frame(frame, detections, selected)
                self._schedule_next(start_time)
                return

            cmd = (int(round(a1)), int(round(a2)), int(round(a3)))
            now = time.time()

            should_send = True
            if self.last_command is not None:
                max_diff = max(abs(cmd[i] - self.last_command[i]) for i in range(3))
                if max_diff < MIN_ANGLE_CHANGE:
                    should_send = False
            if now - self.last_command_time < MIN_COMMAND_INTERVAL:
                should_send = False

            if should_send:
                from serial_sender import send_angles
                try:
                    if self.serial is not None:
                        send_angles(cmd[0], cmd[1], cmd[2], self.serial)
                        self.log(
                            f"MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}  "
                            f"({selected['target']} @ ({cx:3d},{cy:3d})→({x_mm:.0f},{y_mm:.0f}))",
                            "MOVE",
                        )
                    else:
                        self.log(
                            f"[SIMULATE] MOVE {cmd[0]:3d} {cmd[1]:3d} {cmd[2]:3d}  "
                            f"({selected['target']} @ ({cx:3d},{cy:3d}))",
                            "MOVE",
                        )
                except Exception:
                    pass
                self.last_command = cmd
                self.last_command_time = now

        # (5) 화면 그리기
        self._draw_frame(frame, detections, selected)

        # (6) FPS
        self.fps_counter += 1
        if time.time() - self.fps_timer >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.fps_timer = time.time()

        # (7) 상태바
        status_parts = []
        if selected:
            lock_mark = "🔒" if self.locked else "●"
            status_parts.append(f"{lock_mark} {selected['target']} @ ({selected['center'][0]},{selected['center'][1]})")
        else:
            status_parts.append("● No target")
        status_parts.append(f"Detected: {len(detections)}")
        status_parts.append(f"FPS: {self.fps}")
        self.status_bar.config(text=" | ".join(status_parts))

        self._schedule_next(start_time)

    def _draw_frame(self, frame: Any, detections: list[dict[str, Any]], selected: dict[str, Any] | None) -> None:
        from PIL import Image, ImageTk

        if self.cv2 is None:
            return

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()
        if canvas_w < 10 or canvas_h < 10:
            canvas_w, canvas_h = PROC_W, PROC_H

        scale = min(canvas_w / PROC_W, canvas_h / PROC_H)
        disp_w = int(PROC_W * scale)
        disp_h = int(PROC_H * scale)
        resized = self.cv2.resize(frame, (disp_w, disp_h))

        display = self.cv2.copyMakeBorder(
            resized,
            (canvas_h - disp_h) // 2, (canvas_h - disp_h + 1) // 2,
            (canvas_w - disp_w) // 2, (canvas_w - disp_w + 1) // 2,
            self.cv2.BORDER_CONSTANT, value=(0, 0, 0),
        )
        x_off = (canvas_w - disp_w) // 2
        y_off = (canvas_h - disp_h) // 2

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            cx, cy = det["center"]
            is_selected = (selected is not None and det["center"] == selected["center"] and det["target"] == selected["target"])
            color = (0, 0, 255) if is_selected else (0, 255, 0)
            thickness = max(1, int(3 * scale)) if is_selected else max(1, int(1 * scale))

            sx1, sy1 = int(x1 * scale + x_off), int(y1 * scale + y_off)
            sx2, sy2 = int(x2 * scale + x_off), int(y2 * scale + y_off)
            scx, scy = int(cx * scale + x_off), int(cy * scale + y_off)

            self.cv2.rectangle(display, (sx1, sy1), (sx2, sy2), color, thickness)
            self.cv2.circle(display, (scx, scy), max(3, int(5 * scale)), color, -1)

            label = f"{det['target']} {det['confidence']:.2f}"
            if is_selected and self.locked:
                label = f"🔒 {label}"
            font_scale = max(0.3, 0.5 * scale)
            self.cv2.putText(display, label, (sx1, max(sy1 - 10, 20)), self.cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, max(1, int(2 * scale)))

        if selected is not None:
            cx, cy = selected["center"]
            scx, scy = int(cx * scale + x_off), int(cy * scale + y_off)
            self.cv2.drawMarker(display, (scx, scy), (0, 0, 255), self.cv2.MARKER_CROSS, max(8, int(20 * scale)), max(1, int(2 * scale)))

        rgb = self.cv2.cvtColor(display, self.cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb)
        self._tk_image = ImageTk.PhotoImage(pil_img)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self._tk_image)

    def _schedule_next(self, start_time: float) -> None:
        elapsed_ms = (time.time() - start_time) * 1000
        interval = max(10, int(elapsed_ms))
        self.root.after(interval, self.update_frame)

    # ── 로그 ─────────────────────────────────────────────

    def log(self, message: str, tag: str = "INFO") -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.text_log.configure(state=tk.NORMAL)
        self.text_log.insert(tk.END, f"[{timestamp}] ", "INFO")
        self.text_log.insert(tk.END, f"{message}\n", tag)
        line_count = int(self.text_log.index("end-1c").split(".")[0])
        if line_count > LOG_MAX_LINES:
            self.text_log.delete("1.0", f"{line_count - LOG_MAX_LINES + 1}.0")
        self.text_log.see(tk.END)
        self.text_log.configure(state=tk.DISABLED)

    # ── 종료 ─────────────────────────────────────────────

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
            self.log("YOLO model loaded: yolov8n.pt ✓", "INFO")
        if self.serial:
            self.log(f"Serial connected: {SERIAL_PORT} ✓", "INFO")
        else:
            self.log("Serial not connected (simulation mode)", "WARN")
        self.log(f"Target Z: {TARGET_Z_MM}mm, Min confidence: {MIN_CONFIDENCE}", "INFO")
        self.log("-" * 40, "INFO")
        self.root.after(100, self.update_frame)
        self.root.mainloop()


# ============================================================
# 3. 메인
# ============================================================

def main() -> None:
    _check_dependencies()
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
        print("\n위 오류를 개발자에게 알려주세요.\n")
        input("Enter 키를 누르면 종료됩니다...")
