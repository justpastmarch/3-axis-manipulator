"""YOLO camera detection for cup/bottle targets.

This module owns project part C from the plan:
camera frame -> target bbox -> center pixel coordinate.
"""

from __future__ import annotations

import argparse
import importlib
import json
from typing import Any


TARGET_CLASSES = {"cup", "bottle"}
DEFAULT_CAMERA_SCAN_LIMIT = 5


def _load_dependencies() -> tuple[Any, Any]:
    try:
        cv2 = importlib.import_module("cv2")
        ultralytics = importlib.import_module("ultralytics")
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency. Install with: "
            "pip install ultralytics opencv-python numpy"
        ) from exc

    return cv2, ultralytics.YOLO


def _best_target(result: Any, target_classes: set[str]) -> dict[str, Any] | None:
    names = result.names
    best_detection: dict[str, Any] | None = None

    for box in result.boxes:
        class_id = int(box.cls[0])
        target = names[class_id]

        if target not in target_classes:
            continue

        confidence = float(box.conf[0])
        x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        detection = {
            "target": target,
            "confidence": confidence,
            "center": [center_x, center_y],
            "bbox": [x1, y1, x2, y2],
        }

        if best_detection is None or confidence > best_detection["confidence"]:
            best_detection = detection

    return best_detection


def detect_target(frame: Any, model: Any, target_classes: set[str] | None = None) -> dict[str, Any] | None:
    """Return the highest-confidence cup/bottle detection in one frame.

    The return value matches the project plan:
    {
        "target": "cup",
        "confidence": 0.82,
        "center": [340, 220],
        "bbox": [300, 180, 380, 260],
    }
    """

    classes = target_classes or TARGET_CLASSES
    results = model(frame, verbose=False)

    if not results:
        return None

    return _best_target(results[0], classes)


def draw_detection(frame: Any, detection: dict[str, Any], cv2: Any) -> None:
    x1, y1, x2, y2 = detection["bbox"]
    center_x, center_y = detection["center"]
    label = f"{detection['target']} {detection['confidence']:.2f}"

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
    cv2.putText(
        frame,
        label,
        (x1, max(y1 - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )


def open_camera(cv2: Any, camera_index: int | None, scan_limit: int) -> tuple[Any, int]:
    if scan_limit < 1:
        raise SystemExit("--scan-limit must be at least 1")

    if camera_index is not None:
        camera = cv2.VideoCapture(camera_index)

        if camera.isOpened():
            return camera, camera_index

        camera.release()
        raise SystemExit(f"Could not open camera index {camera_index}")

    for index in range(scan_limit):
        camera = cv2.VideoCapture(index)

        if camera.isOpened():
            ok, _frame = camera.read()

            if ok:
                return camera, index

        camera.release()

    raise SystemExit(f"Could not find a working camera in indices 0-{scan_limit - 1}")


def run_camera(camera_index: int | None, model_path: str, scan_limit: int) -> None:
    cv2, YOLO = _load_dependencies()
    model = YOLO(model_path)
    camera, opened_index = open_camera(cv2, camera_index, scan_limit)
    print(f"Using camera index {opened_index}. Press q in the video window to quit.")

    try:
        while True:
            ok, frame = camera.read()

            if not ok:
                raise SystemExit("Could not read frame from camera")

            detection = detect_target(frame, model)

            if detection is not None:
                print(json.dumps(detection, ensure_ascii=False), flush=True)
                draw_detection(frame, detection, cv2)

            cv2.imshow("YOLO cup/bottle detection", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect cup/bottle objects and print bbox center pixels.")
    parser.add_argument(
        "--camera",
        type=int,
        default=None,
        help="OpenCV camera index. Default: auto-detect indices 0-4",
    )
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model path/name. Default: yolov8n.pt")
    parser.add_argument(
        "--scan-limit",
        type=int,
        default=DEFAULT_CAMERA_SCAN_LIMIT,
        help="Number of camera indices to scan when --camera is omitted. Default: 5",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_camera(args.camera, args.model, args.scan_limit)


if __name__ == "__main__":
    main()
