"""YOLO camera detection for cup/bottle targets."""
from __future__ import annotations

from typing import Any

TARGET_CLASSES = {"cup", "bottle"}


def detect_target(frame: Any, model: Any) -> dict[str, Any] | None:
    """Return the highest-confidence cup/bottle detection in one frame."""
    results = model(frame, verbose=False)
    if not results:
        return None

    result = results[0]
    names = result.names
    best: dict[str, Any] | None = None

    for box in result.boxes:
        class_id = int(box.cls[0])
        target = names[class_id]
        if target not in TARGET_CLASSES:
            continue
        confidence = float(box.conf[0])
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        det = {"target": target, "confidence": confidence, "center": [cx, cy], "bbox": [x1, y1, x2, y2]}
        if best is None or confidence > best["confidence"]:
            best = det
    return best


def draw_detection(frame: Any, detection: dict[str, Any], cv2: Any) -> None:
    x1, y1, x2, y2 = detection["bbox"]
    cx, cy = detection["center"]
    label = f"{detection['target']} {detection['confidence']:.2f}"
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    cv2.putText(frame, label, (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
