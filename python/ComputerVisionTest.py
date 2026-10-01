from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parent.parent

VIDEO_SOURCE = "http://192.168.1.225:81/stream"
MODEL_NAME = PROJECT_ROOT / "models" / "yolo11n.pt"

WINDOW_NAME = "Computer Vision Test"
SAVE_DIR = PROJECT_ROOT / "poster_snapshots" / "computer_vision"
SAVE_DIR.mkdir(parents=True, exist_ok=True)


model = YOLO(MODEL_NAME)
class_names = model.names


def timestamp_string() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def draw_label(
    image,
    text: str,
    x: int,
    y: int,
    color=(0, 255, 0),
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 2

    text_size, baseline = cv2.getTextSize(
        text,
        font,
        scale,
        thickness,
    )

    text_width, text_height = text_size
    box_y1 = max(0, y - text_height - baseline - 8)
    box_y2 = max(text_height + baseline + 8, y)

    cv2.rectangle(
        image,
        (x, box_y1),
        (x + text_width + 10, box_y2),
        (0, 0, 0),
        -1,
    )

    cv2.putText(
        image,
        text,
        (x + 5, box_y2 - baseline - 4),
        font,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


cap = cv2.VideoCapture(VIDEO_SOURCE)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    raise RuntimeError("Could not open ESP32-CAM video source")


try:
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        results = model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            conf=0.35,
            verbose=False,
        )

        annotated = frame.copy()
        boxes = results[0].boxes

        if boxes is not None:
            for box in boxes:
                if box.id is None:
                    continue

                track_id = int(box.id.item())
                class_id = int(box.cls.item())
                confidence = float(box.conf.item())

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0],
                )

                cv2.rectangle(
                    annotated,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )

                label = (
                    f"{class_names[class_id]}  "
                    f"ID:{track_id}  "
                    f"{confidence:.2f}"
                )

                draw_label(
                    annotated,
                    label,
                    x1,
                    max(y1, 25),
                )

        cv2.imshow(
            WINDOW_NAME,
            annotated,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            save_path = (
                SAVE_DIR
                / f"computer_vision_{timestamp_string()}.png"
            )

            cv2.imwrite(
                str(save_path),
                annotated,
            )

            print(f"Saved snapshot: {save_path}")

        elif key == 27 or key == ord("q"):
            break


finally:
    cap.release()
    cv2.destroyAllWindows()