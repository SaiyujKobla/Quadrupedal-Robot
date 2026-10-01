from __future__ import annotations

import time
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from object_detection.Cluster import ClusterManager
from object_detection.ClusterMemory import ClusterMemory


PROJECT_ROOT = Path(__file__).resolve().parent.parent

VIDEO_SOURCE = "http://192.168.1.225:81/stream"
MODEL_NAME = PROJECT_ROOT / "models" / "yolo11n.pt"

WINDOW_NAME = "Object Tracking + Clustering Test"
SAVE_DIR = PROJECT_ROOT / "poster_snapshots" / "tracking_clustering"
SAVE_DIR.mkdir(parents=True, exist_ok=True)


TRACK_COLOR = (0, 255, 0)
CLUSTER_COLOR = (255, 0, 255)
MERGED_COLOR = (0, 165, 255)


model = YOLO(MODEL_NAME)
class_names = model.names

cluster_manager = ClusterManager(
    physical_merge_gap_m=0.40,
    fallback_merge_distance_px=30.0,
    max_depth_gap_m=0.75,
    max_relative_depth_gap=0.40,
)

cluster_memory = ClusterMemory(
    history_size=15,
    stale_after_s=2.5,
)

track_history: dict[int, deque] = defaultdict(
    lambda: deque(maxlen=20)
)


def timestamp_string() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def draw_text_box(
    image,
    text: str,
    x: int,
    y: int,
    color,
    scale: float = 0.50,
    thickness: int = 1,
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX

    text_size, baseline = cv2.getTextSize(
        text,
        font,
        scale,
        thickness,
    )

    text_width, text_height = text_size
    x = max(0, x)
    y = max(text_height + baseline + 8, y)

    cv2.rectangle(
        image,
        (x, y - text_height - baseline - 8),
        (x + text_width + 10, y),
        (0, 0, 0),
        -1,
    )

    cv2.putText(
        image,
        text,
        (x + 5, y - baseline - 4),
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

        frame_time = time.monotonic()

        results = model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            conf=0.35,
            verbose=False,
        )

        annotated = frame.copy()
        objects = []

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

                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                area = (
                    max(0, x2 - x1)
                    * max(0, y2 - y1)
                )

                objects.append(
                    {
                        "id": track_id,
                        "class_id": class_id,
                        "class_name": class_names[class_id],
                        "confidence": confidence,
                        "box": (x1, y1, x2, y2),
                        "center": (center_x, center_y),
                        "area": area,
                    }
                )

                track_history[track_id].append(
                    (center_x, center_y)
                )

        clusters, merged_clusters = (
            cluster_manager.process(objects)
        )

        cluster_memory.update(
            merged_clusters,
            frame_time,
        )

        for obj in objects:
            x1, y1, x2, y2 = obj["box"]

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                TRACK_COLOR,
                2,
            )

            label = (
                f"{obj['class_name']}  "
                f"ID:{obj['id']}"
            )

            draw_text_box(
                annotated,
                label,
                x1,
                max(y1, 24),
                TRACK_COLOR,
                scale=0.48,
                thickness=1,
            )

            points = list(
                track_history[obj["id"]]
            )

            if len(points) >= 2:
                cv2.polylines(
                    annotated,
                    [np.array(points, dtype=np.int32)],
                    False,
                    TRACK_COLOR,
                    2,
                    cv2.LINE_AA,
                )

        for cluster in clusters:
            x1, y1, x2, y2 = cluster["box"]

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                CLUSTER_COLOR,
                2,
            )

            cluster_label = (
                f"Cluster {cluster['temporary_id']}  "
                f"n={cluster['count']}"
            )

            draw_text_box(
                annotated,
                cluster_label,
                x1,
                min(frame.shape[0] - 2, y2 + 22),
                CLUSTER_COLOR,
                scale=0.44,
                thickness=1,
            )

        for merged_cluster in merged_clusters:
            x1, y1, x2, y2 = merged_cluster["box"]

            padding = 5

            x1 = max(0, x1 - padding)
            y1 = max(0, y1 - padding)
            x2 = min(frame.shape[1] - 1, x2 + padding)
            y2 = min(frame.shape[0] - 1, y2 + padding)

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                MERGED_COLOR,
                3,
            )

            ids = ",".join(
                str(object_id)
                for object_id in merged_cluster["object_ids"]
            )

            merged_label = (
                f"Merged {merged_cluster['id']}  "
                f"IDs:{ids}"
            )

            draw_text_box(
                annotated,
                merged_label,
                x1,
                max(y1, 24),
                MERGED_COLOR,
                scale=0.48,
                thickness=1,
            )

        legend_lines = (
            ("Green: tracked objects + trails", TRACK_COLOR),
            ("Magenta: overlap clusters", CLUSTER_COLOR),
            ("Orange: merged clusters", MERGED_COLOR),
        )

        legend_y = 24

        for legend_text, legend_color in legend_lines:
            draw_text_box(
                annotated,
                legend_text,
                10,
                legend_y,
                legend_color,
                scale=0.44,
                thickness=1,
            )

            legend_y += 26

        cv2.imshow(
            WINDOW_NAME,
            annotated,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            save_path = (
                SAVE_DIR
                / f"tracking_clustering_{timestamp_string()}.png"
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