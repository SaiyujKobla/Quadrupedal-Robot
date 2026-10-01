from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from avoidance.AvoidanceLogic import AvoidancePlanner, PlannerConfig
from avoidance.DangerScoring import DangerConfig, DangerEvaluator, ZONE_NAMES
from object_detection.Cluster import ClusterManager
from object_detection.ClusterMemory import ClusterMemory
from object_detection.ObjectMemory import ObjectMemory
from sensors.SensorFusion import FusionConfig, SensorFusion
from sensors.Sensors import SensorHub


PROJECT_ROOT = Path(__file__).resolve().parent.parent

VIDEO_SOURCE = "http://192.168.1.225:81/stream"
MODEL_NAME = PROJECT_ROOT / "models" / "yolo11n.pt"

ROBOT_ESP32_IP = "192.168.1.220"
MAC_SENSOR_BIND_IP = "0.0.0.0"
SENSOR_UDP_PORT = 5005

ENABLE_LIDAR = True
ENABLE_ULTRASONIC = True
ENABLE_TOF = True

DISPLAY_CAMERA_WIDTH = 800
DISPLAY_CAMERA_HEIGHT = 600
PANEL_WIDTH = 620

WINDOW_NAME = "Range Sensing + Danger Scoring Test"
SAVE_DIR = PROJECT_ROOT / "poster_snapshots" / "range_sensing"
SAVE_DIR.mkdir(parents=True, exist_ok=True)


model = YOLO(MODEL_NAME)
class_names = model.names

object_memory = ObjectMemory(
    history_size=12,
    stale_after_s=2.0,
)

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

fusion = SensorFusion(
    FusionConfig(
        camera_hfov_deg=65.0,
        camera_yaw_offset_deg=0.0,
        lidar_angle_sign=1.0,
        lidar_angle_offset_deg=0.0,
        lidar_front_half_angle_deg=90.0,
        lidar_min_quality=0,
        lidar_association_margin_deg=1.5,
        min_lidar_points_per_object=2,
        object_distance_quantile=0.20,
        enable_ultrasonic_fallback=ENABLE_ULTRASONIC,
        ultrasonic_beam_half_angle_deg=15.0,
    )
)

danger_evaluator = DangerEvaluator(
    fusion,
    DangerConfig(
        critical_distance_m=0.35,
        safe_distance_m=2.00,
        approach_speed_for_full_score_mps=0.60,
        ultrasonic_critical_m=0.20,
        ultrasonic_safe_m=1.20,
        tof_baseline_m=None,
        tof_drop_warning_m=0.045,
        tof_drop_stop_m=0.090,
        tof_rise_warning_m=0.035,
        tof_rise_stop_m=0.070,
        require_lidar=ENABLE_LIDAR,
        require_ultrasonic=ENABLE_ULTRASONIC,
        require_tof=ENABLE_TOF,
    ),
)

planner = AvoidancePlanner(
    fusion,
    PlannerConfig(
        corridor_width_m=0.40,
        corridor_lookahead_m=3.0,
        minimum_forward_clearance_m=0.45,
        preferred_forward_clearance_m=1.50,
        zone_stop_threshold=88.0,
        all_zones_stop_threshold=82.0,
        route_switch_improvement=8.0,
    ),
)

sensor_hub = SensorHub(
    bind_host=MAC_SENSOR_BIND_IP,
    sensor_port=SENSOR_UDP_PORT,
    allowed_robot_ip=ROBOT_ESP32_IP,
)


def timestamp_string() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

def pad_to_height(image, target_height):
    height, width = image.shape[:2]

    if height >= target_height:
        return image

    total_padding = target_height - height

    top_padding = total_padding // 2
    bottom_padding = total_padding - top_padding

    return cv2.copyMakeBorder(
        image,
        top_padding,
        bottom_padding,
        0,
        0,
        cv2.BORDER_CONSTANT,
        value=(25, 25, 25),
    )

def format_age(age_s: float) -> str:
    if age_s == float("inf"):
        return "--"
    return f"{age_s:.2f}s"


def format_distance(value) -> str:
    if value is None:
        return "--"
    return f"{float(value):.2f} m"


def put_text(
    panel,
    text,
    x,
    y,
    scale=0.50,
    color=(230, 230, 230),
    thickness=1,
):
    cv2.putText(
        panel,
        str(text),
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def section_title(panel, text, y):
    cv2.line(
        panel,
        (20, y - 14),
        (PANEL_WIDTH - 20, y - 14),
        (80, 80, 80),
        1,
    )

    put_text(
        panel,
        text,
        20,
        y,
        scale=0.58,
        color=(255, 255, 255),
        thickness=2,
    )

    return y + 27


def build_range_panel(
    decision,
    danger_report,
    sensor_snapshot,
):
    panel = np.zeros(
        (DISPLAY_CAMERA_HEIGHT, PANEL_WIDTH, 3),
        dtype=np.uint8,
    )

    panel[:] = (25, 25, 25)

    y = 34

    put_text(
        panel,
        "RANGE SENSING + DANGER SCORING",
        20,
        y,
        scale=0.62,
        color=(255, 255, 255),
        thickness=2,
    )

    y += 38
    y = section_title(panel, "SENSOR READINGS", y)

    lidar_points = len(
        fusion.transformed_lidar_points(
            sensor_snapshot
        )
    )

    put_text(
        panel,
        (
            f"LiDAR: {lidar_points} front points   "
            f"age {format_age(sensor_snapshot.lidar_age_s)}"
        ),
        20,
        y,
    )

    y += 24

    put_text(
        panel,
        (
            f"Ultrasonic raw: "
            f"{format_distance(sensor_snapshot.ultrasonic_raw_m)}"
        ),
        20,
        y,
    )

    y += 22

    put_text(
        panel,
        (
            f"Ultrasonic filtered: "
            f"{format_distance(sensor_snapshot.ultrasonic_m)}   "
            f"age {format_age(sensor_snapshot.ultrasonic_age_s)}"
        ),
        20,
        y,
    )

    y += 24

    put_text(
        panel,
        (
            f"ToF raw: "
            f"{format_distance(sensor_snapshot.tof_down_raw_m)}"
        ),
        20,
        y,
    )

    y += 22

    put_text(
        panel,
        (
            f"ToF filtered: "
            f"{format_distance(sensor_snapshot.tof_down_m)}   "
            f"{danger_report.ground.status}"
        ),
        20,
        y,
    )

    y += 22

    put_text(
        panel,
        (
            f"ESP32: {sensor_snapshot.network_peer_ip or '--'}   "
            f"packet age {format_age(sensor_snapshot.network_age_s)}"
        ),
        20,
        y,
    )

    y += 35
    y = section_title(panel, "ZONE DANGER SCORES", y)

    abbreviations = {
        "FAR_LEFT": "FL",
        "LEFT": "L",
        "CENTER": "C",
        "RIGHT": "R",
        "FAR_RIGHT": "FR",
    }

    for zone_name in ZONE_NAMES:
        clearance = danger_report.lidar_zone_clearances_m.get(
            zone_name
        )

        clearance_text = (
            f"{clearance:.2f} m"
            if clearance is not None
            else "--"
        )

        put_text(
            panel,
            (
                f"{abbreviations[zone_name]}:  "
                f"danger {danger_report.zone_scores[zone_name]:5.1f}   "
                f"LiDAR {clearance_text}"
            ),
            30,
            y,
            scale=0.47,
            color=(120, 220, 255),
        )

        y += 21

    y += 14
    y = section_title(panel, "MERGED CLUSTER SCORES", y)

    if danger_report.cluster_scores:
        for cluster in danger_report.cluster_scores[:4]:
            distance = cluster.get("closest_distance_m")
            distance_text = (
                f"{distance:.2f} m"
                if distance is not None
                else "--"
            )

            motion = cluster.get(
                "approaching_or_receding",
                "UNKNOWN",
            )

            put_text(
                panel,
                (
                    f"Cluster {cluster['id']}:  "
                    f"danger {cluster['danger_score']:.1f}   "
                    f"{distance_text}   {motion}"
                ),
                20,
                y,
                scale=0.43,
                color=(0, 190, 255),
            )

            y += 21
    else:
        put_text(
            panel,
            "No merged clusters detected",
            20,
            y,
            scale=0.46,
            color=(160, 160, 160),
        )

        y += 22

    y += 12
    y = section_title(panel, "NAVIGATION DECISION", y)

    heading_text = (
        f"{decision.selected_heading_deg:+.0f} deg"
        if decision.selected_heading_deg is not None
        else "--"
    )

    put_text(
        panel,
        f"Command: {decision.command}",
        20,
        y,
        scale=0.55,
        color=(80, 180, 255),
        thickness=2,
    )

    y += 24

    put_text(
        panel,
        (
            f"Heading: {heading_text}    "
            f"Speed: {decision.desired_speed_scale:.2f}"
        ),
        20,
        y,
        scale=0.48,
    )

    return panel


sensor_hub.start()

cap = cv2.VideoCapture(VIDEO_SOURCE)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    sensor_hub.stop()
    raise RuntimeError("Could not open ESP32-CAM video source")


try:
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        frame_time = time.monotonic()
        sensor_snapshot = sensor_hub.snapshot()

        results = model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            conf=0.35,
            verbose=False,
        )

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

        fusion.enrich_objects(
            objects,
            sensor_snapshot,
            frame.shape,
        )

        for obj in objects:
            object_memory.update(
                obj,
                frame_time,
            )

        object_memory.prune(frame_time)

        clusters, merged_clusters = (
            cluster_manager.process(objects)
        )

        cluster_memory.update(
            merged_clusters,
            frame_time,
        )

        danger_report = danger_evaluator.evaluate(
            merged_clusters,
            sensor_snapshot,
            frame.shape,
        )

        cluster_memory.record_danger(
            danger_report.cluster_scores
        )

        decision = planner.choose(
            danger_report,
            sensor_snapshot,
        )

        camera_display = frame.copy()

        # camera_display = cv2.resize(
        #     frame,
        #     (
        #         DISPLAY_CAMERA_WIDTH,
        #         DISPLAY_CAMERA_HEIGHT,
        #     ),
        #     interpolation=cv2.INTER_LINEAR,
        # )

        # scale_x = DISPLAY_CAMERA_WIDTH / frame.shape[1]
        # scale_y = DISPLAY_CAMERA_HEIGHT / frame.shape[0]

        for obj in objects:
            x1, y1, x2, y2 = obj["box"]

            # x1 = int(x1 * scale_x)
            # y1 = int(y1 * scale_y)
            # x2 = int(x2 * scale_x)
            # y2 = int(y2 * scale_y)

            cv2.rectangle(
                camera_display,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

        for cluster in danger_report.cluster_scores:
            x1, y1, x2, y2 = cluster["box"]

            # x1 = int(x1 * scale_x)
            # y1 = int(y1 * scale_y)
            # x2 = int(x2 * scale_x)
            # y2 = int(y2 * scale_y)

            cv2.rectangle(
                camera_display,
                (x1, y1),
                (x2, y2),
                (0, 165, 255),
                3,
            )

        # panel = build_range_panel(
        #     decision,
        #     danger_report,
        #     sensor_snapshot,
        # )

        # display_frame = np.hstack(
        #     (
        #         camera_display,
        #         panel,
        #     )
        # )
        
        panel = build_range_panel(
            decision,
            danger_report,
            sensor_snapshot,
        )

        target_height = max(
            camera_display.shape[0],
            panel.shape[0],
        )

        camera_display = pad_to_height(
            camera_display,
            target_height,
        )

        panel = pad_to_height(
            panel,
            target_height,
        )

        display_frame = np.hstack(
            (
                camera_display,
                panel,
            )
        )

        cv2.imshow(
            WINDOW_NAME,
            display_frame,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            timestamp = timestamp_string()

            panel_path = (
                SAVE_DIR
                / f"range_sensing_panel_{timestamp}.png"
            )

            full_path = (
                SAVE_DIR
                / f"range_sensing_full_{timestamp}.png"
            )

            cv2.imwrite(
                str(panel_path),
                panel,
            )

            cv2.imwrite(
                str(full_path),
                display_frame,
            )

            print(f"Saved panel snapshot: {panel_path}")
            print(f"Saved full snapshot:  {full_path}")

        elif key == 27 or key == ord("q"):
            break


finally:
    cap.release()
    sensor_hub.stop()
    cv2.destroyAllWindows()