from __future__ import annotations

import time
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
from sensors.Sensors import (
    CommandNetworkConfig,
    RobotCommandSender,
    SensorHub,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent

VIDEO_SOURCE = "http://192.168.1.225:81/stream"
MODEL_NAME = PROJECT_ROOT / "models" / "yolo11n.pt"


ROBOT_ESP32_IP = "192.168.1.220"
MAC_SENSOR_BIND_IP = "0.0.0.0"

SENSOR_UDP_PORT = 5005
COMMAND_UDP_PORT = 5006


ENABLE_LIDAR = True
ENABLE_ULTRASONIC = True
ENABLE_TOF = True

ENABLE_COMMAND_TX = False


DISPLAY_CAMERA_WIDTH = 800
DISPLAY_CAMERA_HEIGHT = 600
PANEL_WIDTH = 620

WINDOW_NAME = "Quadruped Perception + Avoidance"


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


command_sender = (
    RobotCommandSender(
        CommandNetworkConfig(
            robot_host=ROBOT_ESP32_IP,
            command_port=COMMAND_UDP_PORT,
        )
    )
    if ENABLE_COMMAND_TX
    else None
)


def scaled_box(box, scale_x, scale_y):
    x1, y1, x2, y2 = box

    return (
        int(x1 * scale_x),
        int(y1 * scale_y),
        int(x2 * scale_x),
        int(y2 * scale_y),
    )


def put_panel_text(
    panel,
    text,
    x,
    y,
    scale=0.52,
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


def draw_wrapped_text(
    panel,
    text,
    x,
    y,
    max_width,
    scale=0.48,
    color=(220, 220, 220),
    thickness=1,
    line_height=22,
    max_lines=3,
):
    words = str(text).split()

    if not words:
        return y

    lines = []
    current = ""

    for word in words:
        test = word if not current else current + " " + word

        text_width = cv2.getTextSize(
            test,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            thickness,
        )[0][0]

        if text_width <= max_width:
            current = test
        else:
            if current:
                lines.append(current)

            current = word

            if len(lines) >= max_lines:
                break

    if current and len(lines) < max_lines:
        lines.append(current)

    for line in lines:
        put_panel_text(
            panel,
            line,
            x,
            y,
            scale=scale,
            color=color,
            thickness=thickness,
        )

        y += line_height

    return y


def section_title(panel, text, y):
    cv2.line(
        panel,
        (20, y - 14),
        (PANEL_WIDTH - 20, y - 14),
        (80, 80, 80),
        1,
    )

    put_panel_text(
        panel,
        text,
        20,
        y,
        scale=0.58,
        color=(255, 255, 255),
        thickness=2,
    )

    return y + 26


def build_dashboard(
    decision,
    danger_report,
    sensor_snapshot,
    objects,
    errors,
):
    panel = np.zeros(
        (DISPLAY_CAMERA_HEIGHT, PANEL_WIDTH, 3),
        dtype=np.uint8,
    )

    panel[:] = (25, 25, 25)

    y = 34

    put_panel_text(
        panel,
        "QUADRUPED PERCEPTION STATUS",
        20,
        y,
        scale=0.67,
        color=(255, 255, 255),
        thickness=2,
    )

    y += 30

    tx_text = (
        "COMMAND TX: ON"
        if ENABLE_COMMAND_TX
        else "COMMAND TX: OFF"
    )

    tx_color = (
        (0, 200, 255)
        if ENABLE_COMMAND_TX
        else (120, 220, 120)
    )

    put_panel_text(
        panel,
        tx_text,
        20,
        y,
        scale=0.55,
        color=tx_color,
        thickness=2,
    )

    y += 34
    y = section_title(panel, "AVOIDANCE DECISION", y)

    selected_heading_text = (
        f"{decision.selected_heading_deg:+.0f} deg"
        if decision.selected_heading_deg is not None
        else "--"
    )

    put_panel_text(
        panel,
        f"Command: {decision.command}",
        20,
        y,
        scale=0.58,
        color=(80, 180, 255),
        thickness=2,
    )

    y += 24

    put_panel_text(
        panel,
        (
            f"Heading: {selected_heading_text}    "
            f"Speed: {decision.desired_speed_scale:.2f}"
        ),
        20,
        y,
        scale=0.50,
    )

    y += 24

    zone_abbreviations = ("FL", "L", "C", "R", "FR")

    zone_text = "   ".join(
        f"{abbreviation}:{decision.zone_scores[name]:.0f}"
        for abbreviation, name in zip(
            zone_abbreviations,
            ZONE_NAMES,
        )
    )

    put_panel_text(
        panel,
        f"Zones: {zone_text}",
        20,
        y,
        scale=0.48,
    )

    y += 24

    put_panel_text(
        panel,
        "Reason:",
        20,
        y,
        scale=0.48,
        color=(180, 180, 180),
    )

    y += 19

    y = draw_wrapped_text(
        panel,
        decision.reason,
        35,
        y,
        PANEL_WIDTH - 60,
        scale=0.44,
        color=(215, 215, 215),
        line_height=19,
        max_lines=2,
    )

    y += 11
    y = section_title(panel, "SENSORS", y)

    lidar_points = len(
        fusion.transformed_lidar_points(
            sensor_snapshot
        )
    )

    lidar_age_text = (
        "--"
        if sensor_snapshot.lidar_age_s == float("inf")
        else f"{sensor_snapshot.lidar_age_s:.2f}s"
    )

    put_panel_text(
        panel,
        f"LiDAR: {lidar_points} front points   age {lidar_age_text}",
        20,
        y,
        scale=0.48,
    )

    y += 22

    if ENABLE_TOF:
        if (
            sensor_snapshot.tof_down_raw_m is not None
            and sensor_snapshot.tof_down_m is not None
        ):
            tof_text = (
                f"ToF: "
                f"{sensor_snapshot.tof_down_raw_m:.3f} / "
                f"{sensor_snapshot.tof_down_m:.3f} m   "
                f"{danger_report.ground.status}"
            )
        else:
            tof_text = (
                f"ToF: --   "
                f"{danger_report.ground.status}"
            )
    else:
        tof_text = "ToF: disabled"

    put_panel_text(
        panel,
        tof_text,
        20,
        y,
        scale=0.48,
    )

    y += 22

    if ENABLE_ULTRASONIC:
        if (
            sensor_snapshot.ultrasonic_raw_m is not None
            and sensor_snapshot.ultrasonic_m is not None
        ):
            ultrasonic_text = (
                f"Ultrasonic: "
                f"{sensor_snapshot.ultrasonic_raw_m:.2f} / "
                f"{sensor_snapshot.ultrasonic_m:.2f} m"
            )
        else:
            ultrasonic_text = "Ultrasonic: --"
    else:
        ultrasonic_text = "Ultrasonic: disabled"

    put_panel_text(
        panel,
        ultrasonic_text,
        20,
        y,
        scale=0.48,
    )

    y += 22

    peer_text = (
        sensor_snapshot.network_peer_ip
        or "--"
    )

    network_age_text = (
        "--"
        if sensor_snapshot.network_age_s == float("inf")
        else f"{sensor_snapshot.network_age_s:.2f}s"
    )

    put_panel_text(
        panel,
        (
            f"ESP32: {peer_text}   "
            f"packet age {network_age_text}"
        ),
        20,
        y,
        scale=0.48,
    )

    y += 32
    y = section_title(panel, "DETECTED OBJECTS", y)

    if not objects:
        put_panel_text(
            panel,
            "No tracked objects",
            20,
            y,
            scale=0.48,
            color=(160, 160, 160),
        )

        y += 22

    else:
        max_objects_to_show = 6

        for obj in objects[:max_objects_to_show]:
            stable_class = obj.get(
                "stable_class_name",
                obj["class_name"],
            )

            if obj.get("distance_m") is None:
                depth_text = "depth --"
            else:
                source = obj.get(
                    "distance_source"
                ) or "?"

                predicted_marker = (
                    "P"
                    if obj.get(
                        "distance_is_predicted"
                    )
                    else "M"
                )

                depth_text = (
                    f"{obj['distance_m']:.2f}m "
                    f"{source[0].upper()}"
                    f"{predicted_marker}"
                )

            motion = obj.get(
                "approaching_or_receding",
                "UNKNOWN",
            )

            object_text = (
                f"ID {obj['id']}  "
                f"{stable_class}  "
                f"{obj['confidence']:.2f}  "
                f"{depth_text}  "
                f"{motion[:3]}"
            )

            put_panel_text(
                panel,
                object_text,
                20,
                y,
                scale=0.45,
                color=(130, 255, 130),
            )

            y += 20

        if len(objects) > max_objects_to_show:
            put_panel_text(
                panel,
                (
                    f"+ {len(objects) - max_objects_to_show} "
                    f"more objects"
                ),
                20,
                y,
                scale=0.42,
                color=(160, 160, 160),
            )

            y += 20

    if y < DISPLAY_CAMERA_HEIGHT - 120:
        y += 10
        y = section_title(
            panel,
            "DANGER CLUSTERS",
            y,
        )

        merged_scores = danger_report.cluster_scores

        if not merged_scores:
            put_panel_text(
                panel,
                "No merged danger clusters",
                20,
                y,
                scale=0.46,
                color=(160, 160, 160),
            )

            y += 20

        else:
            max_clusters_to_show = 4

            for cluster in merged_scores[
                :max_clusters_to_show
            ]:
                distance = cluster.get(
                    "closest_distance_m"
                )

                distance_text = (
                    f"{distance:.2f}m"
                    if distance is not None
                    else "--"
                )

                rolling = cluster.get(
                    "rolling_danger_score"
                )

                rolling_text = (
                    f"{rolling:.0f}"
                    if isinstance(
                        rolling,
                        (int, float),
                    )
                    else "--"
                )

                cluster_text = (
                    f"Cluster {cluster['id']}   "
                    f"Danger {cluster['danger_score']:.0f}   "
                    f"Avg {rolling_text}   "
                    f"{distance_text}"
                )

                put_panel_text(
                    panel,
                    cluster_text,
                    20,
                    y,
                    scale=0.44,
                    color=(0, 190, 255),
                )

                y += 20

    if errors:
        error_y = DISPLAY_CAMERA_HEIGHT - 48

        put_panel_text(
            panel,
            "ERROR:",
            20,
            error_y,
            scale=0.46,
            color=(80, 80, 255),
            thickness=2,
        )

        draw_wrapped_text(
            panel,
            errors[0],
            85,
            error_y,
            PANEL_WIDTH - 105,
            scale=0.40,
            color=(100, 100, 255),
            line_height=17,
            max_lines=2,
        )

    return panel


sensor_hub.start()

cap = cv2.VideoCapture(VIDEO_SOURCE)

cap.set(
    cv2.CAP_PROP_BUFFERSIZE,
    1,
)


if not cap.isOpened():
    sensor_hub.stop()

    if command_sender:
        command_sender.send_stop()
        command_sender.close()

    raise RuntimeError(
        "Could not open ESP32-CAM video source"
    )


try:
    while True:
        ret, frame = cap.read()

        if not ret:
            break


        frame_time = time.monotonic()

        sensor_snapshot = (
            sensor_hub.snapshot()
        )


        results = model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            conf=0.35,
            verbose=False,
        )


        objects = []

        height, width, _ = frame.shape


        boxes = results[0].boxes

        if boxes is not None:
            for box in boxes:
                if box.id is None:
                    continue


                track_id = int(
                    box.id.item()
                )

                class_id = int(
                    box.cls.item()
                )

                confidence = float(
                    box.conf.item()
                )

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0],
                )


                center_x = int(
                    (x1 + x2) / 2
                )

                center_y = int(
                    (y1 + y2) / 2
                )

                area = (
                    max(0, x2 - x1)
                    * max(0, y2 - y1)
                )


                objects.append(
                    {
                        "id": track_id,
                        "class_id": class_id,
                        "class_name": class_names[
                            class_id
                        ],
                        "confidence": confidence,
                        "box": (
                            x1,
                            y1,
                            x2,
                            y2,
                        ),
                        "center": (
                            center_x,
                            center_y,
                        ),
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

        object_memory.prune(
            frame_time
        )


        clusters, merged_clusters = (
            cluster_manager.process(
                objects
            )
        )

        cluster_memory.update(
            merged_clusters,
            frame_time,
        )


        danger_report = (
            danger_evaluator.evaluate(
                merged_clusters,
                sensor_snapshot,
                frame.shape,
            )
        )

        cluster_memory.record_danger(
            danger_report.cluster_scores
        )


        decision = planner.choose(
            danger_report,
            sensor_snapshot,
        )


        if command_sender:
            command_sender.send_decision(
                decision
            )


        camera_display = cv2.resize(
            frame,
            (
                DISPLAY_CAMERA_WIDTH,
                DISPLAY_CAMERA_HEIGHT,
            ),
            interpolation=cv2.INTER_LINEAR,
        )


        scale_x = (
            DISPLAY_CAMERA_WIDTH / width
        )

        scale_y = (
            DISPLAY_CAMERA_HEIGHT / height
        )


        for fraction in (
            0.20,
            0.40,
            0.60,
            0.80,
        ):
            x = int(
                DISPLAY_CAMERA_WIDTH
                * fraction
            )

            cv2.line(
                camera_display,
                (x, 0),
                (
                    x,
                    DISPLAY_CAMERA_HEIGHT,
                ),
                (150, 150, 150),
                1,
            )


        for obj in objects:
            x1, y1, x2, y2 = (
                scaled_box(
                    obj["box"],
                    scale_x,
                    scale_y,
                )
            )

            cv2.rectangle(
                camera_display,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )


        for cluster in clusters:
            x1, y1, x2, y2 = (
                scaled_box(
                    cluster["box"],
                    scale_x,
                    scale_y,
                )
            )

            padding = 3

            x1 = max(
                0,
                x1 - padding,
            )

            y1 = max(
                0,
                y1 - padding,
            )

            x2 = min(
                DISPLAY_CAMERA_WIDTH - 1,
                x2 + padding,
            )

            y2 = min(
                DISPLAY_CAMERA_HEIGHT - 1,
                y2 + padding,
            )

            cv2.rectangle(
                camera_display,
                (x1, y1),
                (x2, y2),
                (255, 0, 255),
                2,
            )


        for cluster in (
            danger_report.cluster_scores
        ):
            x1, y1, x2, y2 = (
                scaled_box(
                    cluster["box"],
                    scale_x,
                    scale_y,
                )
            )

            padding = 7

            x1 = max(
                0,
                x1 - padding,
            )

            y1 = max(
                0,
                y1 - padding,
            )

            x2 = min(
                DISPLAY_CAMERA_WIDTH - 1,
                x2 + padding,
            )

            y2 = min(
                DISPLAY_CAMERA_HEIGHT - 1,
                y2 + padding,
            )

            cv2.rectangle(
                camera_display,
                (x1, y1),
                (x2, y2),
                (0, 165, 255),
                3,
            )


        errors = sensor_hub.errors()

        if (
            command_sender
            and command_sender.last_error
        ):
            errors.append(
                command_sender.last_error
            )


        dashboard = build_dashboard(
            decision,
            danger_report,
            sensor_snapshot,
            objects,
            errors,
        )


        display_frame = np.hstack(
            (
                camera_display,
                dashboard,
            )
        )


        cv2.imshow(
            WINDOW_NAME,
            display_frame,
        )


        key = cv2.waitKey(1) & 0xFF

        if key == 27 or key == ord("q"):
            break


finally:
    cap.release()

    sensor_hub.stop()


    if command_sender:
        command_sender.send_stop()
        command_sender.close()


    cv2.destroyAllWindows()