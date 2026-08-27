from __future__ import annotations


from dataclasses import dataclass
from math import cos, radians, sin, tan
from typing import Optional


import numpy as np


from sensors.SensorModels import LidarPoint, SensorSnapshot




@dataclass(frozen=True)
class FusionConfig:
   camera_hfov_deg: float = 65.0
   camera_yaw_offset_deg: float = 0.0


   lidar_angle_sign: float = 1.0
   lidar_angle_offset_deg: float = 0.0
   lidar_front_half_angle_deg: float = 90.0
   lidar_min_quality: int = 0
   lidar_min_distance_m: float = 0.15
   lidar_max_distance_m: float = 8.0
   max_lidar_age_s: float = 0.50


   lidar_association_margin_deg: float = 1.5
   min_lidar_points_per_object: int = 2
   object_distance_quantile: float = 0.20


   enable_ultrasonic_fallback: bool = True
   ultrasonic_beam_half_angle_deg: float = 15.0
   ultrasonic_max_age_s: float = 0.35
   ultrasonic_min_distance_m: float = 0.02
   ultrasonic_max_distance_m: float = 4.0


   minimum_estimated_half_width_m: float = 0.03
   maximum_estimated_half_width_m: float = 1.50




class SensorFusion:
   def __init__(self, config: Optional[FusionConfig] = None) -> None:
       self.config = config or FusionConfig()


   @staticmethod
   def normalize_angle(angle_deg: float) -> float:
       return (angle_deg + 180.0) % 360.0 - 180.0


   @staticmethod
   def angular_difference(angle_a_deg: float, angle_b_deg: float) -> float:
       return SensorFusion.normalize_angle(angle_a_deg - angle_b_deg)


   def lidar_bearing_deg(self, raw_angle_deg: float) -> float:
       return self.normalize_angle(
           self.config.lidar_angle_sign * raw_angle_deg
           + self.config.lidar_angle_offset_deg
       )


   def pixel_to_bearing_deg(self, x_px: float, frame_width: int) -> float:
       normalized_left = 0.5 - (x_px / max(frame_width, 1))
       return self.normalize_angle(
           normalized_left * self.config.camera_hfov_deg
           + self.config.camera_yaw_offset_deg
       )


   def transformed_lidar_points(
       self,
       snapshot: SensorSnapshot,
   ) -> list[LidarPoint]:
       if not snapshot.lidar_fresh(self.config.max_lidar_age_s):
           return []


       transformed = []


       for point in snapshot.lidar_points:
           bearing = self.lidar_bearing_deg(point.angle_deg)


           if point.quality < self.config.lidar_min_quality:
               continue
           if not (
               self.config.lidar_min_distance_m
               <= point.distance_m
               <= self.config.lidar_max_distance_m
           ):
               continue
           if abs(bearing) > self.config.lidar_front_half_angle_deg:
               continue


           transformed.append(
               LidarPoint(
                   angle_deg=bearing,
                   distance_m=point.distance_m,
                   quality=point.quality,
               )
           )


       return transformed


   def _set_metric_geometry(self, obj: dict, distance_m: float) -> None:
       bearing_deg = obj.get("bearing_deg")
       angular_width_deg = obj.get("angular_width_deg")


       if bearing_deg is None:
           obj["forward_m"] = None
           obj["left_m"] = None
           obj["estimated_half_width_m"] = None
           return


       bearing_rad = radians(float(bearing_deg))
       obj["forward_m"] = distance_m * cos(bearing_rad)
       obj["left_m"] = distance_m * sin(bearing_rad)


       if angular_width_deg is None:
           obj["estimated_half_width_m"] = None
           return


       estimated_half_width = distance_m * tan(
           radians(max(float(angular_width_deg), 0.1)) / 2.0
       )
       obj["estimated_half_width_m"] = min(
           self.config.maximum_estimated_half_width_m,
           max(
               self.config.minimum_estimated_half_width_m,
               estimated_half_width,
           ),
       )


   def _apply_ultrasonic_fallback(
       self,
       objects: list[dict],
       snapshot: SensorSnapshot,
       frame_shape: tuple[int, int, int],
   ) -> None:
       if not self.config.enable_ultrasonic_fallback:
           return
       if not snapshot.ultrasonic_fresh(
           self.config.ultrasonic_max_age_s
       ):
           return
       if snapshot.ultrasonic_m is None:
           return


       ultrasonic_m = float(snapshot.ultrasonic_m)
       if not (
           self.config.ultrasonic_min_distance_m
           <= ultrasonic_m
           <= self.config.ultrasonic_max_distance_m
       ):
           return


       frame_height, frame_width, _ = frame_shape
       frame_area = max(float(frame_height * frame_width), 1.0)
       candidates: list[tuple[float, dict]] = []


       for obj in objects:
           if obj.get("distance_raw_m") is not None:
               continue


           center_bearing = float(obj.get("bearing_deg", 999.0))
           angular_width = float(obj.get("angular_width_deg", 0.0))
           half_span = angular_width / 2.0


           if (
               abs(center_bearing)
               > self.config.ultrasonic_beam_half_angle_deg + half_span
           ):
               continue


           _, _, _, y2 = obj["box"]
           bottom_fraction = y2 / max(frame_height, 1)
           area_fraction = float(obj.get("area", 0.0)) / frame_area
           alignment = 1.0 - min(
               abs(center_bearing)
               / max(self.config.ultrasonic_beam_half_angle_deg, 1e-6),
               1.0,
           )


           match_score = (
               1.5 * alignment
               + 1.0 * bottom_fraction
               + 0.5 * min(area_fraction / 0.25, 1.0)
           )
           candidates.append((match_score, obj))


       if not candidates:
           return


       _, best_match = max(candidates, key=lambda item: item[0])
       best_match["distance_raw_m"] = ultrasonic_m
       best_match["distance_source"] = "ultrasonic"
       best_match["distance_source_predicted"] = bool(
           snapshot.ultrasonic_predicted
       )
       best_match["ultrasonic_fallback_match"] = True
       self._set_metric_geometry(best_match, ultrasonic_m)


   def enrich_objects(
       self,
       objects: list[dict],
       snapshot: SensorSnapshot,
       frame_shape: tuple[int, int, int],
   ) -> list[dict]:
       _, frame_width, _ = frame_shape
       points = self.transformed_lidar_points(snapshot)


       for obj in objects:
           x1, _, x2, _ = obj["box"]


           left_bearing = self.pixel_to_bearing_deg(x1, frame_width)
           right_bearing = self.pixel_to_bearing_deg(x2, frame_width)
           center_bearing = self.pixel_to_bearing_deg(
               (x1 + x2) / 2.0,
               frame_width,
           )


           angular_width = abs(
               self.angular_difference(left_bearing, right_bearing)
           )
           half_span = angular_width / 2.0
           association_half_span = (
               half_span + self.config.lidar_association_margin_deg
           )


           matching_points = [
               point
               for point in points
               if abs(
                   self.angular_difference(
                       point.angle_deg,
                       center_bearing,
                   )
               )
               <= association_half_span
           ]


           distance_raw_m: Optional[float] = None
           if len(matching_points) >= self.config.min_lidar_points_per_object:
               distances = np.asarray(
                   [point.distance_m for point in matching_points],
                   dtype=float,
               )
               distance_raw_m = float(
                   np.quantile(
                       distances,
                       self.config.object_distance_quantile,
                   )
               )


           obj["bearing_deg"] = center_bearing
           obj["left_bearing_deg"] = left_bearing
           obj["right_bearing_deg"] = right_bearing
           obj["angular_width_deg"] = angular_width
           obj["distance_raw_m"] = distance_raw_m
           obj["distance_source"] = (
               "lidar" if distance_raw_m is not None else None
           )
           obj["distance_source_predicted"] = False
           obj["lidar_point_count"] = len(matching_points)
           obj["ultrasonic_fallback_match"] = False


           if distance_raw_m is None:
               obj["forward_m"] = None
               obj["left_m"] = None
               obj["estimated_half_width_m"] = None
           else:
               self._set_metric_geometry(obj, distance_raw_m)


       self._apply_ultrasonic_fallback(
           objects,
           snapshot,
           frame_shape,
       )


       return objects


   def sector_clearance_m(
       self,
       snapshot: SensorSnapshot,
       center_bearing_deg: float,
       half_width_deg: float,
       quantile: float = 0.10,
   ) -> Optional[float]:
       distances = [
           point.distance_m
           for point in self.transformed_lidar_points(snapshot)
           if abs(
               self.angular_difference(
                   point.angle_deg,
                   center_bearing_deg,
               )
           )
           <= half_width_deg
       ]


       if not distances:
           return None


       return float(np.quantile(np.asarray(distances), quantile))


   def corridor_clearance_m(
       self,
       snapshot: SensorSnapshot,
       heading_deg: float,
       corridor_width_m: float,
       max_forward_m: float = 3.0,
       quantile: float = 0.05,
   ) -> Optional[float]:
       half_width_m = corridor_width_m / 2.0
       heading_rad = radians(heading_deg)
       along_distances = []


       for point in self.transformed_lidar_points(snapshot):
           point_angle_rad = radians(point.angle_deg)
           forward = point.distance_m * cos(point_angle_rad)
           left = point.distance_m * sin(point_angle_rad)


           along_corridor = (
               forward * cos(heading_rad) + left * sin(heading_rad)
           )
           lateral_to_corridor = (
               -forward * sin(heading_rad) + left * cos(heading_rad)
           )


           if (
               0.0 < along_corridor <= max_forward_m
               and abs(lateral_to_corridor) <= half_width_m
           ):
               along_distances.append(along_corridor)


       if not along_distances:
           return None


       return float(
           np.quantile(
               np.asarray(along_distances, dtype=float),
               quantile,
           )
       )
