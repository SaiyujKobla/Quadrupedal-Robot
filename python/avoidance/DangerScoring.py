from __future__ import annotations


from dataclasses import dataclass, field
from statistics import median
from typing import Optional


from sensors.SensorFusion import SensorFusion
from sensors.SensorModels import SensorSnapshot




ZONE_NAMES = (
   "FAR_LEFT",
   "LEFT",
   "CENTER",
   "RIGHT",
   "FAR_RIGHT",
)


ZONE_FRACTIONS = {
   "FAR_LEFT": (0.00, 0.20),
   "LEFT": (0.20, 0.40),
   "CENTER": (0.40, 0.60),
   "RIGHT": (0.60, 0.80),
   "FAR_RIGHT": (0.80, 1.00),
}


ZONE_HEADINGS_DEG = {
   "FAR_LEFT": 40.0,
   "LEFT": 20.0,
   "CENTER": 0.0,
   "RIGHT": -20.0,
   "FAR_RIGHT": -40.0,
}




@dataclass(frozen=True)
class DangerConfig:
   critical_distance_m: float = 0.35
   safe_distance_m: float = 2.00
   distance_weight: float = 50.0


   approach_speed_for_full_score_mps: float = 0.60
   approach_weight: float = 20.0


   area_fraction_for_full_score: float = 0.25
   area_weight: float = 12.0


   persistence_for_full_score_s: float = 1.50
   persistence_weight: float = 8.0


   unknown_depth_penalty: float = 15.0
   predicted_depth_penalty: float = 5.0


   zone_path_weights: dict[str, float] = field(
       default_factory=lambda: {
           "FAR_LEFT": 0.65,
           "LEFT": 0.82,
           "CENTER": 1.00,
           "RIGHT": 0.82,
           "FAR_RIGHT": 0.65,
       }
   )


   lidar_zone_half_width_deg: float = 9.0
   lidar_critical_clearance_m: float = 0.35
   lidar_safe_clearance_m: float = 1.80
   lidar_score_scale: float = 0.90
   lidar_max_age_s: float = 0.50


   ultrasonic_critical_m: float = 0.20
   ultrasonic_safe_m: float = 1.20
   ultrasonic_max_age_s: float = 0.35


   tof_baseline_m: Optional[float] = None
   tof_calibration_samples: int = 30
   tof_max_age_s: float = 0.35
   tof_invalid_grace_s: float = 0.20
   tof_drop_warning_m: float = 0.045
   tof_drop_stop_m: float = 0.090
   tof_rise_warning_m: float = 0.035
   tof_rise_stop_m: float = 0.070


   require_lidar: bool = True
   require_ultrasonic: bool = True
   require_tof: bool = True




@dataclass(frozen=True)
class GroundState:
   status: str
   score: float
   baseline_m: Optional[float]
   current_m: Optional[float]
   reason: str




@dataclass
class DangerReport:
   zone_scores: dict[str, float]
   cluster_scores: list[dict]
   lidar_zone_clearances_m: dict[str, Optional[float]]
   ground: GroundState
   force_stop: bool
   reasons: list[str]




class DangerEvaluator:
   def __init__(
       self,
       fusion: SensorFusion,
       config: Optional[DangerConfig] = None,
   ) -> None:
       self.fusion = fusion
       self.config = config or DangerConfig()
       self._tof_calibration_values: list[float] = []
       self._tof_baseline_m = self.config.tof_baseline_m


   @staticmethod
   def _clamp(
       value: float,
       low: float = 0.0,
       high: float = 100.0,
   ) -> float:
       return max(low, min(high, value))


   @staticmethod
   def _near_score(
       distance_m: float,
       critical_m: float,
       safe_m: float,
   ) -> float:
       if distance_m <= critical_m:
           return 1.0
       if distance_m >= safe_m:
           return 0.0


       return (safe_m - distance_m) / max(
           safe_m - critical_m,
           1e-6,
       )


   def _ground_state(self, snapshot: SensorSnapshot) -> GroundState:
       if not self.config.require_tof:
           return GroundState(
               status="DISABLED",
               score=0.0,
               baseline_m=self._tof_baseline_m,
               current_m=snapshot.tof_down_m,
               reason="Downward ToF requirement is disabled.",
           )


       if not snapshot.tof_fresh(self.config.tof_max_age_s):
           return GroundState(
               status="STALE",
               score=100.0,
               baseline_m=self._tof_baseline_m,
               current_m=None,
               reason="Downward ToF has no recent valid floor reading.",
           )


       if not snapshot.tof_valid:
           if snapshot.tof_age_s <= self.config.tof_invalid_grace_s:
               return GroundState(
                   status="TRANSIENT_INVALID",
                   score=35.0,
                   baseline_m=self._tof_baseline_m,
                   current_m=snapshot.tof_down_m,
                   reason=(
                       "A recent ToF reading was invalid; the filtered "
                       "estimate is being held briefly."
                   ),
               )


           return GroundState(
               status="NO_VALID_RETURN",
               score=100.0,
               baseline_m=self._tof_baseline_m,
               current_m=snapshot.tof_down_m,
               reason=(
                   "Downward ToF has remained invalid long enough that "
                   "ground clearance is unknown."
               ),
           )


       current_m = snapshot.tof_down_m
       if current_m is None:
           return GroundState(
               status="MISSING",
               score=100.0,
               baseline_m=self._tof_baseline_m,
               current_m=None,
               reason="Downward ToF value is missing.",
           )


       if self._tof_baseline_m is None:
           if not snapshot.tof_predicted:
               self._tof_calibration_values.append(float(current_m))


           if (
               len(self._tof_calibration_values)
               < self.config.tof_calibration_samples
           ):
               return GroundState(
                   status="CALIBRATING",
                   score=100.0,
                   baseline_m=None,
                   current_m=current_m,
                   reason=(
                       "Hold the robot still on flat floor while the "
                       "downward ToF baseline is calibrated."
                   ),
               )


           self._tof_baseline_m = float(
               median(self._tof_calibration_values)
           )


       delta_m = float(current_m) - float(self._tof_baseline_m)


       if delta_m >= self.config.tof_drop_stop_m:
           return GroundState(
               status="DROP_STOP",
               score=100.0,
               baseline_m=self._tof_baseline_m,
               current_m=current_m,
               reason="Downward ToF indicates a possible cliff or hole.",
           )


       if delta_m >= self.config.tof_drop_warning_m:
           fraction = (
               delta_m - self.config.tof_drop_warning_m
           ) / max(
               self.config.tof_drop_stop_m
               - self.config.tof_drop_warning_m,
               1e-6,
           )


           return GroundState(
               status="DROP_WARNING",
               score=60.0 + 35.0 * self._clamp(fraction, 0.0, 1.0),
               baseline_m=self._tof_baseline_m,
               current_m=current_m,
               reason=(
                   "Ground appears farther away than the calibrated floor."
               ),
           )


       if delta_m <= -self.config.tof_rise_stop_m:
           return GroundState(
               status="RISE_STOP",
               score=95.0,
               baseline_m=self._tof_baseline_m,
               current_m=current_m,
               reason=(
                   "Downward ToF indicates a sudden raised surface or "
                   "unexpected geometry."
               ),
           )


       if delta_m <= -self.config.tof_rise_warning_m:
           return GroundState(
               status="RISE_WARNING",
               score=55.0,
               baseline_m=self._tof_baseline_m,
               current_m=current_m,
               reason=(
                   "Ground appears closer than the calibrated floor."
               ),
           )


       return GroundState(
           status="SAFE",
           score=0.0,
           baseline_m=self._tof_baseline_m,
           current_m=current_m,
           reason=(
               "Ground clearance is near the calibrated floor baseline."
           ),
       )


   def _cluster_score(
       self,
       cluster: dict,
       frame_area: float,
   ) -> float:
       distance_m = cluster.get("closest_distance_m")


       if distance_m is None:
           distance_component = self.config.unknown_depth_penalty
       else:
           distance_component = (
               self.config.distance_weight
               * self._near_score(
                   float(distance_m),
                   self.config.critical_distance_m,
                   self.config.safe_distance_m,
               )
           )


       velocity_mps = cluster.get("average_distance_velocity_mps")
       approaching_speed = (
           max(0.0, -float(velocity_mps))
           if velocity_mps is not None
           else 0.0
       )
       approach_component = self.config.approach_weight * min(
           approaching_speed
           / max(
               self.config.approach_speed_for_full_score_mps,
               1e-6,
           ),
           1.0,
       )


       area_fraction = float(
           cluster.get("object_area_sum", 0.0)
       ) / max(frame_area, 1.0)
       area_component = self.config.area_weight * min(
           area_fraction
           / max(self.config.area_fraction_for_full_score, 1e-6),
           1.0,
       )


       time_seen = float(cluster.get("cluster_time_seen", 0.0))
       persistence_component = self.config.persistence_weight * min(
           time_seen
           / max(self.config.persistence_for_full_score_s, 1e-6),
           1.0,
       )


       predicted_component = 0.0
       if int(cluster.get("predicted_depth_count", 0)) > 0:
           predicted_component = self.config.predicted_depth_penalty


       return self._clamp(
           distance_component
           + approach_component
           + area_component
           + persistence_component
           + predicted_component
       )


   @staticmethod
   def _horizontal_overlap_fraction(
       box: tuple[int, int, int, int],
       zone_start_px: float,
       zone_end_px: float,
   ) -> float:
       x1, _, x2, _ = box
       overlap = max(
           0.0,
           min(float(x2), zone_end_px)
           - max(float(x1), zone_start_px),
       )
       return overlap / max(float(x2 - x1), 1.0)


   def evaluate(
       self,
       clusters: list[dict],
       snapshot: SensorSnapshot,
       frame_shape: tuple[int, int, int],
   ) -> DangerReport:
       height, width, _ = frame_shape
       frame_area = float(height * width)
       zone_scores = {name: 0.0 for name in ZONE_NAMES}
       reasons: list[str] = []


       for cluster in clusters:
           danger_score = self._cluster_score(cluster, frame_area)
           cluster["danger_score"] = danger_score


           for zone_name, (
               start_fraction,
               end_fraction,
           ) in ZONE_FRACTIONS.items():
               overlap_fraction = self._horizontal_overlap_fraction(
                   cluster["box"],
                   width * start_fraction,
                   width * end_fraction,
               )


               if overlap_fraction <= 0.0:
                   continue


               contribution = (
                   danger_score
                   * overlap_fraction
                   * self.config.zone_path_weights[zone_name]
               )
               zone_scores[zone_name] = self._clamp(
                   zone_scores[zone_name] + contribution
               )


       lidar_zone_clearances: dict[str, Optional[float]] = {}


       if snapshot.lidar_fresh(self.config.lidar_max_age_s):
           for zone_name in ZONE_NAMES:
               clearance = self.fusion.sector_clearance_m(
                   snapshot,
                   ZONE_HEADINGS_DEG[zone_name],
                   self.config.lidar_zone_half_width_deg,
               )
               lidar_zone_clearances[zone_name] = clearance


               if clearance is None:
                   continue


               lidar_danger = 100.0 * self._near_score(
                   clearance,
                   self.config.lidar_critical_clearance_m,
                   self.config.lidar_safe_clearance_m,
               )
               zone_scores[zone_name] = max(
                   zone_scores[zone_name],
                   self.config.lidar_score_scale * lidar_danger,
               )
       else:
           lidar_zone_clearances = {
               name: None for name in ZONE_NAMES
           }
           if self.config.require_lidar:
               reasons.append("RPLIDAR scan is missing or stale.")


       if snapshot.ultrasonic_fresh(
           self.config.ultrasonic_max_age_s
       ):
           ultrasonic_danger = 100.0 * self._near_score(
               float(snapshot.ultrasonic_m),
               self.config.ultrasonic_critical_m,
               self.config.ultrasonic_safe_m,
           )


           zone_scores["CENTER"] = max(
               zone_scores["CENTER"],
               ultrasonic_danger,
           )
           zone_scores["LEFT"] = max(
               zone_scores["LEFT"],
               ultrasonic_danger * 0.35,
           )
           zone_scores["RIGHT"] = max(
               zone_scores["RIGHT"],
               ultrasonic_danger * 0.35,
           )
       elif self.config.require_ultrasonic:
           reasons.append(
               "Forward ultrasonic reading is missing or stale."
           )


       ground = self._ground_state(snapshot)
       if ground.score >= 90.0:
           reasons.append(ground.reason)


       force_stop = ground.score >= 90.0


       if self.config.require_lidar and not snapshot.lidar_fresh(
           self.config.lidar_max_age_s
       ):
           force_stop = True


       if (
           self.config.require_ultrasonic
           and not snapshot.ultrasonic_fresh(
               self.config.ultrasonic_max_age_s
           )
       ):
           force_stop = True


       zone_scores = {
           name: self._clamp(score)
           for name, score in zone_scores.items()
       }


       return DangerReport(
           zone_scores=zone_scores,
           cluster_scores=clusters,
           lidar_zone_clearances_m=lidar_zone_clearances,
           ground=ground,
           force_stop=force_stop,
           reasons=reasons,
       )
