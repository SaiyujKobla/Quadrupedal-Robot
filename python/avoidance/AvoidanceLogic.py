from __future__ import annotations


from dataclasses import dataclass, field
from typing import Optional


from avoidance.DangerScoring import DangerReport, ZONE_HEADINGS_DEG, ZONE_NAMES
from sensors.SensorFusion import SensorFusion
from sensors.SensorModels import SensorSnapshot




@dataclass(frozen=True)
class PlannerConfig:
   corridor_width_m: float = 0.40
   corridor_lookahead_m: float = 3.0
   minimum_forward_clearance_m: float = 0.45
   preferred_forward_clearance_m: float = 1.50


   zone_stop_threshold: float = 88.0
   all_zones_stop_threshold: float = 82.0
   route_switch_improvement: float = 8.0


   turn_penalties: dict[str, float] = field(
       default_factory=lambda: {
           "FAR_LEFT": 16.0,
           "LEFT": 7.0,
           "CENTER": 0.0,
           "RIGHT": 7.0,
           "FAR_RIGHT": 16.0,
       }
   )




@dataclass(frozen=True)
class AvoidanceDecision:
   command: str
   selected_zone: Optional[str]
   selected_heading_deg: Optional[float]
   desired_speed_scale: float
   selected_clearance_m: Optional[float]
   zone_scores: dict[str, float]
   candidate_costs: dict[str, float]
   reason: str




class AvoidancePlanner:
   def __init__(
       self,
       fusion: SensorFusion,
       config: Optional[PlannerConfig] = None,
   ) -> None:
       self.fusion = fusion
       self.config = config or PlannerConfig()
       self._previous_selected_zone: Optional[str] = None


   @staticmethod
   def _clearance_penalty(
       clearance_m: float,
       minimum_m: float,
       preferred_m: float,
   ) -> float:
       if clearance_m >= preferred_m:
           return 0.0
       if clearance_m <= minimum_m:
           return 100.0


       fraction = (preferred_m - clearance_m) / max(
           preferred_m - minimum_m,
           1e-6,
       )
       return 55.0 * fraction


   @staticmethod
   def _command_for_heading(heading_deg: float) -> str:
       if heading_deg > 5.0:
           return "TURN LEFT"
       if heading_deg < -5.0:
           return "TURN RIGHT"
       return "FORWARD"


   def _stop(
       self,
       report: DangerReport,
       candidate_costs: Optional[dict[str, float]],
       reason: str,
   ) -> AvoidanceDecision:
       self._previous_selected_zone = None


       return AvoidanceDecision(
           command="STOP",
           selected_zone=None,
           selected_heading_deg=None,
           desired_speed_scale=0.0,
           selected_clearance_m=None,
           zone_scores=dict(report.zone_scores),
           candidate_costs=candidate_costs or {},
           reason=reason,
       )


   def choose(
       self,
       report: DangerReport,
       snapshot: SensorSnapshot,
   ) -> AvoidanceDecision:
       if report.force_stop:
           reason = (
               "; ".join(report.reasons)
               or "A required safety input is unsafe."
           )
           return self._stop(report, None, reason)


       if all(
           report.zone_scores[name]
           >= self.config.all_zones_stop_threshold
           for name in ZONE_NAMES
       ):
           return self._stop(
               report,
               None,
               "Every front zone has a high danger score.",
           )


       candidate_costs: dict[str, float] = {}
       candidate_clearances: dict[str, float] = {}
       blocked_zones: set[str] = set()


       for zone_name in ZONE_NAMES:
           heading_deg = ZONE_HEADINGS_DEG[zone_name]
           clearance = self.fusion.corridor_clearance_m(
               snapshot,
               heading_deg=heading_deg,
               corridor_width_m=self.config.corridor_width_m,
               max_forward_m=self.config.corridor_lookahead_m,
           )


           effective_clearance = (
               self.config.corridor_lookahead_m
               if clearance is None
               else clearance
           )
           candidate_clearances[zone_name] = effective_clearance


           clearance_penalty = self._clearance_penalty(
               effective_clearance,
               self.config.minimum_forward_clearance_m,
               self.config.preferred_forward_clearance_m,
           )


           if (
               effective_clearance
               < self.config.minimum_forward_clearance_m
               or report.zone_scores[zone_name]
               >= self.config.zone_stop_threshold
           ):
               blocked_zones.add(zone_name)


           candidate_costs[zone_name] = (
               report.zone_scores[zone_name]
               + clearance_penalty
               + self.config.turn_penalties[zone_name]
           )


       available_zones = [
           name for name in ZONE_NAMES if name not in blocked_zones
       ]


       if not available_zones:
           return self._stop(
               report,
               candidate_costs,
               "No front corridor has enough clearance.",
           )


       best_zone = min(
           available_zones,
           key=lambda name: candidate_costs[name],
       )
       selected_zone = best_zone
       kept_previous = False


       if self._previous_selected_zone in available_zones:
           previous_zone = self._previous_selected_zone
           improvement = (
               candidate_costs[previous_zone]
               - candidate_costs[best_zone]
           )


           if improvement < self.config.route_switch_improvement:
               selected_zone = previous_zone
               kept_previous = selected_zone != best_zone


       self._previous_selected_zone = selected_zone


       selected_heading = ZONE_HEADINGS_DEG[selected_zone]
       selected_clearance = candidate_clearances[selected_zone]
       selected_danger = report.zone_scores[selected_zone]


       danger_speed = max(
           0.20,
           1.0 - selected_danger / 110.0,
       )
       clearance_speed = max(
           0.20,
           min(
               1.0,
               selected_clearance
               / max(
                   self.config.preferred_forward_clearance_m,
                   1e-6,
               ),
           ),
       )
       ground_speed = max(
           0.20,
           1.0 - report.ground.score / 100.0,
       )
       desired_speed_scale = min(
           danger_speed,
           clearance_speed,
           ground_speed,
       )


       if kept_previous:
           reason = (
               f"Kept {selected_zone} to avoid left/right oscillation; "
               f"the new best route was not better by at least "
               f"{self.config.route_switch_improvement:.1f} cost points."
           )
       else:
           reason = (
               f"Selected {selected_zone} because it had the lowest "
               "safe cost after danger, clearance, and turn penalties."
           )


       return AvoidanceDecision(
           command=self._command_for_heading(selected_heading),
           selected_zone=selected_zone,
           selected_heading_deg=selected_heading,
           desired_speed_scale=desired_speed_scale,
           selected_clearance_m=selected_clearance,
           zone_scores=dict(report.zone_scores),
           candidate_costs=candidate_costs,
           reason=reason,
       )
