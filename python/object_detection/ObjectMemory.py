from __future__ import annotations


from collections import Counter, deque
from math import cos, radians, sin, tan
from typing import Optional

from sensors.Filters import ConstantVelocityKalman1D, Kalman1DConfig




class ObjectMemory:
   def __init__(
       self,
       history_size: int = 12,
       stale_after_s: float = 2.0,
       approaching_threshold_mps: float = 0.05,
       distance_filter_config: Optional[Kalman1DConfig] = None,
   ) -> None:
       self.history_size = history_size
       self.stale_after_s = stale_after_s
       self.approaching_threshold_mps = approaching_threshold_mps
       self.distance_filter_config = (
           distance_filter_config
           or Kalman1DConfig(
               process_acceleration_std=1.2,
               measurement_std=0.04,
               innovation_gate_sigma=4.5,
           )
       )
       self.objects: dict[int, dict] = {}


   def _new_record(self, obj: dict, timestamp: float) -> dict:
       return {
           "class_names": deque(maxlen=self.history_size),
           "confidences": deque(maxlen=self.history_size),
           "positions": deque(maxlen=self.history_size),
           "areas": deque(maxlen=self.history_size),
           "timestamps": deque(maxlen=self.history_size),
           "distances": deque(maxlen=self.history_size),
           "distance_timestamps": deque(maxlen=self.history_size),
           "distance_sources": deque(maxlen=self.history_size),
           "first_seen_time": timestamp,
           "last_seen_time": timestamp,
           "distance_filter": ConstantVelocityKalman1D(
               self.distance_filter_config
           ),
           "stable_class_name": obj["class_name"],
           "last_distance_source": None,
       }


   @staticmethod
   def _history_velocity(
       values: deque,
       timestamps: deque,
   ) -> Optional[float]:
       if len(values) < 2 or len(timestamps) < 2:
           return None


       dt = timestamps[-1] - timestamps[0]
       if dt <= 1e-6:
           return None


       return (float(values[-1]) - float(values[0])) / dt


   @staticmethod
   def _position_velocity(
       positions: deque,
       timestamps: deque,
   ) -> tuple[Optional[float], Optional[float]]:
       if len(positions) < 2 or len(timestamps) < 2:
           return None, None


       dt = timestamps[-1] - timestamps[0]
       if dt <= 1e-6:
           return None, None


       x0, y0 = positions[0]
       x1, y1 = positions[-1]
       return (x1 - x0) / dt, (y1 - y0) / dt


   @staticmethod
   def _stable_class(record: dict) -> str:
       weighted_votes: Counter[str] = Counter()


       for class_name, confidence in zip(
           record["class_names"],
           record["confidences"],
       ):
           weighted_votes[class_name] += max(float(confidence), 0.01)


       if not weighted_votes:
           return record["stable_class_name"]


       return weighted_votes.most_common(1)[0][0]


   def update(self, obj: dict, timestamp: float) -> dict:
       obj_id = int(obj["id"])
       if obj_id not in self.objects:
           self.objects[obj_id] = self._new_record(obj, timestamp)


       record = self.objects[obj_id]


       record["class_names"].append(obj["class_name"])
       record["confidences"].append(obj.get("confidence", 0.0))
       record["positions"].append(obj["center"])
       record["areas"].append(obj["area"])
       record["timestamps"].append(timestamp)
       record["last_seen_time"] = timestamp
       record["stable_class_name"] = self._stable_class(record)


       raw_distance = obj.get("distance_raw_m")
       source_predicted = bool(obj.get("distance_source_predicted", False))
       measurement_valid = (
           raw_distance is not None and not source_predicted
       )


       estimate = record["distance_filter"].step(
           raw_distance,
           timestamp,
           measurement_valid=measurement_valid,
       )


       if estimate.value is not None:
           record["distances"].append(estimate.value)
           record["distance_timestamps"].append(timestamp)


       if measurement_valid and obj.get("distance_source") is not None:
           record["last_distance_source"] = obj["distance_source"]
           record["distance_sources"].append(obj["distance_source"])


       velocity_x, velocity_y = self._position_velocity(
           record["positions"],
           record["timestamps"],
       )


       distance_velocity = estimate.velocity
       if distance_velocity is None:
           distance_velocity = self._history_velocity(
               record["distances"],
               record["distance_timestamps"],
           )


       if distance_velocity is None:
           motion_state = "UNKNOWN"
       elif distance_velocity < -self.approaching_threshold_mps:
           motion_state = "APPROACHING"
       elif distance_velocity > self.approaching_threshold_mps:
           motion_state = "RECEDING"
       else:
           motion_state = "STABLE"


       obj["detected_class_name"] = obj["class_name"]
       obj["stable_class_name"] = record["stable_class_name"]
       obj["class_name"] = record["stable_class_name"]
       obj["first_seen_time"] = record["first_seen_time"]
       obj["last_seen_time"] = record["last_seen_time"]
       obj["time_seen"] = timestamp - record["first_seen_time"]
       obj["velocity_x"] = velocity_x
       obj["velocity_y"] = velocity_y
       obj["distance_m"] = estimate.value
       obj["distance_velocity"] = distance_velocity
       obj["distance_is_predicted"] = (
           estimate.predicted_only or source_predicted
       )
       obj["distance_source"] = (
           obj.get("distance_source")
           or record["last_distance_source"]
       )
       obj["approaching_or_receding"] = motion_state


       distance_m = estimate.value
       bearing_deg = obj.get("bearing_deg")
       angular_width_deg = obj.get("angular_width_deg")


       if distance_m is not None and bearing_deg is not None:
           bearing_rad = radians(float(bearing_deg))
           obj["forward_m"] = distance_m * cos(bearing_rad)
           obj["left_m"] = distance_m * sin(bearing_rad)


           if angular_width_deg is not None:
               half_width = distance_m * tan(
                   radians(max(float(angular_width_deg), 0.1)) / 2.0
               )
               obj["estimated_half_width_m"] = min(
                   1.50,
                   max(0.03, half_width),
               )
       else:
           obj["forward_m"] = None
           obj["left_m"] = None
           obj["estimated_half_width_m"] = None


       return obj


   def prune(self, timestamp: float) -> None:
       stale_ids = [
           obj_id
           for obj_id, record in self.objects.items()
           if timestamp - record["last_seen_time"] > self.stale_after_s
       ]


       for obj_id in stale_ids:
           del self.objects[obj_id]


   def get_object_history(self, obj_id: int) -> Optional[dict]:
       return self.objects.get(obj_id)
