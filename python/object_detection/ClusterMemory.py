from __future__ import annotations


from collections import deque
from math import hypot
from statistics import mean
from typing import Optional




class ClusterMemory:
   def __init__(
       self,
       history_size: int = 15,
       stale_after_s: float = 2.5,
       center_match_distance_px: float = 100.0,
       depth_match_distance_m: float = 0.80,
   ) -> None:
       self.history_size = history_size
       self.stale_after_s = stale_after_s
       self.center_match_distance_px = center_match_distance_px
       self.depth_match_distance_m = depth_match_distance_m


       self._records: dict[int, dict] = {}
       self._next_id = 0


   def _new_record(self, cluster: dict, timestamp: float) -> dict:
       return {
           "object_ids": set(cluster["object_ids"]),
           "center": cluster["center"],
           "closest_distance_m": cluster.get("closest_distance_m"),
           "first_seen_time": timestamp,
           "last_seen_time": timestamp,
           "closest_distances": deque(maxlen=self.history_size),
           "average_distances": deque(maxlen=self.history_size),
           "distance_velocities": deque(maxlen=self.history_size),
           "danger_scores": deque(maxlen=self.history_size),
       }


   def _match_score(self, cluster: dict, record: dict) -> float:
       current_ids = set(cluster["object_ids"])
       previous_ids = record["object_ids"]
       union = current_ids | previous_ids


       overlap_score = (
           len(current_ids & previous_ids) / len(union)
           if union
           else 0.0
       )


       cx, cy = cluster["center"]
       px, py = record["center"]
       center_distance = hypot(cx - px, cy - py)
       center_score = max(
           0.0,
           1.0
           - center_distance / max(self.center_match_distance_px, 1.0),
       )


       current_depth = cluster.get("closest_distance_m")
       previous_depth = record.get("closest_distance_m")


       if current_depth is None or previous_depth is None:
           depth_score = 0.25
       else:
           depth_gap = abs(float(current_depth) - float(previous_depth))
           depth_score = max(
               0.0,
               1.0
               - depth_gap / max(self.depth_match_distance_m, 0.01),
           )


       if overlap_score == 0.0 and center_score < 0.35:
           return 0.0


       return (
           0.65 * overlap_score
           + 0.25 * center_score
           + 0.10 * depth_score
       )


   @staticmethod
   def _append_if_number(history: deque, value: object) -> None:
       if isinstance(value, (int, float)):
           history.append(float(value))


   @staticmethod
   def _average_or_none(history: deque) -> Optional[float]:
       return mean(history) if history else None


   def update(self, clusters: list[dict], timestamp: float) -> list[dict]:
       available_ids = set(self._records)


       for cluster in clusters:
           best_id = None
           best_score = 0.0


           for record_id in available_ids:
               score = self._match_score(
                   cluster,
                   self._records[record_id],
               )
               if score > best_score:
                   best_score = score
                   best_id = record_id


           if best_id is None or best_score < 0.30:
               best_id = self._next_id
               self._next_id += 1
               self._records[best_id] = self._new_record(
                   cluster,
                   timestamp,
               )
           else:
               available_ids.remove(best_id)


           record = self._records[best_id]
           record["object_ids"] = set(cluster["object_ids"])
           record["center"] = cluster["center"]
           record["closest_distance_m"] = cluster.get(
               "closest_distance_m"
           )
           record["last_seen_time"] = timestamp


           self._append_if_number(
               record["closest_distances"],
               cluster.get("closest_distance_m"),
           )
           self._append_if_number(
               record["average_distances"],
               cluster.get("average_distance_m"),
           )
           self._append_if_number(
               record["distance_velocities"],
               cluster.get("average_distance_velocity_mps"),
           )


           cluster["id"] = best_id
           cluster["cluster_first_seen_time"] = record[
               "first_seen_time"
           ]
           cluster["cluster_last_seen_time"] = record["last_seen_time"]
           cluster["cluster_time_seen"] = (
               timestamp - record["first_seen_time"]
           )
           cluster["rolling_closest_distance_m"] = (
               self._average_or_none(record["closest_distances"])
           )
           cluster["rolling_average_distance_m"] = (
               self._average_or_none(record["average_distances"])
           )
           cluster["rolling_distance_velocity_mps"] = (
               self._average_or_none(record["distance_velocities"])
           )
           cluster["rolling_danger_score"] = self._average_or_none(
               record["danger_scores"]
           )


       self.prune(timestamp)
       return clusters


   def record_danger(self, clusters: list[dict]) -> None:
       for cluster in clusters:
           cluster_id = cluster.get("id")
           danger_score = cluster.get("danger_score")


           if cluster_id not in self._records:
               continue
           if not isinstance(danger_score, (int, float)):
               continue


           history = self._records[cluster_id]["danger_scores"]
           history.append(float(danger_score))
           cluster["rolling_danger_score"] = self._average_or_none(history)


   def prune(self, timestamp: float) -> None:
       stale_ids = [
           cluster_id
           for cluster_id, record in self._records.items()
           if timestamp - record["last_seen_time"] > self.stale_after_s
       ]


       for cluster_id in stale_ids:
           del self._records[cluster_id]
