from __future__ import annotations


from collections import deque
from math import hypot
from statistics import mean, median
from typing import Optional




class ClusterManager:
   def __init__(
       self,
       physical_merge_gap_m: float = 0.40,
       fallback_merge_distance_px: float = 30.0,
       max_depth_gap_m: float = 0.75,
       max_relative_depth_gap: float = 0.40,
   ) -> None:
       if physical_merge_gap_m < 0.0:
           raise ValueError("physical_merge_gap_m must be nonnegative")
       if fallback_merge_distance_px < 0.0:
           raise ValueError("fallback_merge_distance_px must be nonnegative")


       self.physical_merge_gap_m = float(physical_merge_gap_m)
       self.fallback_merge_distance_px = float(fallback_merge_distance_px)
       self.max_depth_gap_m = float(max_depth_gap_m)
       self.max_relative_depth_gap = float(max_relative_depth_gap)


   def process(self, objects: list[dict]) -> tuple[list[dict], list[dict]]:
       if not objects:
           return [], []


       overlap_groups = self._bfs_groups(
           objects,
           self._objects_overlap_and_depth_match,
       )
       clusters = [
           self._make_cluster(group, temporary_id=index)
           for index, group in enumerate(overlap_groups)
       ]


       merge_groups = self._bfs_groups(
           clusters,
           self._clusters_are_close,
       )


       merged_clusters = []
       for temporary_id, cluster_group in enumerate(merge_groups):
           merged_objects = []
           source_cluster_ids = []


           for cluster in cluster_group:
               merged_objects.extend(cluster["objects"])
               source_cluster_ids.append(cluster["temporary_id"])


           merged_cluster = self._make_cluster(
               merged_objects,
               temporary_id=temporary_id,
           )
           merged_cluster["source_cluster_ids"] = source_cluster_ids
           merged_clusters.append(merged_cluster)


       return clusters, merged_clusters


   def _bfs_groups(self, items: list, connected) -> list[list]:
       groups = []
       visited = set()


       for start_index in range(len(items)):
           if start_index in visited:
               continue


           queue = deque([start_index])
           visited.add(start_index)
           group = []


           while queue:
               current_index = queue.popleft()
               current_item = items[current_index]
               group.append(current_item)


               for other_index, other_item in enumerate(items):
                   if other_index in visited:
                       continue
                   if connected(current_item, other_item):
                       visited.add(other_index)
                       queue.append(other_index)


           groups.append(group)


       return groups


   @staticmethod
   def _boxes_overlap(box_a: tuple, box_b: tuple) -> bool:
       ax1, ay1, ax2, ay2 = box_a
       bx1, by1, bx2, by2 = box_b
       return ax1 <= bx2 and ax2 >= bx1 and ay1 <= by2 and ay2 >= by1


   def _depths_compatible(self, item_a: dict, item_b: dict) -> bool:
       distance_a = item_a.get("distance_m")
       distance_b = item_b.get("distance_m")


       if distance_a is None or distance_b is None:
           return True


       absolute_gap = abs(float(distance_a) - float(distance_b))
       relative_gap = absolute_gap / max(
           min(float(distance_a), float(distance_b)),
           0.05,
       )


       return not (
           absolute_gap > self.max_depth_gap_m
           and relative_gap > self.max_relative_depth_gap
       )


   def _objects_overlap_and_depth_match(
       self,
       object_a: dict,
       object_b: dict,
   ) -> bool:
       return self._boxes_overlap(
           object_a["box"],
           object_b["box"],
       ) and self._depths_compatible(object_a, object_b)


   @staticmethod
   def _box_distance_px(box_a: tuple, box_b: tuple) -> float:
       ax1, ay1, ax2, ay2 = box_a
       bx1, by1, bx2, by2 = box_b


       horizontal_gap = max(bx1 - ax2, ax1 - bx2, 0)
       vertical_gap = max(by1 - ay2, ay1 - by2, 0)
       return hypot(horizontal_gap, vertical_gap)


   @staticmethod
   def _metric_object_gap_m(
       object_a: dict,
       object_b: dict,
   ) -> Optional[float]:
       required = (
           object_a.get("forward_m"),
           object_a.get("left_m"),
           object_b.get("forward_m"),
           object_b.get("left_m"),
       )


       if any(value is None for value in required):
           return None


       center_distance = hypot(
           float(object_a["forward_m"])
           - float(object_b["forward_m"]),
           float(object_a["left_m"])
           - float(object_b["left_m"]),
       )


       radius_a = float(
           object_a.get("estimated_half_width_m") or 0.05
       )
       radius_b = float(
           object_b.get("estimated_half_width_m") or 0.05
       )


       return max(0.0, center_distance - radius_a - radius_b)


   def _clusters_are_close(self, cluster_a: dict, cluster_b: dict) -> bool:
       for object_a in cluster_a["objects"]:
           for object_b in cluster_b["objects"]:
               if not self._depths_compatible(object_a, object_b):
                   continue


               metric_gap = self._metric_object_gap_m(
                   object_a,
                   object_b,
               )


               if metric_gap is not None:
                   if metric_gap <= self.physical_merge_gap_m:
                       return True
                   continue


               pixel_gap = self._box_distance_px(
                   object_a["box"],
                   object_b["box"],
               )
               if pixel_gap <= self.fallback_merge_distance_px:
                   return True


       return False


   @staticmethod
   def _make_cluster(objects: list[dict], temporary_id: int) -> dict:
       x1 = min(obj["box"][0] for obj in objects)
       y1 = min(obj["box"][1] for obj in objects)
       x2 = max(obj["box"][2] for obj in objects)
       y2 = max(obj["box"][3] for obj in objects)


       distances = [
           float(obj["distance_m"])
           for obj in objects
           if obj.get("distance_m") is not None
       ]
       distance_velocities = [
           float(obj["distance_velocity"])
           for obj in objects
           if obj.get("distance_velocity") is not None
       ]
       bearings = [
           float(obj["bearing_deg"])
           for obj in objects
           if obj.get("bearing_deg") is not None
       ]
       forward_values = [
           float(obj["forward_m"])
           for obj in objects
           if obj.get("forward_m") is not None
       ]
       left_values = [
           float(obj["left_m"])
           for obj in objects
           if obj.get("left_m") is not None
       ]
       distance_sources = sorted(
           {
               str(obj["distance_source"])
               for obj in objects
               if obj.get("distance_source") is not None
           }
       )


       closest_distance = min(distances) if distances else None
       average_distance = mean(distances) if distances else None
       median_distance = median(distances) if distances else None
       average_distance_velocity = (
           mean(distance_velocities) if distance_velocities else None
       )


       if average_distance_velocity is None:
           motion_state = "UNKNOWN"
       elif average_distance_velocity < -0.05:
           motion_state = "APPROACHING"
       elif average_distance_velocity > 0.05:
           motion_state = "RECEDING"
       else:
           motion_state = "STABLE"


       return {
           "id": temporary_id,
           "temporary_id": temporary_id,
           "class_name": "cluster",
           "objects": list(objects),
           "object_ids": sorted({int(obj["id"]) for obj in objects}),
           "box": (x1, y1, x2, y2),
           "center": ((x1 + x2) // 2, (y1 + y2) // 2),
           "area": max(0, x2 - x1) * max(0, y2 - y1),
           "object_area_sum": sum(obj["area"] for obj in objects),
           "count": len(objects),
           "closest_distance_m": closest_distance,
           "average_distance_m": average_distance,
           "median_distance_m": median_distance,
           "average_distance_velocity_mps": average_distance_velocity,
           "approaching_or_receding": motion_state,
           "min_bearing_deg": min(bearings) if bearings else None,
           "max_bearing_deg": max(bearings) if bearings else None,
           "average_forward_m": (
               mean(forward_values) if forward_values else None
           ),
           "average_left_m": mean(left_values) if left_values else None,
           "distance_sources": distance_sources,
           "unknown_depth_count": sum(
               obj.get("distance_m") is None for obj in objects
           ),
           "predicted_depth_count": sum(
               bool(obj.get("distance_is_predicted")) for obj in objects
           ),
       }
