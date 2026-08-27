from __future__ import annotations


from dataclasses import dataclass
from typing import Optional, Tuple




@dataclass(frozen=True)
class LidarPoint:
   angle_deg: float
   distance_m: float
   quality: int = 0




@dataclass(frozen=True)
class SensorSnapshot:
   timestamp: float


   lidar_points: Tuple[LidarPoint, ...] = ()
   lidar_age_s: float = float("inf")


   ultrasonic_raw_m: Optional[float] = None
   ultrasonic_m: Optional[float] = None
   ultrasonic_velocity_mps: Optional[float] = None
   ultrasonic_predicted: bool = False
   ultrasonic_age_s: float = float("inf")


   tof_down_raw_m: Optional[float] = None
   tof_down_m: Optional[float] = None
   tof_down_velocity_mps: Optional[float] = None
   tof_valid: bool = False
   tof_predicted: bool = False
   tof_age_s: float = float("inf")


   network_age_s: float = float("inf")
   network_peer_ip: Optional[str] = None


   def lidar_fresh(self, max_age_s: float = 0.50) -> bool:
       return bool(self.lidar_points) and self.lidar_age_s <= max_age_s


   def ultrasonic_fresh(self, max_age_s: float = 0.35) -> bool:
       return (
           self.ultrasonic_m is not None
           and self.ultrasonic_age_s <= max_age_s
       )


   def tof_fresh(self, max_age_s: float = 0.35) -> bool:
       return self.tof_down_m is not None and self.tof_age_s <= max_age_s


   def network_fresh(self, max_age_s: float = 0.50) -> bool:
       return self.network_age_s <= max_age_s
