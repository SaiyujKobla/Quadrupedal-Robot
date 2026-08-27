from __future__ import annotations


import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional


from sensors.Filters import ConstantVelocityKalman1D, Kalman1DConfig
from networking.NetworkProtocol import (
   LidarChunkPacket,
   RangePacket,
   decode_sensor_datagram,
   encode_command_packet,
)
from sensors.SensorModels import LidarPoint, SensorSnapshot




@dataclass(frozen=True)
class SensorNetworkConfig:
   bind_host: str = "0.0.0.0"
   sensor_port: int = 5005
   allowed_robot_ip: Optional[str] = None
   receive_buffer_bytes: int = 4096
   socket_timeout_s: float = 0.20
   incomplete_scan_timeout_s: float = 0.35




class NetworkSensorReceiver:
   def __init__(
       self,
       config: Optional[SensorNetworkConfig] = None,
       ultrasonic_filter_config: Optional[Kalman1DConfig] = None,
       tof_filter_config: Optional[Kalman1DConfig] = None,
   ) -> None:
       self.config = config or SensorNetworkConfig()


       self._ultrasonic_filter = ConstantVelocityKalman1D(
           ultrasonic_filter_config
           or Kalman1DConfig(
               process_acceleration_std=2.0,
               measurement_std=0.025,
               innovation_gate_sigma=5.0,
           )
       )
       self._tof_filter = ConstantVelocityKalman1D(
           tof_filter_config
           or Kalman1DConfig(
               process_acceleration_std=0.6,
               measurement_std=0.012,
               innovation_gate_sigma=5.0,
           )
       )


       self._lock = threading.Lock()
       self._stop_event = threading.Event()
       self._thread: Optional[threading.Thread] = None
       self._socket: Optional[socket.socket] = None


       self._latest_scan: tuple[LidarPoint, ...] = ()
       self._latest_scan_time = 0.0
       self._scan_assemblies: dict[int, dict] = {}


       self._ultrasonic_raw_m: Optional[float] = None
       self._ultrasonic_m: Optional[float] = None
       self._ultrasonic_velocity_mps: Optional[float] = None
       self._ultrasonic_predicted = False
       self._ultrasonic_last_valid_time = 0.0


       self._tof_raw_m: Optional[float] = None
       self._tof_m: Optional[float] = None
       self._tof_velocity_mps: Optional[float] = None
       self._tof_valid = False
       self._tof_predicted = False
       self._tof_last_valid_time = 0.0


       self._last_network_packet_time = 0.0
       self._last_peer_ip: Optional[str] = None
       self._last_sequence: Optional[int] = None
       self.last_error: Optional[str] = None


   def start(self) -> None:
       if self._thread and self._thread.is_alive():
           return


       self._stop_event.clear()
       self._thread = threading.Thread(
           target=self._run,
           name="NetworkSensorReceiver",
           daemon=True,
       )
       self._thread.start()


   @staticmethod
   def _positive_meters(value_mm: object) -> Optional[float]:
       if not isinstance(value_mm, (int, float)):
           return None


       value_m = float(value_mm) / 1000.0
       return value_m if value_m > 0.0 else None


   def _run(self) -> None:
       while not self._stop_event.is_set():
           try:
               sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
               sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
               sock.bind((self.config.bind_host, self.config.sensor_port))
               sock.settimeout(self.config.socket_timeout_s)
               self._socket = sock
               self.last_error = None


               while not self._stop_event.is_set():
                   try:
                       data, address = sock.recvfrom(
                           self.config.receive_buffer_bytes
                       )
                   except socket.timeout:
                       self._prune_scan_assemblies(time.monotonic())
                       continue


                   peer_ip = address[0]
                   if (
                       self.config.allowed_robot_ip is not None
                       and peer_ip != self.config.allowed_robot_ip
                   ):
                       continue


                   now = time.monotonic()


                   try:
                       packet = decode_sensor_datagram(data)
                   except ValueError as exc:
                       self.last_error = f"Bad sensor datagram: {exc}"
                       continue


                   with self._lock:
                       self._last_network_packet_time = now
                       self._last_peer_ip = peer_ip
                       self._last_sequence = packet.header.sequence


                   if isinstance(packet, RangePacket):
                       self._process_range_packet(packet, now)
                   elif isinstance(packet, LidarChunkPacket):
                       self._process_lidar_chunk(packet, now)


                   self._prune_scan_assemblies(now)
                   self.last_error = None


           except OSError as exc:
               if not self._stop_event.is_set():
                   self.last_error = f"Sensor UDP socket error: {exc}"
                   time.sleep(0.5)
           finally:
               sock = self._socket
               self._socket = None
               if sock is not None:
                   try:
                       sock.close()
                   except OSError:
                       pass


   def _process_range_packet(
       self,
       packet: RangePacket,
       now: float,
   ) -> None:
       ultrasonic_raw = self._positive_meters(packet.ultrasonic_mm)
       ultrasonic_estimate = self._ultrasonic_filter.step(
           ultrasonic_raw,
           now,
           measurement_valid=ultrasonic_raw is not None,
       )


       tof_raw = self._positive_meters(packet.tof_down_mm)
       tof_measurement_valid = packet.tof_valid and tof_raw is not None
       tof_estimate = self._tof_filter.step(
           tof_raw,
           now,
           measurement_valid=tof_measurement_valid,
       )


       with self._lock:
           self._ultrasonic_raw_m = ultrasonic_raw
           self._ultrasonic_m = ultrasonic_estimate.value
           self._ultrasonic_velocity_mps = ultrasonic_estimate.velocity
           self._ultrasonic_predicted = ultrasonic_estimate.predicted_only
           if ultrasonic_estimate.accepted_measurement:
               self._ultrasonic_last_valid_time = now


           self._tof_raw_m = tof_raw
           self._tof_m = tof_estimate.value
           self._tof_velocity_mps = tof_estimate.velocity
           self._tof_valid = tof_measurement_valid
           self._tof_predicted = tof_estimate.predicted_only
           if tof_estimate.accepted_measurement:
               self._tof_last_valid_time = now


   def _process_lidar_chunk(
       self,
       packet: LidarChunkPacket,
       now: float,
   ) -> None:
       with self._lock:
           assembly = self._scan_assemblies.get(packet.scan_id)


           if (
               assembly is None
               or assembly["chunk_count"] != packet.chunk_count
           ):
               assembly = {
                   "chunk_count": packet.chunk_count,
                   "chunks": {},
                   "first_time": now,
                   "last_time": now,
               }
               self._scan_assemblies[packet.scan_id] = assembly


           assembly["chunks"][packet.chunk_index] = packet.points
           assembly["last_time"] = now


           if len(assembly["chunks"]) != assembly["chunk_count"]:
               return


           complete_points: list[LidarPoint] = []
           for chunk_index in range(assembly["chunk_count"]):
               chunk_points = assembly["chunks"].get(chunk_index)
               if chunk_points is None:
                   return
               complete_points.extend(chunk_points)


           self._latest_scan = tuple(complete_points)
           self._latest_scan_time = now
           del self._scan_assemblies[packet.scan_id]


   def _prune_scan_assemblies(self, now: float) -> None:
       with self._lock:
           stale_ids = [
               scan_id
               for scan_id, assembly in self._scan_assemblies.items()
               if now - assembly["last_time"]
               > self.config.incomplete_scan_timeout_s
           ]


           for scan_id in stale_ids:
               del self._scan_assemblies[scan_id]


   def snapshot(self, now: Optional[float] = None) -> SensorSnapshot:
       current_time = time.monotonic() if now is None else float(now)


       with self._lock:
           lidar_age = (
               float("inf")
               if self._latest_scan_time <= 0.0
               else max(0.0, current_time - self._latest_scan_time)
           )
           ultrasonic_age = (
               float("inf")
               if self._ultrasonic_last_valid_time <= 0.0
               else max(
                   0.0,
                   current_time - self._ultrasonic_last_valid_time,
               )
           )
           tof_age = (
               float("inf")
               if self._tof_last_valid_time <= 0.0
               else max(0.0, current_time - self._tof_last_valid_time)
           )
           network_age = (
               float("inf")
               if self._last_network_packet_time <= 0.0
               else max(
                   0.0,
                   current_time - self._last_network_packet_time,
               )
           )


           return SensorSnapshot(
               timestamp=current_time,
               lidar_points=self._latest_scan,
               lidar_age_s=lidar_age,
               ultrasonic_raw_m=self._ultrasonic_raw_m,
               ultrasonic_m=self._ultrasonic_m,
               ultrasonic_velocity_mps=(
                   self._ultrasonic_velocity_mps
               ),
               ultrasonic_predicted=self._ultrasonic_predicted,
               ultrasonic_age_s=ultrasonic_age,
               tof_down_raw_m=self._tof_raw_m,
               tof_down_m=self._tof_m,
               tof_down_velocity_mps=self._tof_velocity_mps,
               tof_valid=self._tof_valid,
               tof_predicted=self._tof_predicted,
               tof_age_s=tof_age,
               network_age_s=network_age,
               network_peer_ip=self._last_peer_ip,
           )


   def stop(self) -> None:
       self._stop_event.set()


       sock = self._socket
       if sock is not None:
           try:
               sock.close()
           except OSError:
               pass


       if self._thread:
           self._thread.join(timeout=2.0)




@dataclass(frozen=True)
class CommandNetworkConfig:
   robot_host: str
   command_port: int = 5006




class RobotCommandSender:
   def __init__(self, config: CommandNetworkConfig) -> None:
       self.config = config
       self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       self._sequence = 0
       self.last_error: Optional[str] = None


   def send(
       self,
       command: str,
       heading_deg: Optional[float],
       speed_scale: float,
   ) -> bool:
       packet = encode_command_packet(
           sequence=self._sequence,
           command=command,
           heading_deg=heading_deg,
           speed_scale=speed_scale,
       )
       self._sequence = (self._sequence + 1) & 0xFFFFFFFF


       try:
           self._socket.sendto(
               packet,
               (self.config.robot_host, self.config.command_port),
           )
           self.last_error = None
           return True
       except OSError as exc:
           self.last_error = f"Command UDP send error: {exc}"
           return False


   def send_decision(self, decision: object) -> bool:
       return self.send(
           command=str(getattr(decision, "command")),
           heading_deg=getattr(decision, "selected_heading_deg"),
           speed_scale=float(
               getattr(decision, "desired_speed_scale")
           ),
       )


   def send_stop(self) -> bool:
       return self.send("STOP", None, 0.0)


   def close(self) -> None:
       try:
           self._socket.close()
       except OSError:
           pass




class SensorHub:
   def __init__(
       self,
       bind_host: str = "0.0.0.0",
       sensor_port: int = 5005,
       allowed_robot_ip: Optional[str] = None,
   ) -> None:
       self.receiver = NetworkSensorReceiver(
           SensorNetworkConfig(
               bind_host=bind_host,
               sensor_port=sensor_port,
               allowed_robot_ip=allowed_robot_ip,
           )
       )


   def start(self) -> None:
       self.receiver.start()


   def snapshot(self) -> SensorSnapshot:
       return self.receiver.snapshot()


   def errors(self) -> list[str]:
       return [self.receiver.last_error] if self.receiver.last_error else []


   def stop(self) -> None:
       self.receiver.stop()
