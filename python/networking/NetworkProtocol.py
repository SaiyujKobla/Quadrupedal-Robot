from __future__ import annotations


import struct
import time
from dataclasses import dataclass
from typing import Optional


from sensors.SensorModels import LidarPoint




PROTOCOL_VERSION = 1


SENSOR_MAGIC = b"QR"
COMMAND_MAGIC = b"QC"


PACKET_RANGE = 1
PACKET_LIDAR_CHUNK = 2
PACKET_COMMAND = 1


RANGE_FLAG_ULTRASONIC_VALID = 1 << 0
RANGE_FLAG_TOF_VALID = 1 << 1


COMMAND_CODES = {
   "STOP": 0,
   "FORWARD": 1,
   "TURN LEFT": 2,
   "TURN RIGHT": 3,
}


HEADER_STRUCT = struct.Struct("<2sBBII")
RANGE_PAYLOAD_STRUCT = struct.Struct("<ffB3x")
LIDAR_CHUNK_META_STRUCT = struct.Struct("<IHHH2x")
LIDAR_POINT_STRUCT = struct.Struct("<HHBB")
COMMAND_STRUCT = struct.Struct("<2sBBIIB3xff")


MAX_LIDAR_POINTS_PER_CHUNK = 200




@dataclass(frozen=True)
class PacketHeader:
   packet_type: int
   sequence: int
   sender_millis: int




@dataclass(frozen=True)
class RangePacket:
   header: PacketHeader
   ultrasonic_mm: Optional[float]
   tof_down_mm: Optional[float]
   tof_valid: bool




@dataclass(frozen=True)
class LidarChunkPacket:
   header: PacketHeader
   scan_id: int
   chunk_index: int
   chunk_count: int
   points: tuple[LidarPoint, ...]




def _decode_header(data: bytes) -> tuple[PacketHeader, int]:
   if len(data) < HEADER_STRUCT.size:
       raise ValueError("Datagram is shorter than the sensor header")


   magic, version, packet_type, sequence, sender_millis = (
       HEADER_STRUCT.unpack_from(data, 0)
   )


   if magic != SENSOR_MAGIC:
       raise ValueError("Sensor packet magic is invalid")
   if version != PROTOCOL_VERSION:
       raise ValueError(
           f"Unsupported sensor protocol version {version}; "
           f"expected {PROTOCOL_VERSION}"
       )


   return (
       PacketHeader(
           packet_type=int(packet_type),
           sequence=int(sequence),
           sender_millis=int(sender_millis),
       ),
       HEADER_STRUCT.size,
   )




def decode_sensor_datagram(
   data: bytes,
) -> RangePacket | LidarChunkPacket:
   header, offset = _decode_header(data)


   if header.packet_type == PACKET_RANGE:
       expected = offset + RANGE_PAYLOAD_STRUCT.size
       if len(data) != expected:
           raise ValueError("Range packet has an unexpected length")


       ultrasonic_mm, tof_down_mm, flags = (
           RANGE_PAYLOAD_STRUCT.unpack_from(data, offset)
       )


       ultrasonic_valid = bool(
           int(flags) & RANGE_FLAG_ULTRASONIC_VALID
       )
       tof_valid = bool(int(flags) & RANGE_FLAG_TOF_VALID)


       return RangePacket(
           header=header,
           ultrasonic_mm=(
               float(ultrasonic_mm) if ultrasonic_valid else None
           ),
           tof_down_mm=(float(tof_down_mm) if tof_valid else None),
           tof_valid=tof_valid,
       )


   if header.packet_type == PACKET_LIDAR_CHUNK:
       meta_end = offset + LIDAR_CHUNK_META_STRUCT.size
       if len(data) < meta_end:
           raise ValueError("LiDAR packet is shorter than its metadata")


       scan_id, chunk_index, chunk_count, point_count = (
           LIDAR_CHUNK_META_STRUCT.unpack_from(data, offset)
       )


       if chunk_count <= 0:
           raise ValueError("LiDAR chunk_count must be positive")
       if chunk_index >= chunk_count:
           raise ValueError("LiDAR chunk_index is outside chunk_count")
       if point_count > MAX_LIDAR_POINTS_PER_CHUNK:
           raise ValueError("LiDAR chunk contains too many points")


       expected = meta_end + point_count * LIDAR_POINT_STRUCT.size
       if len(data) != expected:
           raise ValueError("LiDAR packet length does not match point_count")


       points: list[LidarPoint] = []
       point_offset = meta_end


       for _ in range(point_count):
           angle_cdeg, distance_mm, quality, _reserved = (
               LIDAR_POINT_STRUCT.unpack_from(data, point_offset)
           )
           point_offset += LIDAR_POINT_STRUCT.size


           points.append(
               LidarPoint(
                   angle_deg=float(angle_cdeg) / 100.0,
                   distance_m=float(distance_mm) / 1000.0,
                   quality=int(quality),
               )
           )


       return LidarChunkPacket(
           header=header,
           scan_id=int(scan_id),
           chunk_index=int(chunk_index),
           chunk_count=int(chunk_count),
           points=tuple(points),
       )


   raise ValueError(f"Unknown sensor packet type {header.packet_type}")




def monotonic_millis_u32() -> int:
   return int(time.monotonic() * 1000.0) & 0xFFFFFFFF




def encode_command_packet(
   sequence: int,
   command: str,
   heading_deg: Optional[float],
   speed_scale: float,
) -> bytes:
   normalized_command = str(command).upper()
   if normalized_command not in COMMAND_CODES:
       raise ValueError(f"Unsupported command: {command}")


   heading = 0.0 if heading_deg is None else float(heading_deg)
   speed = max(0.0, min(1.0, float(speed_scale)))


   return COMMAND_STRUCT.pack(
       COMMAND_MAGIC,
       PROTOCOL_VERSION,
       PACKET_COMMAND,
       int(sequence) & 0xFFFFFFFF,
       monotonic_millis_u32(),
       COMMAND_CODES[normalized_command],
       heading,
       speed,
   )
