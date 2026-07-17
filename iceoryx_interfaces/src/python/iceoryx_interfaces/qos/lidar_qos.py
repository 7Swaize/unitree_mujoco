import ctypes
from dataclasses import dataclass

@dataclass(frozen=True)
class LidarQoS:
	TOPIC_LIDAR_DECODED = "control/lidar_decoded"

	RESPONSE_INITIAL_SLICE_LEN_HINT = 20000 * ctypes.sizeof(ctypes.c_double)