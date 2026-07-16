import ctypes
from dataclasses import dataclass

@dataclass(frozen=True)
class LidarQoS:
	TOPIC_LIDAR_DECODED = "control/lidar_decoded"

	INITIAL_SLICE_LEN_HINT = 20000 * ctypes.sizeof(ctypes.c_double)

	MAX_PUBLISHERS = 1
	MAX_SUBSCRIBERS = 1
	SUBSCRIBER_MAX_BUFFER_SIZE = 3
	SUBSCRIBER_MAX_BORROWED_SAMPLES = 2
	HISTORY_SIZE = 1