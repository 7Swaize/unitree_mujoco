from dataclasses import dataclass

@dataclass(frozen=True)
class LidarQoS:
	TOPIC_ROS_LIDAR_DECODED = "control/lidar_decoded"
	TOPIC_ROS_LIDAR_FILTERED = "control/lidar_filtered"

	MAX_PUBLISHERS = 1
	MAX_SUBSCRIBERS = 1
	SUBSCRIBER_MAX_BUFFER_SIZE = 3
	SUBSCRIBER_MAX_BORROWED_SAMPLES = 2
	HISTORY_SIZE = 1