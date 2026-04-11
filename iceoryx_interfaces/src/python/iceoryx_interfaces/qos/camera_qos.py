from dataclasses import dataclass


@dataclass
class CameraQoS:
    TOPIC_SIM_CAMERA_DEPTH = "rt/sim/depth"
    TOPIC_SIM_CAMERA_RGB = "rt/sim/rgb"
    
    MAX_PUBLISHERS = 1
    MAX_SUBSCRIBERS = 1
    SUBSCRIBER_MAX_BUFFER_SIZE = 3 # we really only care about the most recent frames
    SUBSCRIBER_MAX_BORROWED_SAMPLES = 2
    HISTORY_SIZE = 1