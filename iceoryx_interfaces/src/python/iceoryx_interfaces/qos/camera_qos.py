from dataclasses import dataclass


@dataclass(frozen=True)
class CameraQoS:
    TOPIC_SIM_CAMERA = "sim/cam"