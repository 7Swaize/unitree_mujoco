from dataclasses import dataclass

@dataclass(frozen=True)
class HeightmapQoS:
    TOPIC_HEIGHTMAP = "sim/heightmap"