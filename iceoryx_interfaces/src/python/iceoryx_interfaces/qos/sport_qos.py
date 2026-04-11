from dataclasses import dataclass


@dataclass
class SportQoS:
    TOPIC_SIM_NOARGS_CMD = "sim/sport/noargs_cmd"
    TOPIC_SIM_MOVE_CMD = "sim/sport/move_cmd"
    TOPIC_SIM_ROTATE_CMD = "sim/sport/rotate_cmd"