from dataclasses import dataclass


@dataclass
class SportQoS:
    TOPIC_SIM_NOARGS_CMD = "sim/sport/noargs_cmd"
    TOPIC_SIM_FLOATARGS_CMD = "sim/sport/floatargs_cmd"