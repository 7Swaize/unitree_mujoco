from enum import IntEnum, auto


class SportCommand(IntEnum):
    STOP = auto()
    STAND_UP = auto()
    STAND_DOWN = auto()
    MOVE = auto()
    ROTATE = auto()