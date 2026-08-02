from enum import IntEnum, auto


class SportCommand(IntEnum):
    STAND_UP = auto()
    STAND_DOWN = auto()
    MOVE = auto()
    STOP_MOVE = auto()
    DAMP = auto()
    BALANCE_STAND = auto()


class CommandKind(IntEnum):
    NO_ARGS = auto()
    FLOAT_ARGS = auto()


class CommandStatus(IntEnum):
    OK = auto()
    INTERRUPTED = auto()
    SUPERSEDED = auto()
    CANCELLED = auto()
