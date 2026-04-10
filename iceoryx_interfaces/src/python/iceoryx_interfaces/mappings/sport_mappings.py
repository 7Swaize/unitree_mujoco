from enum import IntEnum


class SportCommand(IntEnum):
    STOP = 0
    STAND_UP = 1
    STAND_DOWN = 2
    MOVE = 3
    ROTATE = 4


# Header layout:
# bits 0–3 : command (0–15)
# bits 4–31 : reserved (future flags)

CMD_BITS = 4
CMD_MASK = (1 << CMD_BITS) - 1


def encode_header(cmd: int) -> int:
    return cmd & CMD_MASK

def decode_command(cmd: int) -> SportCommand:
    return SportCommand(cmd & CMD_MASK)