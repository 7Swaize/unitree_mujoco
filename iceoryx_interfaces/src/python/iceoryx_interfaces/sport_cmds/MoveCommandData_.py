import ctypes


class MoveCommandData_(ctypes.Structure):
    _fields_ = [
        ("vx", ctypes.c_float),
        ("vy", ctypes.c_float)
    ]

    @staticmethod
    def type_name() -> str:
        return "MoveCommandData_"