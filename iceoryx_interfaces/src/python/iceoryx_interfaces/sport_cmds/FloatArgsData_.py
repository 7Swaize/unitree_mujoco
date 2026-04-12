import ctypes


class FloatArgsData_(ctypes.Structure):
    _fields_ = [
        ("arg1", ctypes.c_float),
        ("arg2", ctypes.c_float)
    ]

    @staticmethod
    def type_name() -> str:
        return "FloatArgsData_"