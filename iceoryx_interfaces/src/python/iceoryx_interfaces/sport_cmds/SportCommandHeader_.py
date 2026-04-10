import ctypes


class SportCommandHeader_(ctypes.Structure):
    _fields_ = [
        ("command", ctypes.c_uint32)
    ]

    @staticmethod
    def type_name() -> str:
        return "SportCommandHeader_"