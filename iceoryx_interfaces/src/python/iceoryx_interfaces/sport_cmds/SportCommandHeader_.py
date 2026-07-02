import ctypes


class SportCommandHeader_(ctypes.Structure):
    _fields_ = [
        ("command", ctypes.c_uint32),
        ("track", ctypes.c_bool)
    ]

    @staticmethod
    def type_name() -> str:
        return "SportCommandHeader_"