import ctypes

class CommandResponse_(ctypes.Structure):
    _fields_ = [
        ("status", ctypes.c_uint32)
    ]

    @staticmethod
    def type_name() -> str:
        return "CommandResponse_"