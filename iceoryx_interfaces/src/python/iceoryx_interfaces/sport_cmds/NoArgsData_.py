import ctypes 


class NoArgsData_(ctypes.Structure):
    _fields_ = [
        ("null", ctypes.c_uint8)
    ]

    @staticmethod
    def type_name() -> str:
        return "NoArgsData_"