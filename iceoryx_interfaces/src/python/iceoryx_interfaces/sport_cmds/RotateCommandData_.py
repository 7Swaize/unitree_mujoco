import ctypes 


class RotateCommandData_(ctypes.Structure):
    _fields_ = [
        ("vrot", ctypes.c_float)
    ]

    @staticmethod
    def type_name(self) -> str:
        return "RotateCommandData_"