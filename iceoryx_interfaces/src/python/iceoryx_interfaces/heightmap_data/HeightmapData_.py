import ctypes
from .constants import DATA_BUFFER_ELEMENTS

class HeightmapData_(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_float * DATA_BUFFER_ELEMENTS)
    ]

    @staticmethod
    def type_name() -> str:
        return "HeightmapData_"