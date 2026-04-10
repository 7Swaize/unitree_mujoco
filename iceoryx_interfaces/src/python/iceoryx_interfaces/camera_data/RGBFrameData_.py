import ctypes
from .constants import FRAME_BUFFER_ELEMENTS_RGB


class RGBFrameData_(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("data", ctypes.c_uint8 * FRAME_BUFFER_ELEMENTS_RGB)
    ]

    @staticmethod
    def type_name() -> str:
        return "RGBFrameData_"