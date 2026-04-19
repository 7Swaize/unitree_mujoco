import ctypes
from .constants import FRAME_BUFFER_ELEMENTS_DEPTH


class DepthFrameData_(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("depth_min", ctypes.c_float),
        ("depth_max", ctypes.c_float),
        ("data", ctypes.c_uint16 * FRAME_BUFFER_ELEMENTS_DEPTH)
    ]

    @staticmethod
    def type_name() -> str:
        return "DepthFrameData_"