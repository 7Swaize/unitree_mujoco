import ctypes
from .constants import FRAME_BUFFER_ELEMENTS_DEPTH, FRAME_BUFFER_ELEMENTS_RGB


class FrameData_(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("depth_min", ctypes.c_float),
        ("depth_max", ctypes.c_float),
        ("rgb_data", ctypes.c_uint8 * FRAME_BUFFER_ELEMENTS_RGB),
        ("depth_data", ctypes.c_uint16 * FRAME_BUFFER_ELEMENTS_DEPTH)
    ]

    @staticmethod
    def type_name() -> str:
        return "FrameData_"