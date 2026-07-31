import ctypes

class HeightmapHeader_(ctypes.Structure):
    _fields_ = [
        ("grid_extent_x", ctypes.c_uint32),
        ("grid_extent_y", ctypes.c_uint32)
    ]

    @staticmethod
    def type_name() -> str:
        return "HeightmapHeader_"