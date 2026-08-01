import ctypes

class HeightmapHeader_(ctypes.Structure):
    _fields_ = [
        ("num_heightscans", ctypes.c_uint32),
        ("num_widthscans", ctypes.c_uint32),
        ("dist_x", ctypes.c_float),
        ("dist_y", ctypes.c_float),
        ("base_x", ctypes.c_float),
        ("base_y", ctypes.c_float),
        ("yaw", ctypes.c_float)
    ]

    @staticmethod
    def type_name() -> str:
        return "HeightmapHeader_"