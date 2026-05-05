import ctypes


class LidarHeader_(ctypes.Structure):
	_fields_ = [
		("rows", ctypes.c_int),
		("cols", ctypes.c_int),
		("stamp_ns", ctypes.c_uint64),
	]

	@staticmethod
	def type_name() -> str:
		return "LidarHeader_"