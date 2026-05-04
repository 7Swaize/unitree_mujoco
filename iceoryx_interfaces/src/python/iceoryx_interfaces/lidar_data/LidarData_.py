import ctypes


class LidarData_(ctypes.Structure):
	_fields_ = [
		("shape", ctypes.c_char_p),
		("stamp_ns", ctypes.c_uint64),
	]

	@staticmethod
	def type_name() -> str:
		return "LidarData_"