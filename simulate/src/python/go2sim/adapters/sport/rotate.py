import numpy as np
from typing_extensions import override

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from ..adapter import Adapter


class Rotate(Adapter):
    @override
    def set_floatargs(self, arg1: float, arg2: float) -> Adapter:
        self._vrot = arg1
        return self

    
    @override
    def execute(self, start_pos: np.ndarray) -> np.ndarray:
        pass