import numpy as np
from abc import ABC, abstractmethod
from typing import Callable, List

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_




class Adapter(ABC):
    def __init__(self, crc: CRC, lowcmd_pub: ChannelPublisher, lowcmd: LowCmd_) -> None:
        self._crc = crc
        self._lowcmd_pub = lowcmd_pub
        self._lowcmd = lowcmd

        self._last_q = np.empty(12)

    @abstractmethod
    def execute(self, start_motor_pos: np.ndarray) -> np.ndarray:
        pass

    def set_floatargs(self, arg1: float, arg2: float) -> "Adapter":
        pass
