import threading
import numpy as np
from abc import ABC, abstractmethod

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_

from ..bridge import HeightmapReceiver


class Adapter(ABC):
    def __init__(self, crc: CRC, lowcmd_pub: ChannelPublisher, lowcmd: LowCmd_) -> None:
        self._crc = crc
        self._lowcmd_pub = lowcmd_pub
        self._lowcmd = lowcmd

        self._last_q = np.empty(12)

    @abstractmethod
    def execute(self, start_motor_pos: np.ndarray, cancel_event: threading.Event) -> np.ndarray:
        pass

    def set_floatargs(self, arg1: float, arg2: float) -> "Adapter":
        pass

    def set_heightmap_receiver(self, hm_receiver: HeightmapReceiver) -> "Adapter":
        self._hm_receiver = hm_receiver
        return self
