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

        self._on_start: List[Callable[["Adapter"], None]] = []
        self._on_end: List[Callable[["Adapter"], None]] = []

    @abstractmethod
    def execute(self, start_motor_pos: np.ndarray) -> np.ndarray:
        pass

    def add_on_start(self, callback: Callable[["Adapter"], None]) -> None:
        self._on_start.append(callback)

    def add_on_end(self, callback: Callable[["Adapter"], None]) -> None:
        self._on_end.append(callback)

    def _notify_start(self) -> None:
        for cb in self._on_start:
            cb(self)

    def _notify_end(self) -> np.ndarray:
        for cb in self._on_end:
            cb(self)

    def run(self) -> None:
        self._notify_start()
        try:
            self.execute()
        finally:
            self._notify_end()