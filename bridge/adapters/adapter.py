from abc import ABC, abstractmethod
from typing import Callable, List

from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC


class Adapter(ABC):
    def __init__(self, crc: CRC) -> None:
        self._crc = crc
        self._on_start: List[Callable[["Adapter"], None]] = []
        self._on_end: List[Callable[["Adapter"], None]] = []

    @abstractmethod
    def execute(self) -> None:
        pass

    def add_on_start(self, callback: Callable[["Adapter"], None]) -> None:
        self._on_start.append(callback)

    def add_on_end(self, callback: Callable[["Adapter"], None]) -> None:
        self._on_end.append(callback)

    def _notify_start(self) -> None:
        for cb in self._on_start:
            cb(self)

    def _notify_end(self) -> None:
        for cb in self._on_end:
            cb(self)

    def run(self) -> None:
        self._notify_start()
        try:
            self.execute()
        finally:
            self._notify_end()


    def _zero_lowcmd_msg(self, cmd: LowCmd_) -> None:
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0
        
        for i in range(20):
            cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 0.0