import threading
from abc import ABC, abstractmethod

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_

from ...nn import PolicyController


class State(ABC):
    def __init__(
        self,
        crc: CRC,
        lowcmd_pub: ChannelPublisher,
        lowcmd: LowCmd_,
        policy_controller: PolicyController
    ) -> None:
        self._crc = crc
        self._lowcmd_pub = lowcmd_pub
        self._lowcmd = lowcmd
        self._policy_controller = policy_controller

    @abstractmethod
    def execute(self, cancel_event: threading.Event) -> None:
        pass

    def set_floatargs(self, arg1: float, arg2: float, arg3: float) -> "State":
        pass