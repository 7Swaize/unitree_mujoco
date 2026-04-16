import numpy as np
from typing_extensions import override

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from ..adapter import Adapter


class Move(Adapter):
    def __init__(self, crc: CRC, lowcmd_pub: ChannelPublisher, lowcmd: LowCmd_) -> None:
        super().__init__(crc, lowcmd_pub, lowcmd)

    
    @override
    def execute(self, start_pos: np.ndarray) -> np.ndarray:
        pass