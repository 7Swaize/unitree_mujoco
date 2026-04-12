from typing_extensions import override
from unitree_sdk2py.utils.crc import CRC

from ..adapter import Adapter


class Rotate(Adapter):
    def __init__(self, crc: CRC) -> None:
        super().__init__(crc)
        pass

    
    @override
    def execute(self) -> None:
        pass