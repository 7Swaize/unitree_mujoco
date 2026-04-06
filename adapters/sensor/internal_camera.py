import numpy as np
from typing import override

from unitree_sdk2py.core.channel import ChannelSubscriber

from ..abstracts.sensor_adapter import SensorAdapter
from .idl.msg._ImageData_ import ImageData_
from ..constants import DDS_SIM_CAMERA_TOPIC


class InternalCamera(SensorAdapter):
    def __init__(self) -> None:
        super().__init__()

        self._sub = ChannelSubscriber(DDS_SIM_CAMERA_TOPIC, ImageData_)
        self._sub.Init(self.execute, 10)

    
    @override
    def execute() -> None:
        pass
