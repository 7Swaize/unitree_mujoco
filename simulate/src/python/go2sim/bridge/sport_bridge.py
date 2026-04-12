import sys
import threading
import iceoryx2 as iox2
from typing import Dict
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from iceoryx_interfaces.mappings import SportCommand
from iceoryx_interfaces.qos import SportQoS
from iceoryx_interfaces.sport_cmds import (
    SportCommandHeader_,
    NoArgsData_,
    FloatArgsData_
)

from ..adapters import Adapter
from ..adapters.sport import (
    Stop,
    StandDown,
    StandUp,
    Move,
    Rotate
)


class SportBridge:
    def __init__(self):
        self._thread = None
        self._stop_event = threading.Event()
        self._crc = CRC()
        
        self._initialize_iox_services()
        self._initialize_cyclonedds_services()
        self._initialize_adapter_mappings()

    def _initialize_iox_services(self) -> None:
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
        self._node = iox2.NodeBuilder.new() \
                        .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
                        .create(iox2.ServiceType.Ipc)
        
        self._noargs_service = self._node.service_builder(iox2.ServiceName.new(SportQoS.TOPIC_SIM_NOARGS_CMD)) \
                                    .publish_subscribe(NoArgsData_) \
                                    .user_header(SportCommandHeader_) \
                                    .open_or_create()
        
        self._floatargs_service = self._node.service_builder(iox2.ServiceName.new(SportQoS.TOPIC_SIM_FLOATARGS_CMD)) \
                                    .publish_subscribe(FloatArgsData_) \
                                    .user_header(SportCommandHeader_) \
                                    .open_or_create()
        
        self._noargs_sub = self._noargs_service.subscriber_builder().create()
        self._floatargs_sub = self._floatargs_service.subscriber_builder().create()
        self._cycle_time = iox2.Duration.from_millis(50) # 20 Hz polling should be fine? 

    def _initialize_cyclonedds_services(self) -> None:
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
        else:
            ChannelFactoryInitialize(0, sys.argv[1])

    def _initialize_adapter_mappings(self) -> None:
        self._api_mappings: Dict[SportCommand, Adapter] = {
            SportCommand.STOP: Stop(crc=self._crc),
            SportCommand.STAND_UP: StandUp(crc=self._crc),
            SportCommand.STAND_DOWN: StandDown(crc=self._crc),
            SportCommand.MOVE: Move(crc=self._crc),
            SportCommand.ROTATE: Rotate(crc=self._crc)
        }

    def _start(self) -> None:
        self._thread = threading.Thread(target=self._iox_thread, daemon=True)
        self._thread.start()

    def _iox_thread(self):
        while not self._stop_event.is_set():
            self._node.wait(self._cycle_time)

            while True:
                sample = self._noargs_sub.receive()
                if sample is None:
                    break
                
                command = sample.user_header().contents.command
                # execute command or add to queue
                self._api_mappings[command].execute()

            while True:
                sample = self._floatargs_sub.receive()
                if sample is None:
                    break

                command = sample.user_header().contents.command
                # execute command or add to queue
                self._api_mappings[command].execute()


    def _shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join()
            self._thread = None
