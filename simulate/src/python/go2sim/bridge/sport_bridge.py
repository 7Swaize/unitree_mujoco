import sys
import threading
import numpy as np
import iceoryx2 as iox2
from typing import Dict
from unitree_sdk2py.utils.crc import CRC, LowCmd_
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from iceoryx_interfaces.mappings import SportCommand
from iceoryx_interfaces.qos import SportQoS
from iceoryx_interfaces.sport_cmds import (
    SportCommandHeader_,
    NoArgsData_,
    FloatArgsData_
)

from ..adapters.sport.constants import DDS_LOW_CMD_TOPIC, STAND_DOWN_JOINT_POS
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

        self._last_q: np.ndarray = STAND_DOWN_JOINT_POS
        
        self._init_iox_services()
        self._init_cyclonedds_services()
        self._init_publishers()
        self._init_adapter_mappings()

    def _init_iox_services(self) -> None:
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

    def _init_cyclonedds_services(self) -> None:
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
        else:
            ChannelFactoryInitialize(0, sys.argv[1])

    def _init_adapter_mappings(self) -> None:
        self._api_mappings: Dict[SportCommand, Adapter] = {
            SportCommand.STOP: Stop(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd),
            SportCommand.STAND_UP: StandUp(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd),
            SportCommand.STAND_DOWN: StandDown(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd),
            SportCommand.MOVE: Move(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd),
            SportCommand.ROTATE: Rotate(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd)
        }

    def _init_publishers(self) -> None:
        self._lowcmd_pub = ChannelPublisher(DDS_LOW_CMD_TOPIC, LowCmd_)
        self._lowcmd_pub.Init()
        self._lowcmd = unitree_go_msg_dds__LowCmd_()

        self._lowcmd.head[0] = 0xFE
        self._lowcmd.head[1] = 0xEF
        self._lowcmd.level_flag = 0xFF
        self._lowcmd.gpio = 0
        
        for i in range(20):
            self._lowcmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self._lowcmd.motor_cmd[i].q = 0.0
            self._lowcmd.motor_cmd[i].kp = 0.0
            self._lowcmd.motor_cmd[i].dq = 0.0
            self._lowcmd.motor_cmd[i].kd = 0.0
            self._lowcmd.motor_cmd[i].tau = 0.0


    def start(self) -> None:
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
                self._handle_noargs_cmd(command)

            while True:
                sample = self._floatargs_sub.receive()
                if sample is None:
                    break
                
                data = sample.payload().contents
                command = sample.user_header().contents.command
                self._handle_floatargs_cmd(command, data.arg1, data.arg2)
    

    def _handle_noargs_cmd(self, command) -> None:
        self._last_q = self._api_mappings[command].execute(self._last_q)

    def _handle_floatargs_cmd(self, command, arg1: float, arg2: float) -> None:
        self._last_q = self._api_mappings[command].set_floatargs(arg1, arg2).execute(self._last_q)
        

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join()
            self._thread = None
