import sys
import threading
import iceoryx2 as iox2
from typing import Optional

from unitree_sdk2py.utils.crc import CRC, LowCmd_
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from iceoryx_interfaces.mappings import CommandKind
from iceoryx_interfaces.qos import SportQoS
from iceoryx_interfaces.sport_cmds import (
    SportCommandHeader_,
    NoArgsData_,
    FloatArgsData_,
    CommandResponse_
)

from ..fsm import StateMachine

DDS_LOW_CMD_TOPIC = "rt/lowcmd"


class SportBridge:
    def __init__(self):
        self._iox_cmd_thread_ref: Optional[threading.Thread] = None
        self._shutdown_event: threading.Event = threading.Event()

        self._crc = CRC()
        
        self._init_iox_services()
        self._init_dds_services()

        self._state_machine = StateMachine(self._crc, self._lowcmd, self._lowcmd_pub)


    def _init_iox_services(self) -> None:
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
        self._node = iox2.NodeBuilder.new() \
                        .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
                        .create(iox2.ServiceType.Ipc)
        
        self._noargs_service = self._node.service_builder(iox2.ServiceName.new(SportQoS.TOPIC_SIM_NOARGS_CMD)) \
                                    .request_response(NoArgsData_, CommandResponse_) \
                                    .request_header(SportCommandHeader_) \
                                    .open_or_create()
        
        self._floatargs_service = self._node.service_builder(iox2.ServiceName.new(SportQoS.TOPIC_SIM_FLOATARGS_CMD)) \
                                    .request_response(FloatArgsData_, CommandResponse_) \
                                    .request_header(SportCommandHeader_) \
                                    .open_or_create()
        
        self._noargs_server = self._noargs_service.server_builder().create()
        self._floatargs_server = self._floatargs_service.server_builder().create()
        self._cmd_cycle_time = iox2.Duration.from_millis(50) # 20 Hz polling should be fine?


    def _init_dds_services(self) -> None:
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
        else:
            ChannelFactoryInitialize(0, sys.argv[1])

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
        self._state_machine.start()

        self._iox_cmd_thread_ref = threading.Thread(target=self._iox_thread, daemon=True)
        self._iox_cmd_thread_ref.start()


    def _iox_thread(self):
        while not self._shutdown_event.is_set():
            self._node.wait(self._cmd_cycle_time)

            while True:
                active_request = self._noargs_server.receive()
                if active_request is None:
                    break
                
                command = int(active_request.user_header().contents.command)

                self._state_machine.receive_command((CommandKind.NO_ARGS, command, [], active_request))

            while True:
                active_request = self._floatargs_server.receive()
                if active_request is None:
                    break
                
                data = active_request.payload().contents
                command = int(active_request.user_header().contents.command)

                self._state_machine.receive_command((CommandKind.FLOAT_ARGS, command, [data.arg1, data.arg2, data.arg3], active_request))


    def shutdown(self) -> None:
        self._shutdown_event.set()

        if self._iox_cmd_thread_ref:
            self._iox_cmd_thread_ref.join()
            self._iox_cmd_thread_ref = None

        self._state_machine.shutdown()