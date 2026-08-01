import sys
import queue
import threading
import numpy as np
import iceoryx2 as iox2
from typing import Dict, Any, Optional

from unitree_sdk2py.utils.crc import CRC, LowCmd_
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from iceoryx_interfaces.mappings import SportCommand, CommandKind, CommandStatus
from iceoryx_interfaces.qos import SportQoS
from iceoryx_interfaces.sport_cmds import (
    SportCommandHeader_,
    NoArgsData_,
    FloatArgsData_,
    CommandResponse_
)

from ..nn import PolicyController
from ..adapters.sport.constants import DDS_LOW_CMD_TOPIC, STAND_DOWN_JOINT_POS
from ..adapters import Adapter
from ..adapters.sport import (
    StandDown,
    StandUp,
    StopMove
)


class SportBridge:
    def __init__(self):
        self._iox_cmd_thread_ref: Optional[threading.Thread] = None
        self._command_thread_ref: Optional[threading.Thread] = None
        self._shutdown_event: threading.Event = threading.Event()
        self._adapter_stop_event: threading.Event = threading.Event()

        self._policy_controller: PolicyController = PolicyController()
        self._command_queue: queue.Queue[tuple[CommandKind, SportCommand, list[Any], Optional[iox2.ActiveRequest]]] = queue.Queue(maxsize=3)
        self._crc = CRC()
        
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


    def _init_cyclonedds_services(self) -> None:
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
        else:
            ChannelFactoryInitialize(0, sys.argv[1])


    def _init_adapter_mappings(self) -> None:
        common = dict(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd, policy_controller=self._policy_controller)

        self._api_mappings: Dict[SportCommand, Adapter] = {
            SportCommand.STAND_UP: StandUp(**common),
            SportCommand.STAND_DOWN: StandDown(**common),
            SportCommand.STOP_MOVE: StopMove(**common),
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
        self._iox_cmd_thread_ref = threading.Thread(target=self._iox_thread, daemon=True)
        self._iox_cmd_thread_ref.start()

        self._command_thread_ref = threading.Thread(target=self._command_thread, daemon=True)
        self._command_thread_ref.start()


    def _iox_thread(self):
        while not self._shutdown_event.is_set():
            self._node.wait(self._cmd_cycle_time)

            while True:
                active_request = self._noargs_server.receive()
                if active_request is None:
                    break
                
                command = active_request.user_header().contents.command
                track = active_request.user_header().contents.track

                if not track:
                    active_request.delete()
                    active_request = None

                if command == SportCommand.STOP:
                    self._request_stop(active_request)
                    break

                self._enqueue((CommandKind.NO_ARGS, command, [], active_request))

            while True:
                active_request = self._floatargs_server.receive()
                if active_request is None:
                    break
                
                data = active_request.payload().contents
                command = active_request.user_header().contents.command
                track = active_request.user_header().contents.track

                if not track:
                    active_request.delete()
                    active_request = None
                    
                self._enqueue((CommandKind.FLOAT_ARGS, command, [data.arg1, data.arg2, data.arg3], active_request))


    def _enqueue(self, item: tuple[CommandKind, SportCommand, list[Any], Optional[iox2.ActiveRequest]]) -> None:
        try:
            self._command_queue.put_nowait(item)
        except queue.Full:
            evicted = self._command_queue.get_nowait()
            self._respond(evicted[3], CommandStatus.SUPERSEDED)
            self._command_queue.put_nowait(item)


    def _command_thread(self):
        while not self._shutdown_event.is_set():
            try:
                kind, command, args, active_request = self._command_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self._adapter_stop_event.is_set():
                self._adapter_stop_event.clear()

            if kind == CommandKind.NO_ARGS:
                self._handle_noargs_cmd(command, active_request)
            elif kind == CommandKind.FLOAT_ARGS:
                self._handle_floatargs_cmd(command, active_request, *map(float, args))
        

    def _handle_noargs_cmd(self, command: SportCommand, active_request: Optional[iox2.ActiveRequest]) -> None:
        self._api_mappings[command].execute(self._adapter_stop_event)
        status = CommandStatus.INTERRUPTED if self._adapter_stop_event.is_set() else CommandStatus.OK
        self._respond(active_request, status)


    def _handle_floatargs_cmd(
        self,
        command: SportCommand,
        active_request: Optional[iox2.ActiveRequest],
        arg1: float,
        arg2: float,
        arg3: float
    ) -> None:
        self._api_mappings[command] \
            .set_floatargs(arg1, arg2, arg3) \
            .execute(self._adapter_stop_event)
        
        status = CommandStatus.INTERRUPTED if self._adapter_stop_event.is_set() else CommandStatus.OK
        self._respond(active_request, status)


    def _respond(self, active_request: Optional[iox2.ActiveRequest], status: CommandStatus) -> None:
        if active_request is None or not active_request.is_connected:
            return
        
        response = active_request.loan_uninit()
        response = response.write_payload(CommandResponse_(status=status))
        response.send()
        active_request.delete()


    def _request_stop(self, active_request: Optional[iox2.ActiveRequest]) -> None:
        while True:
            try:
                item = self._command_queue.get_nowait()
                self._respond(item[3], CommandStatus.CANCELLED)
            except queue.Empty:
                break

        self._adapter_stop_event.set()
        self._enqueue((CommandKind.NO_ARGS, SportCommand.STOP, [], active_request))


    def shutdown(self) -> None:
        self._shutdown_event.set()

        if self._iox_cmd_thread_ref:
            self._iox_cmd_thread_ref.join()
            self._iox_cmd_thread_ref = None

        if self._command_thread_ref:
            self._command_thread_ref.join()
            self._command_thread_ref = None

        self._policy_controller.shutdown()

        while True:
            try:
                self._adapter_stop_event.set()
                item = self._command_queue.get_nowait()
                self._respond(item[3], CommandStatus.CANCELLED)
            except queue.Empty:
                break