import threading
from enum import Enum, Flag, auto
from typing import Dict, Any, Optional

import iceoryx2 as iox2
from unitree_sdk2py.utils.crc import CRC, LowCmd_
from unitree_sdk2py.core.channel import ChannelPublisher
from iceoryx_interfaces.sport_cmds import CommandResponse_
from iceoryx_interfaces.mappings import (
    SportCommand,
    CommandKind,
    CommandStatus
)

from ..nn import PolicyController
from .typing import _CommandItem
from .priority_queue import CommandPriorityQueue
from .states import State
from .states import (
    Damp,
    BalanceStand,
    Move,
    StandDown,
    StandUp,
    StopMove
)

class CommandPreemption(Enum):
    REFRESHABLE = auto()
    LOCKED = auto()
    OVERRIDE = auto()


class LocomotionState(Flag):
    SITTING = auto()
    STANDING = auto()


class StateMachine:
    def __init__(self, crc: CRC, low_cmd: LowCmd_, lowcmd_pub: ChannelPublisher) -> None:
        self._crc: CRC = crc
        self._lowcmd: LowCmd_ = low_cmd
        self._lowcmd_pub: ChannelPublisher = lowcmd_pub
        self._policy_controller: PolicyController = PolicyController(crc, low_cmd, lowcmd_pub)

        self._command_recv_thread_ref: Optional[threading.Thread] = None
        self._current_worker_thread_ref: Optional[threading.Thread] = None
        self._shutdown_event: threading.Event = threading.Event()
        self._state_transition_event: threading.Event = threading.Event()

        self._init_states()

        self._lock: threading.Lock = threading.Lock()
        self._current_command: Optional[SportCommand] = None
        self._current_locomotion_state: LocomotionState = LocomotionState.SITTING
        self._command_queue: CommandPriorityQueue = CommandPriorityQueue(
            maxsize=3,
            priority={
                cmd: self._preemption_prority[pre] for cmd, pre in self._state_preemptions.items()
            }
        )


    def _init_states(self) -> None:
        common = dict(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd, policy_controller=self._policy_controller)

        self._states: Dict[SportCommand, State] = {
            SportCommand.BALANCE_STAND: BalanceStand(**common),
            SportCommand.DAMP: Damp(**common),
            SportCommand.MOVE: Move(**common),
            SportCommand.STAND_DOWN: StandDown(**common),
            SportCommand.STAND_UP: StandUp(**common),
            SportCommand.STOP_MOVE: StopMove(**common)
        }

        self._state_preemptions: Dict[SportCommand, CommandPreemption] = {
            SportCommand.BALANCE_STAND: CommandPreemption.LOCKED,
            SportCommand.DAMP: CommandPreemption.OVERRIDE,
            SportCommand.MOVE: CommandPreemption.REFRESHABLE,
            SportCommand.STAND_DOWN: CommandPreemption.LOCKED,
            SportCommand.STAND_UP: CommandPreemption.LOCKED,
            SportCommand.STOP_MOVE: CommandPreemption.LOCKED
        }

        self._preemption_prority: Dict[SportCommand, int] = {
            CommandPreemption.REFRESHABLE: 0,
            CommandPreemption.LOCKED: 1,
            CommandPreemption.OVERRIDE: 2
        }

        self._allowed_state_executions_during_locomotion: Dict[SportCommand, LocomotionState] = {
            SportCommand.BALANCE_STAND: LocomotionState.STANDING,
            SportCommand.DAMP: LocomotionState.STANDING | LocomotionState.SITTING,
            SportCommand.MOVE: LocomotionState.STANDING,
            SportCommand.STAND_DOWN: LocomotionState.STANDING,
            SportCommand.STAND_UP: LocomotionState.SITTING,
            SportCommand.STOP_MOVE: LocomotionState.STANDING
        }

        self._command_complete_locomotion_state_target: Dict[SportCommand, LocomotionState] = {
            SportCommand.BALANCE_STAND: LocomotionState.STANDING,
            SportCommand.DAMP: LocomotionState.SITTING,
            SportCommand.MOVE: LocomotionState.STANDING,
            SportCommand.STAND_DOWN: LocomotionState.SITTING,
            SportCommand.STAND_UP: LocomotionState.STANDING,
            SportCommand.STOP_MOVE: LocomotionState.STANDING
        }


    def start(self) -> None:
        self._command_recv_thread_ref = threading.Thread(target=self._command_recv_thread, daemon=True)
        self._command_recv_thread_ref.start()


    def receive_command(self, item: _CommandItem) -> None:
        self._command_queue.put(item, lambda ar: self._respond(ar, CommandStatus.REJECTED))


    def _command_recv_thread(self) -> None:
        while not self._shutdown_event.is_set():
            item = self._command_queue.get(timeout=0.1)
            if item is None:
                continue

            self._dispatch(item)


    def _is_allowed(self, command: SportCommand) -> bool:
        return bool(self._current_locomotion_state & self._allowed_state_executions_during_locomotion[command])


    def _is_refreshable(self, command: SportCommand) -> bool:
        return self._state_preemptions[command] == CommandPreemption.REFRESHABLE


    def _can_preempt(self, current: SportCommand, incoming: SportCommand) -> bool:
        return (
            self._state_preemptions[current] is CommandPreemption.REFRESHABLE
            or self._state_preemptions[incoming] is CommandPreemption.OVERRIDE
        )


    def _dispatch(self, item: _CommandItem) -> None:
        kind, command, args, active_request = item

        if not self._is_allowed(command):
            self._respond(active_request, CommandStatus.REJECTED)
            return

        worker_to_join: Optional[threading.Thread] = None
        with self._lock:
            current = self._current_command

            if current is not None and current == command and self._is_refreshable(command):
                self._refresh_current(kind, command, args, active_request)
                return

            if current is not None and not self._can_preempt(current, command):
                self._respond(active_request, CommandStatus.REJECTED)
                return

            if current is not None:
                worker_to_join = self._current_worker_thread_ref
                self._state_transition_event.set()

        if worker_to_join is not None:
            worker_to_join.join()

        with self._lock:
            self._switch_command(kind, command, args, active_request)


    def _refresh_current(self, kind: CommandKind, command: SportCommand, args: list[Any], active_request: iox2.ActiveRequest) -> None:
        state = self._states[command]
        if kind == CommandKind.FLOAT_ARGS:
            state.set_floatargs(*map(float, args))

        state.execute(self._state_transition_event)
        self._respond(active_request, CommandStatus.OK)


    def _switch_command(self, kind: CommandKind, command: SportCommand, args: list[Any], active_request: iox2.ActiveRequest) -> None:
        state = self._states[command]
        if kind == CommandKind.FLOAT_ARGS:
            state.set_floatargs(*map(float, args))

        self._state_transition_event.clear()
        self._current_command = command
        self._current_locomotion_state = self._command_complete_locomotion_state_target[command]

        worker = threading.Thread(target=self._run_state, args=(state, active_request), daemon=True)
        self._current_worker_thread_ref = worker
        worker.start()


    def _run_state(self, state: State, active_request: iox2.ActiveRequest) -> None:
        state.execute(self._state_transition_event)

        with self._lock:
            self._current_command = None
            self._current_worker_thread_ref = None

        self._respond(active_request, CommandStatus.OK)


    def _respond(self, active_request: iox2.ActiveRequest, status: CommandStatus) -> None:
        if not active_request.is_connected:
            return

        response = active_request.loan_uninit()
        response = response.write_payload(CommandResponse_(status=status))
        response.send()
        active_request.delete()


    def shutdown(self) -> None:
        self._shutdown_event.set()

        if self._command_recv_thread_ref:
            self._command_recv_thread_ref.join()

        self._state_transition_event.set()

        with self._lock:
            if self._current_worker_thread_ref:
                self._current_worker_thread_ref.join()

        self._policy_controller.shutdown()

        for item in self._command_queue.flush():
            self._respond(item[3], CommandStatus.REJECTED)