import queue
import threading
from enum import Enum, auto
from typing import Dict, Any, Optional

from unitree_sdk2py.utils.crc import CRC, LowCmd_
from unitree_sdk2py.core.channel import ChannelPublisher
from iceoryx_interfaces.mappings import SportCommand, CommandKind, CommandStatus

from ..nn import PolicyController
from .states import State
from .states import (
    Move
)

_CommandItem = tuple[CommandKind, SportCommand, list[Any]]


class CommandPreemption(Enum):
    REFRESHABLE = auto()
    LOCKED = auto()
    OVERRIDE = auto()


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

        self._lock: threading.Lock = threading.Lock()
        self._command_queue: queue.Queue[_CommandItem] = queue.Queue(maxsize=3)
        self._current_command: Optional[SportCommand] = None

        self._init_states()


    def _init_states(self) -> None:
        common = dict(crc=self._crc, lowcmd_pub=self._lowcmd_pub, lowcmd=self._lowcmd, policy_controller=self._policy_controller)

        self._states: Dict[SportCommand, State] = {
            SportCommand.MOVE: Move(**common)
        }

        self._state_preemptions: Dict[SportCommand, CommandPreemption] = {
            SportCommand.MOVE: CommandPreemption.REFRESHABLE
        }


    def start(self) -> None:
        self._command_recv_thread_ref = threading.Thread(target=self._command_recv_thread, daemon=True)
        self._command_recv_thread_ref.start()


    def receive_command(self, item: _CommandItem) -> None:
        try:
            self._command_queue.put_nowait(item)
        except queue.Full:
            evicted = self._command_queue.get_nowait()
            self._respond(evicted[3], CommandStatus.SUPERSEDED)
            self._command_queue.put_nowait(item)


    def _command_recv_thread(self) -> None:
        while not self._shutdown_event.is_set():
            try:
                item = self._command_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            self._dispatch(item)


    def _is_refreshable(self, command: SportCommand) -> bool:
        return self._state_preemptions[command] == CommandPreemption.REFRESHABLE


    def _can_preempt(self, current: SportCommand, incoming: SportCommand) -> bool:
        return (
            self._state_preemptions[current] is CommandPreemption.REFRESHABLE
            or self._state_preemptions[incoming] is CommandPreemption.OVERRIDE
        )


    def _dispatch(self, item: _CommandItem) -> None:
        kind, command, args = item

        worker_to_join: Optional[threading.Thread] = None
        with self._lock:
            current = self._current_command

            if current is not None and current == command and self._is_refreshable(command):
                self._refresh_current(kind, command, args)
                return

            if current is not None and not self._can_preempt(current, command):
                return

            if current is not None:
                worker_to_join = self._current_worker_thread_ref
                self._state_transition_event.set()

        if worker_to_join is not None:
            worker_to_join.join()

        with self._lock:
            self._switch_command(kind, command, args)


    def _refresh_current(self, kind: CommandKind, command: SportCommand, args: list[Any]) -> None:
        state = self._states[command]
        if kind == CommandKind.FLOAT_ARGS:
            state.set_floatargs(*map(float, args))

        state.execute(self._state_transition_event)


    def _switch_command(self, kind: CommandKind, command: SportCommand, args: list[Any]) -> None:
        state = self._states[command]
        if kind == CommandKind.FLOAT_ARGS:
            state.set_floatargs(*map(float, args))

        self._state_transition_event.clear()
        self._current_command = command

        worker = threading.Thread(target=self._run_state, args=(command, state), daemon=True)
        self._current_worker_thread_ref = worker
        worker.start()


    def _run_state(self, state: State) -> None:
        state.execute(self._state_transition_event)

        with self._lock:
            self._current_command = None
            self._current_worker_thread_ref = None


    def shutdown(self) -> None:
        self._shutdown_event.set()

        if self._command_recv_thread_ref:
            self._command_recv_thread_ref.join()

        self._state_transition_event.set()

        with self._lock:
            if self._current_worker_thread_ref:
                self._current_worker_thread_ref.join()

        self._policy_controller.shutdown()

        while True:
            try:
                item = self._command_queue.get_nowait()
            except queue.Empty:
                break