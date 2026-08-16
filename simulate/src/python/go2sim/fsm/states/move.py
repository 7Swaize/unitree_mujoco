import threading
from typing_extensions import override

from .state import State


class Move(State):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0

    @override
    def set_floatargs(self, arg1: float, arg2: float, arg3: float) -> "State":
        self._vx = arg1
        self._vy = arg2
        self._vyaw = arg3
        return self

    @override
    def execute(self, cancel_event: threading.Event) -> None:
        self._policy_controller.set_move_cmd(self._vx, self._vy, self._vyaw)
        self._policy_controller.activate()
        cancel_event.wait(timeout=0.05)