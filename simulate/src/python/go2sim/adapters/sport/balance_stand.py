import threading
from typing_extensions import override

from ..adapter import Adapter


class BalanceStand(Adapter):
    @override
    def execute(self, cancel_event: threading.Event) -> None:
        self._policy_controller.set_move_cmd(0.0, 0.0, 0.0)
        self._policy_controller.activate()
        cancel_event.wait(timeout=0.05)