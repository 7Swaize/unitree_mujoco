import threading
from typing_extensions import override

from ..adapter import Adapter


class StopMove(Adapter):
    @override
    def execute(self, cancel_event: threading.Event) -> None:
        pass
