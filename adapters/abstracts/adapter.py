from abc import ABC, abstractmethod
from typing import Callable, List


class Adapter(ABC):
    def __init__(self) -> None:
        self._on_start: List[Callable[["Adapter"], None]] = []
        self._on_end: List[Callable[["Adapter"], None]] = []

    @abstractmethod
    def execute(self) -> None:
        pass

    def add_on_start(self, callback: Callable[["Adapter"], None]) -> None:
        self._on_start.append(callback)

    def add_on_end(self, callback: Callable[["Adapter"], None]) -> None:
        self._on_end.append(callback)

    def _notify_start(self) -> None:
        for cb in self._on_start:
            cb(self)

    def _notify_end(self) -> None:
        for cb in self._on_end:
            cb(self)

    def run(self) -> None:
        self._notify_start()
        try:
            self.execute()
        finally:
            self._notify_end()