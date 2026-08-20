import threading
from typing import Callable, Dict, Optional

from iceoryx_interfaces.mappings import SportCommand

from .typing import _CommandItem

# Straight out the depths of Claude, but it works.
class CommandPriorityQueue:
    def __init__(self, maxsize: int, priority: Dict[SportCommand, int]) -> None:
        self._maxsize = maxsize
        self._priority = priority
        self._items: list[_CommandItem] = []
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)

    def put(self, item: _CommandItem, on_rejected: Callable[[_CommandItem], None]) -> None:
        _, command, _, _ = item
        stale: Optional[_CommandItem] = None

        with self._lock:
            for i, existing in enumerate(self._items):
                if existing[1] == command:
                    stale = self._items[i]
                    self._items[i] = item
                    self._not_empty.notify()
                    break
            else:
                if len(self._items) < self._maxsize:
                    self._items.append(item)
                    self._not_empty.notify()
                else:
                    idx_to_evict = self._find_eviction_candidate(command)
                    if idx_to_evict is None:
                        stale = item
                    else:
                        stale = self._items[idx_to_evict]
                        self._items[idx_to_evict] = item
                        self._not_empty.notify()

        if stale is not None:
            on_rejected(stale[3])

    def _find_eviction_candidate(self, incoming: SportCommand) -> Optional[int]:
        incoming_priority = self._priority[incoming]
        worst_idx: Optional[int] = None
        worst_priority: Optional[int] = None
        for i, (_, cmd, _, _) in enumerate(self._items):
            p = self._priority[cmd]
            if p < incoming_priority and (worst_priority is None or p < worst_priority):
                worst_idx = i
                worst_priority = p

        return worst_idx

    def get(self, timeout: float) -> Optional[_CommandItem]:
        with self._not_empty:
            if not self._items and not self._not_empty.wait(timeout=timeout):
                return None
            if not self._items:
                return None

            best_idx = max(range(len(self._items)), key=lambda i: self._priority[self._items[i][1]])
            return self._items.pop(best_idx)

    def flush(self) -> list[_CommandItem]:
        with self._lock:
            items = self._items
            self._items = []
            return items