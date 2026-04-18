import queue
import threading
import numpy as np

from ..adapters import Adapter
from ..adapters.sport import Stop
from ..adapters.sport.constants import STAND_DOWN_JOINT_POS


class CommandManager:
    def __init__(self) -> None:
        self._thread = None
        self._stop_event = threading.Event()
        self._cmd_buf = queue.Queue[Adapter]()

        self._last_q: np.ndarray = STAND_DOWN_JOINT_POS

    def add_command(self, cmd: Adapter) -> None:
        if isinstance(cmd, Stop):
            self._clear_q()

        self._cmd_buf.put(cmd)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._exec_thread, daemon=True)
        self._thread.start()
        

    def _exec_thread(self):
        while not self._stop_event.is_set():
            self._last_q = self._cmd_buf.get().execute(self._last_q)
            self._cmd_buf.task_done()

    
    def _clear_q(self):
        with self._cmd_buf.mutex:
            self._cmd_buf.queue.clear()
            self._cmd_buf.all_tasks_done.notify_all()
            self._cmd_buf.not_full.notify_all()
            self._cmd_buf.unfinished_tasks = 0

    
    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join()
            self._thread = None