import time
import threading
from typing_extensions import override

from .state import State
from .constants import SIMULATION_DT

DAMP_KD = 3.0
DAMP_DURATION = 0.5


class Damp(State):
    @override
    def execute(self, cancel_event: threading.Event) -> None:
        start_pos = self._policy_controller.deactivate()
        last_q = start_pos.copy()

        runtime = 0.0
        while runtime < DAMP_DURATION and not cancel_event.is_set():
            step_start = time.perf_counter()
            runtime += SIMULATION_DT

            for i in range(12):
                cmd = self._lowcmd.motor_cmd[i]
                cmd.q = 0.0
                cmd.kp = 0.0
                cmd.dq = 0.0
                cmd.kd = DAMP_KD
                cmd.tau = 0.0

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            remaining = SIMULATION_DT - (time.perf_counter() - step_start)
            if remaining > 0:
                cancel_event.wait(remaining)

        self._policy_controller.override_joint_pos_no_active(last_q)
