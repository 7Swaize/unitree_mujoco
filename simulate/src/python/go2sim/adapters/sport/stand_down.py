import time
import threading
import numpy as np
from typing_extensions import override

from ..adapter import Adapter
from .constants import SIMULATION_DT, TRANSITION_TAU

STAND_DOWN_DURATION = 3
STAND_DOWN_JOINT_POS = np.array([
                        0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
                        0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
                    ], dtype=float)


class StandDown(Adapter):
    @override
    def execute(self, cancel_event: threading.Event) -> None:
        runtime = 0.0

        start_pos = self._policy_controller.deactivate()
        last_q = start_pos.copy()

        while runtime < STAND_DOWN_DURATION and not cancel_event.is_set():
            step_start = time.perf_counter()
            runtime += SIMULATION_DT

            phase = np.tanh(runtime / TRANSITION_TAU)

            for i in range(12):
                target = phase * STAND_DOWN_JOINT_POS[i] + (1 - phase) * start_pos[i]
                self._lowcmd.motor_cmd[i].q = target
                self._lowcmd.motor_cmd[i].kp = 50.0 
                self._lowcmd.motor_cmd[i].dq = 0.0
                self._lowcmd.motor_cmd[i].kd = 3.5
                self._lowcmd.motor_cmd[i].tau = 0.0
                last_q[i] = target

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            time_until_next_step = SIMULATION_DT - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        self._policy_controller.override_joint_pos_no_active(last_q)