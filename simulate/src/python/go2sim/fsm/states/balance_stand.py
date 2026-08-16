import time
import threading
from typing_extensions import override

from ...nn.policy_controller import POLICY_KP, POLICY_KD, PGTT_DEFAULT_JOINT_POS
from .state import State
from .constants import SIMULATION_DT

MOTOR_SETTLE_DURATION = 0.3
STOP_SETTLE_DURATION = 0.8

class BalanceStand(State):
    @override
    def execute(self, cancel_event: threading.Event) -> None:
        if self._policy_controller.is_active():
            self._policy_controller.set_move_cmd(0.0, 0.0, 0.0)
            cancel_event.wait(MOTOR_SETTLE_DURATION)

        start_pos = self._policy_controller.deactivate()
        last_q = start_pos.copy()

        runtime = 0.0
        while runtime < STOP_SETTLE_DURATION and not cancel_event.is_set():
            step_start = time.perf_counter()
            runtime += SIMULATION_DT
            percent = min(runtime / STOP_SETTLE_DURATION, 1.0)

            for i in range(12):
                target = float((1 - percent) * start_pos[i] + percent * PGTT_DEFAULT_JOINT_POS[i])
                cmd = self._lowcmd.motor_cmd[i]
                cmd.q = target
                cmd.kp = POLICY_KP
                cmd.dq = 0.0
                cmd.kd = POLICY_KD
                cmd.tau = 0.0
                last_q[i] = target

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            remaining = SIMULATION_DT - (time.perf_counter() - step_start)
            if remaining > 0:
                cancel_event.wait(remaining)

        self._policy_controller.override_joint_pos_no_active(last_q)