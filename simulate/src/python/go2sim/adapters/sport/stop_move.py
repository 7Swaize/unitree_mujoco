import time
import threading
from typing_extensions import override

from ...nn.policy_controller import POLICY_KP, POLICY_KD, PGTT_DEFAULT_JOINT_POS
from ..adapter import Adapter
from .constants import SIMULATION_DT

STOP_SETTLE_DURATION = 0.3


class StopMove(Adapter):
    @override
    def execute(self, cancel_event: threading.Event) -> None:
        start_pos = self._policy_controller.deactivate()
        target_pos = PGTT_DEFAULT_JOINT_POS

        runtime = 0.0
        while runtime < STOP_SETTLE_DURATION and not cancel_event.is_set():
            step_start = time.perf_counter()
            runtime += SIMULATION_DT
            percent = min(runtime / STOP_SETTLE_DURATION, 1.0)

            for i in range(12):
                cmd = self._lowcmd.motor_cmd[i]
                cmd.q = float((1 - percent) * start_pos[i] + percent * target_pos[i])
                cmd.kp = POLICY_KP
                cmd.dq = 0.0
                cmd.kd = POLICY_KD
                cmd.tau = 0.0

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            remaining = SIMULATION_DT - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)

        self._policy_controller.override_joint_pos_no_active(target_pos.copy())