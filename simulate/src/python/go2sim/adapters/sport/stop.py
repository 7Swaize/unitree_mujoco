import time
import numpy as np
from typing_extensions import override

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from .constants import SIMULATION_DT, STAND_UP_JOINT_POS, STAND_DOWN_JOINT_POS
from ..adapter import Adapter


class Stop(Adapter):
    @override
    def execute(self, start_pos: np.ndarray) -> np.ndarray:
        runtime = 0.0
        STOP_DURATION = 0.5

        dist_to_down = np.linalg.norm(start_pos - STAND_DOWN_JOINT_POS)
        if dist_to_down < 0.8: 
            target_q = STAND_DOWN_JOINT_POS.copy() # If we are already sitting down, we dont want to stand up.
        else:
            target_q = STAND_UP_JOINT_POS.copy()

        self._last_q = start_pos.copy()

        while runtime < STOP_DURATION:
            step_start = time.perf_counter()
            runtime += SIMULATION_DT

            alpha = float(np.clip(runtime / STOP_DURATION, 0.0, 1.0))

            for i in range(12):
                q_target = start_pos[i] + (target_q[i] - start_pos[i]) * alpha
                
                cmd = self._lowcmd.motor_cmd[i]
                cmd.q = q_target
                cmd.kp = 60.0
                cmd.dq = 0.0
                cmd.kd = 5.0
                cmd.tau = 0.0
                self._last_q[i] = q_target

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            elapsed = time.perf_counter() - step_start
            remaining = SIMULATION_DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

        return self._last_q