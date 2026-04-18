import time
import numpy as np
from typing_extensions import override

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from ..adapter import Adapter
from .constants import SIMULATION_DT


class StandUp(Adapter):
    def __init__(self, crc: CRC, lowcmd_pub: ChannelPublisher, lowcmd: LowCmd_) -> None:
        super().__init__(crc, lowcmd_pub, lowcmd)

        self._stand_up_join_pos = np.array([
            0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
            0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
        ], dtype=float)

    
    @override
    def execute(self, start_pos: np.ndarray) -> np.ndarray:
        runtime = 0.0
        duration = 3 # Actual total time for standing up or standing down is about 1.2s
        self._last_q = start_pos

        while (runtime < duration):
            step_start = time.perf_counter()
            runtime += SIMULATION_DT

            phase = np.tanh(runtime / 1.2)

            for i in range(12):
                target = phase * self._stand_up_join_pos[i] + (1 - phase) * start_pos[i]
                self._lowcmd.motor_cmd[i].q = target
                self._lowcmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                self._lowcmd.motor_cmd[i].dq = 0.0
                self._lowcmd.motor_cmd[i].kd = 3.5
                self._lowcmd.motor_cmd[i].tau = 0.0
                self._last_q[i] = target

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            time_until_next_step = SIMULATION_DT - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        return self._last_q