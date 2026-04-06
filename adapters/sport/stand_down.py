import time
import numpy as np
from typing import override

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from ..abstracts.sport_adapter import SportAdapter
from ..constants import SIMULATION_DT, DDS_LOW_CMD_TOPIC


class StandDown(SportAdapter):
    def __init__(self, crc: CRC) -> None:
        super().__init__(crc)

        self._stand_down_join_pos = np.array([
            0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
            1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
        ], dtype=float)

        self._pub = ChannelPublisher(DDS_LOW_CMD_TOPIC, LowCmd_)
        self._pub.Init()
        self._cmd = unitree_go_msg_dds__LowCmd_()

    
    @override
    def execute(self) -> None:
        self._zero_lowcmd_msg(self._cmd)

        execution_time_s = 3
        runtime = 0.0

        while (runtime < execution_time_s):
            step_start = time.perf_counter()
            runtime += SIMULATION_DT
            phase = np.tanh(runtime / 1.2) # Actual total time for standing up or standing down is about 1.2s

            for i in range(12):
                self._cmd.motor_cmd[i].q = phase * self._stand_down_join_pos[i] + (
                    1 - phase) * self._stand_down_join_pos[i]
                self._cmd.motor_cmd[i].kp = 50.0
                self._cmd.motor_cmd[i].dq = 0.0
                self._cmd.motor_cmd[i].kd = 3.5
                self._cmd.motor_cmd[i].tau = 0.0

            self._cmd.crc = self._crc.Crc(self._cmd)
            self._pub.Write(self._cmd)

            time_until_next_step = SIMULATION_DT - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)