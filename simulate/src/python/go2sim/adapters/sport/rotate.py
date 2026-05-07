import time
import numpy as np
from typing_extensions import override

from ..adapter import Adapter
from .constants import SIMULATION_DT, STAND_UP_JOINT_POS

# AI - Generated Experiment File (Updated for Higher Speed)

_LEG_PHASE_OFFSET = [0.0, np.pi, np.pi, 0.0]    # FR, FL, RR, RL  (diagonal trot)
_ROT_STRIDE_SIGN  = [1.0, -1.0,  1.0, -1.0]     # FR, FL, RR, RL

# --- SPEED TUNING PARAMETERS (Updated for 4.0 Max) ---
_GAIT_FREQ    = 2.5    # Keeping the high-speed cycle frequency
_STRIDE_AMP   = 0.25   # Keeping the wide thigh sweep
_LIFT_AMP     = 0.22   # Keeping the stable foot lift
_ROT_DURATION = 3.0    
_VROT_MAX     = 4.0    # Lowered from 6.0: Now "4" triggers the full 0.25 stride amp
# -------------------------------

class Rotate(Adapter):
    @override
    def set_floatargs(self, arg1: float, arg2: float) -> "Adapter":
        self._vrot = arg1
        return self

    @override
    def execute(self, start_pos: np.ndarray) -> np.ndarray:
        runtime   = 0.0
        # Omega governs the cycle speed
        omega     = 2.0 * np.pi * _GAIT_FREQ
        vrot_norm = float(np.clip(self._vrot / _VROT_MAX, -1.0, 1.0))

        base_q        = STAND_UP_JOINT_POS.copy()
        self._last_q = start_pos.copy()

        while runtime < _ROT_DURATION:
            step_start = time.perf_counter()
            runtime   += SIMULATION_DT

            # Quick blend to prevent jerky start
            blend = float(np.clip(runtime / 0.3, 0.0, 1.0))

            for leg in range(4):
                idx   = leg * 3
                phase = omega * runtime + _LEG_PHASE_OFFSET[leg]
                sin_p = np.sin(phase)

                # Thigh sweep: Increased amplitude = faster yaw per step
                thigh_delta = _STRIDE_AMP * vrot_norm * _ROT_STRIDE_SIGN[leg] * sin_p

                # Foot lift: sin_p > 0 is the swing phase
                calf_delta = _LIFT_AMP * max(0.0, sin_p)

                hip_q   = blend * base_q[idx]                   + (1.0 - blend) * start_pos[idx]
                thigh_q = blend * (base_q[idx+1] + thigh_delta) + (1.0 - blend) * start_pos[idx+1]
                calf_q  = blend * (base_q[idx+2] + calf_delta)  + (1.0 - blend) * start_pos[idx+2]

                for j, q_target in enumerate((hip_q, thigh_q, calf_q)):
                    cmd       = self._lowcmd.motor_cmd[idx + j]
                    cmd.q     = q_target
                    cmd.kp    = 55.0  # Slightly stiffened for higher frequency response
                    cmd.dq    = 0.0
                    cmd.kd    = 3.8   # Slightly increased damping to prevent oscillation
                    cmd.tau   = 0.0
                    self._last_q[idx + j] = q_target

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            elapsed = time.perf_counter() - step_start
            if SIMULATION_DT - elapsed > 0:
                time.sleep(SIMULATION_DT - elapsed)

        return self._last_q