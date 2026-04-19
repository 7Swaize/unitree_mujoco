import time
import numpy as np
from typing_extensions import override

from ..adapter import Adapter
from .constants import SIMULATION_DT, STAND_UP_JOINT_POS

# Diagonal trot — same phase pairs as Move (FR+RL, FL+RR).
# Two-point cross-body support; dynamically stable.
#
#   Leg  │ idx │ phase │ sign │ Reason
#   ─────┼─────┼───────┼──────┼────────────────────────────────────────────
#   FR=0 │  0  │   0   │  +1  │ r_y < 0 → needs +Fx → foot sweeps back
#   FL=1 │  3  │   π   │  −1  │ r_y > 0 → needs −Fx → foot sweeps forward
#   RR=2 │  6  │   π   │  +1  │ r_y < 0 → needs +Fx → foot sweeps back
#   RL=3 │  9  │   0   │  −1  │ r_y > 0 → needs −Fx → foot sweeps forward
#
# During stance sin_p < 0:
#   FR/RL: delta = +1 * sin_p < 0 → thigh back  → Fx > 0 → τ_z > 0 (CCW) ✓
#   FL/RR: delta = −1 * sin_p > 0 → thigh fwd   → Fx < 0 → τ_z > 0 (CCW) ✓
_LEG_PHASE_OFFSET = [0.0, np.pi, np.pi, 0.0]   # FR, FL, RR, RL  (diagonal trot)
_ROT_STRIDE_SIGN  = [1.0, -1.0,  1.0, -1.0]    # FR, FL, RR, RL

_GAIT_FREQ    = 1.5
_STRIDE_AMP   = 0.18   # rad – thigh excursion at full vrot
_LIFT_AMP     = 0.25   # rad – calf lift during swing
_ROT_DURATION = 3.0    # s per command
_VROT_MAX     = 4.0    # rad/s


class Rotate(Adapter):
    @override
    def set_floatargs(self, arg1: float, arg2: float) -> "Adapter":
        self._vrot = arg1
        return self

    @override
    def execute(self, start_pos: np.ndarray) -> np.ndarray:
        runtime   = 0.0
        omega     = 2.0 * np.pi * _GAIT_FREQ
        vrot_norm = float(np.clip(self._vrot / _VROT_MAX, -1.0, 1.0))

        base_q       = STAND_UP_JOINT_POS.copy()
        self._last_q = start_pos.copy()

        while runtime < _ROT_DURATION:
            step_start = time.perf_counter()
            runtime   += SIMULATION_DT

            blend = float(np.clip(runtime / 0.4, 0.0, 1.0))

            for leg in range(4):
                idx   = leg * 3
                phase = omega * runtime + _LEG_PHASE_OFFSET[leg]
                sin_p = np.sin(phase)

                # Opposite thigh sweep direction on each side to generate yaw torque
                thigh_delta = _STRIDE_AMP * vrot_norm * _ROT_STRIDE_SIGN[leg] * sin_p

                # Foot lift only during swing half-cycle
                calf_delta = _LIFT_AMP * max(0.0, sin_p)

                # Hip stays neutral — no lateral shift needed for pure yaw
                hip_q   = blend * base_q[idx]                   + (1.0 - blend) * start_pos[idx]
                thigh_q = blend * (base_q[idx+1] + thigh_delta) + (1.0 - blend) * start_pos[idx+1]
                calf_q  = blend * (base_q[idx+2] + calf_delta)  + (1.0 - blend) * start_pos[idx+2]

                for j, q_target in enumerate((hip_q, thigh_q, calf_q)):
                    cmd       = self._lowcmd.motor_cmd[idx + j]
                    cmd.q     = q_target
                    cmd.kp    = 50.0
                    cmd.dq    = 0.0
                    cmd.kd    = 3.5
                    cmd.tau   = 0.0
                    self._last_q[idx + j] = q_target

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            elapsed = time.perf_counter() - step_start
            if SIMULATION_DT - elapsed > 0:
                time.sleep(SIMULATION_DT - elapsed)

        return self._last_q