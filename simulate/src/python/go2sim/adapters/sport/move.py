import time
import numpy as np
from typing_extensions import override

from ..adapter import Adapter
from .constants import SIMULATION_DT, STAND_UP_JOINT_POS

# AI - Generated Experement File

# Diagonal trot pairs: (FR=0, RL=3) and (FL=1, RR=2)
# Each entry is the gait phase offset (radians) for that leg.
# Legs 0 and 3 are in-phase; legs 1 and 2 are in-phase but 180° offset.
#
#   Leg index │ Joint indices │ Side  │ Trot pair
#   ──────────┼───────────────┼───────┼──────────
#      0 (FR) │   0,  1,  2  │ Right │ A
#      1 (FL) │   3,  4,  5  │ Left  │ B
#      2 (RR) │   6,  7,  8  │ Right │ B
#      3 (RL) │   9, 10, 11  │ Left  │ A
_LEG_PHASE_OFFSET = [0.0, np.pi, np.pi, 0.0]  # FR, FL, RR, RL

# Positive sign for the hip-abduction joint per leg.
# FR/RR hips are positive-outward; FL/RL are negative-outward.
_HIP_LATERAL_SIGN = [1.0, -1.0, 1.0, -1.0]  # FR, FL, RR, RL

_GAIT_FREQ = 1.7  # Increased from 1.5 Hz
_STRIDE_AMP = 0.30  # Increased from 0.18 rad
_LIFT_AMP = 0.30  # Slightly increased from 0.25 rad
_LAT_AMP        = 0.04   # rad – hip abduction per unit vy (clipped to ±1 m/s)
_MOVE_DURATION  = 3.0    # seconds per command invocation (matches StandUp/StandDown)


class Move(Adapter):
    @override
    def set_floatargs(self, arg1: float, arg2: float) -> "Adapter":
        self._vx = arg1
        self._vy = arg2
        return self

    @override
    def execute(self, start_pos: np.ndarray) -> np.ndarray:
        runtime = 0.0
        omega = 2.0 * np.pi * _GAIT_FREQ

        # Interpolate from the incoming position to the nominal stand pose at
        # the start so we don't jerk if the robot is slightly off-pose.
        base_q = STAND_UP_JOINT_POS.copy()
        self._last_q = start_pos.copy()

        while runtime < _MOVE_DURATION:
            step_start = time.perf_counter()
            runtime += SIMULATION_DT

            # Blend from start_pos → base_q over the first 0.4 s to prevent
            # initial joint jerk.
            blend = float(np.clip(runtime / 0.4, 0.0, 1.0))

            for leg in range(4):
                idx = leg * 3  # first joint index for this leg
                phase = omega * runtime + _LEG_PHASE_OFFSET[leg]

                sin_p = np.sin(phase)

                # ── Fore-aft thigh swing (drives vx) ─────────────────────────
                thigh_delta = _STRIDE_AMP * self._vx * sin_p

                # ── Foot lift during swing half-cycle (calf is negative = extended)
                # sin_p > 0  →  swing phase  →  raise calf (less negative = more bent)
                calf_delta = _LIFT_AMP * max(0.0, sin_p)

                # ── Lateral hip abduction (drives vy) ────────────────────────
                # All four legs abduct in-phase to shift the body laterally.
                hip_delta = _LAT_AMP * self._vy * _HIP_LATERAL_SIGN[leg] * sin_p

                # Blend from start_pos for smooth entry.
                hip_q    = blend * (base_q[idx]     + hip_delta)   + (1.0 - blend) * start_pos[idx]
                thigh_q  = blend * (base_q[idx + 1] + thigh_delta) + (1.0 - blend) * start_pos[idx + 1]
                calf_q   = blend * (base_q[idx + 2] + calf_delta)  + (1.0 - blend) * start_pos[idx + 2]

                for j, q_target in enumerate((hip_q, thigh_q, calf_q)):
                    cmd = self._lowcmd.motor_cmd[idx + j]
                    cmd.q   = q_target
                    cmd.kp  = 50.0
                    cmd.dq  = 0.0
                    cmd.kd  = 3.5
                    cmd.tau = 0.0
                    self._last_q[idx + j] = q_target

            self._lowcmd.crc = self._crc.Crc(self._lowcmd)
            self._lowcmd_pub.Write(self._lowcmd)

            elapsed = time.perf_counter() - step_start
            remaining = SIMULATION_DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

        return self._last_q