import time
import threading
import numpy as np

from pathlib import Path
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_

from ..receivers import HeightmapReceiver, RobotStateReceiver
from .policy_net import MLP

NPZ_POLICY_PATH: Path = Path(
    Path(__file__).parent / "policies_npz" / "policy_go2_pgtt_level13_run1.npz"
).resolve()

PGTT_DEFAULT_JOINT_POS = np.tile(np.array([0.0, 0.9, -1.8]), 4)
CMD_SCALE = np.array([0.5, 0.5, 0.8], dtype=np.float32)

POLICY_PD_DT = 0.005 # 200 Hz
POLICY_CTRL_DT = 0.02 # 50 Hz
POLICY_DECIMATION = round(POLICY_CTRL_DT / POLICY_PD_DT)

POLICY_KP = 40.0
POLICY_KD = 0.5
ACTION_SCALE = 0.5

MOVE_COMMAND_TIMEOUT_S = 1

DEFAULT_GAIT_FREQ_HZ = 2.0

# We don't keep the gait warm always (b/c of the deactivate).
# Therefore, we need to do a colde start after activate() call.
COLD_START_RAMP_CYCLES = 1.0

# Unitree publishes messages ordered [FR, FL, RR, RL].
# The model was trained on [FL, FR, RL, RR].
# So we need to convert when passing data into our policy.
HW_TO_TRAIN_JOINT_ORDER = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])


class PolicyController:
    def __init__(self, crc: CRC, low_cmd: LowCmd_, lowcmd_pub: ChannelPublisher) -> None:
        self._crc: CRC = crc
        self._lowcmd: LowCmd_ = low_cmd
        self._lowcmd_pub = lowcmd_pub
        self._gait_freq_hz = DEFAULT_GAIT_FREQ_HZ

        self._nn: MLP = MLP(NPZ_POLICY_PATH)
        self._heightmap_receiver: HeightmapReceiver = HeightmapReceiver()
        self._robotstate_receiver: RobotStateReceiver = RobotStateReceiver()
        self._phase: GaitPhase = GaitPhase()

        self._target_q = PGTT_DEFAULT_JOINT_POS.copy()
        self._last_q: np.ndarray = PGTT_DEFAULT_JOINT_POS.copy()
        self._last_action: np.ndarray = np.zeros(12, dtype=np.float32)
        self._mv_command: np.ndarray = np.zeros(3, dtype=np.float32)
        self._mv_command_ts: float = 0.0
        self._pd_counter: int = 0

        self._cold_start_ts: float = 0.0
        self._cold_start_ramp_s: float = COLD_START_RAMP_CYCLES / DEFAULT_GAIT_FREQ_HZ

        self._set_mv_cmd_lock: threading.Lock = threading.Lock()
        self._continue: threading.Event = threading.Event()
        self._shutdown: threading.Event = threading.Event()
        self._thread_ref: threading.Thread = threading.Thread(target=self._run, daemon=True)
        
        self._heightmap_receiver.start()
        self._thread_ref.start()

    def activate(self) -> None:
        if not self._continue.is_set():
            self._last_action[:] = 0.0
            self._phase.reset()
            self._cold_start_ts = time.monotonic()
            self._cold_start_ramp_s = (
                COLD_START_RAMP_CYCLES / self._gait_freq_hz
                if self._gait_freq_hz > 0.0
                else 0.0
            )
            self._continue.set()

    def deactivate(self) -> np.ndarray:
        self._continue.clear()
        return self._last_q

    def is_active(self) -> bool:
        return self._continue.is_set()

    # This is always called in a deactive state.
    # In a deactive state, the tick thread never touches anything that needs locks.
    # Therefore, I think we can forgoe any need of locks.
    def override_joint_pos_no_active(self, new_pos: np.ndarray) -> None:
        self._target_q = new_pos
        self._last_q = new_pos

    def set_move_cmd(self, vx: float, vy: float, vyaw: float) -> None:
        cmd = np.array([vx, vy, vyaw], dtype=np.float32) * CMD_SCALE
        with self._set_mv_cmd_lock:
            self._mv_command[:] = np.clip(cmd, -CMD_SCALE, CMD_SCALE)
            self._mv_command_ts = time.monotonic()

    def _cold_start_scale(self) -> float:
        if self._cold_start_ramp_s <= 0.0:
            return 1.0

        elapsed = time.monotonic() - self._cold_start_ts
        if elapsed >= self._cold_start_ramp_s:
            return 1.0
        if elapsed <= 0.0:
            return 0.0

        return 0.5 * (1.0 - np.cos(np.pi * elapsed / self._cold_start_ramp_s))

    def _run(self) -> None:
        next_tick = time.perf_counter()

        while not self._shutdown.is_set():
            next_tick += POLICY_PD_DT

            if self._continue.is_set():
                self._pd_tick()

            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                self._shutdown.wait(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _pd_tick(self) -> None:
        if self._pd_counter % POLICY_DECIMATION == 0:
            self._infer()
        self._pd_counter += 1

        for i in range(12):
            cmd = self._lowcmd.motor_cmd[i]
            cmd.q = float(self._target_q[i])
            cmd.kp = POLICY_KP
            cmd.dq = 0.0
            cmd.kd = POLICY_KD
            cmd.tau = 0.0

        self._lowcmd.crc = self._crc.Crc(self._lowcmd)
        self._lowcmd_pub.Write(self._lowcmd)

        self._last_q[:] = self._target_q

    def _infer(self) -> None:
        if not self._robotstate_receiver.ready:
            return

        state = self._robotstate_receiver.snapshot()
        phase = self._phase.step(self._gait_freq_hz, POLICY_CTRL_DT)
        phase_feat = self._phase.cos_sin()

        q_train_order = state["q"][HW_TO_TRAIN_JOINT_ORDER]
        dq_train_order = state["dq"][HW_TO_TRAIN_JOINT_ORDER]

        joint_angles = q_train_order - PGTT_DEFAULT_JOINT_POS
        joint_velocities = dq_train_order
        z_normal = self._heightmap_receiver.latest_z_normal()

        with self._set_mv_cmd_lock:
            if time.monotonic() - self._mv_command_ts > MOVE_COMMAND_TIMEOUT_S:
                self._mv_command[:] = 0.0

            eased_mv_command = self._mv_command * self._cold_start_scale()

            obs = np.concatenate([
                state["gyro"],
                state["gravity"],
                joint_angles,
                joint_velocities,
                phase_feat,
                z_normal,
                np.array([self._gait_freq_hz], dtype=np.float32),
                self._last_action,
                eased_mv_command,
            ], dtype=np.float32)

        action = self._nn(obs)
        self._last_action[:] = action
        self._target_q[:] = PGTT_DEFAULT_JOINT_POS + ACTION_SCALE * action


    def shutdown(self) -> None:
        self._shutdown.set()
        self._heightmap_receiver.shutdown()
        self._robotstate_receiver.shutdown()


class GaitPhase:
    PHASES_INIT = np.array([0.0, np.pi, np.pi, 0.0], dtype=np.float32)

    def __init__(self) -> None:
        self._phase = self.PHASES_INIT.copy()

    def reset(self) -> None:
        self._phase = self.PHASES_INIT.copy()

    def step(self, gait_freq_hz: float, dt: float) -> np.ndarray:
        self._phase = np.fmod(self._phase + 2.0 * np.pi * gait_freq_hz * dt, 2.0 * np.pi)
        return self._phase

    def cos_sin(self) -> np.ndarray:
        return np.concatenate(
            [np.cos(self._phase), np.sin(self._phase)],
            dtype=np.float32
        )