import threading
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


class RobotStateReceiver:
    def __init__(self) -> None:
        self._lock: threading.Lock = threading.Lock()

        self._q: np.ndarray = np.zeros(12, dtype=np.float64)
        self._dq: np.ndarray = np.zeros(12, dtype=np.float64)
        self._quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])
        self._gyro: np.ndarray = np.zeros(3, dtype=np.float64)
        self._ready = False

        self._sub: ChannelSubscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._on_lowstate, 10)


    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready


    def snapshot(self) -> dict[str, np.ndarray]:
        with self._lock:
            q = self._q.copy()
            dq = self._dq.copy()
            w, x, y, z = self._quat
            gyro = self._gyro.copy().astype(np.float32, copy=False)

        gravity, yaw = self._quat_to_gravity_and_yaw(w, x, y, z)
        return {
            "q": q,
            "dq": dq,
            "gyro": gyro,
            "gravity": gravity,
            "yaw": yaw,
        }


    def _on_lowstate(self, msg: LowState_) -> None:
        with self._lock:
            for i in range(12):
                self._q[i] = msg.motor_state[i].q
                self._dq[i] = msg.motor_state[i].dq
            self._quat[:] = msg.imu_state.quaternion
            self._gyro[:] = msg.imu_state.gyroscope
            self._ready = True


    def _quat_to_gravity_and_yaw(self, w: float, x: float, y: float, z: float) -> tuple[np.ndarray, float]:
        # Builtin publishing of quaternion data from mujoco::mjData::sensordata is guaranteed to be normalized.
        # Therefore, we can use the inhomogenous matrix specified here: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        R = np.array([
            [1 - 2 * (y * y + z * z),   2 * (x * y - z * w),       2 * (x * z + y * w)],
            [2 * (x * y + z * w),       1 - 2 * (x * x + z * z),   2 * (y * z - x * w)],
            [2 * (x * z - y * w),       2 * (y * z + x * w),       1 - 2 * (x * x + y * y)],
        ])

        # Since the matrix is orthogonal, its inverse is its transpose
        gravity_body = R.T @ np.array([0.0, 0.0, -1.0])

        yaw = np.arctan2(R[1, 0], R[0, 0])
        return gravity_body.astype(np.float32, copy=False), float(yaw)
    

    def shutdown(self) -> None:
        self._sub.Close()

