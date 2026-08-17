import numba
import threading
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


# Definition moved out here so we can allow for JIT comp. Can't take a reference to a non jitclass 'self'.
@numba.njit()
def quat_to_gravity_and_yaw(w: float, x: float, y: float, z: float) -> tuple[np.ndarray, np.float32]:
    # Builtin publishing of quaternion data from mujoco::mjData::sensordata is guaranteed to be normalized.
    # Therefore, we can use the inhomogenous matrix specified here: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    x2 = x + x
    y2 = y + y
    z2 = z + z

    xx = x * x2
    xy = x * y2
    xz = x * z2
    yy = y * y2
    yz = y * z2
    zz = z * z2
    wx = w * x2
    wy = w * y2
    wz = w * z2

    gravity_body = np.empty(3, dtype=np.float32)
    gravity_body[0] = -(xz - wy)
    gravity_body[1] = -(yz + wx)
    gravity_body[2] = -1 + (xx + yy)

    yaw = np.arctan2(xy + wz, 1 - (yy + zz))
    return gravity_body, np.float32(yaw)


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

        gravity, yaw = quat_to_gravity_and_yaw(w, x, y, z)
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
    

    def shutdown(self) -> None:
        self._sub.Close()

