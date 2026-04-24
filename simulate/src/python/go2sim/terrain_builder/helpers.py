import numpy as np
from typing import Tuple, Union
from scipy.spatial.transform import Rotation as R

# ALL - XYZ input as degrees

def euler_to_quat(x: float, y: float, z: float) -> np.ndarray:
    quat = R.from_euler("xyz", [x, y, z], degrees=True).as_quat()
    return np.array([quat[3], quat[0], quat[1], quat[2]]) # Mujoco uses wxyz quats

def euler_to_rot(x: float, y: float, z: float) -> np.ndarray:
    return R.from_euler("xyz", [x, y, z], degrees=True).as_matrix()

def rot2d(x: float, y: float, yaw: float) -> Tuple[float, float]:
    rot = R.from_euler("z", yaw, degrees=True)
    rotated = rot.apply([x, y, 0])
    return rotated[0], rotated[1]

def rot3d(pos: np.ndarray, euler: np.ndarray) -> np.ndarray:
    rot = R.from_euler("xyz", euler, degrees=True)
    return rot.apply(pos)

def list_to_str(vec: Union[list, np.ndarray]) -> str:
    return " ".join(str(s) for s in vec)
