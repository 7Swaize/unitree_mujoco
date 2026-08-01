import numpy as np

from ..receivers import HeightmapReceiver, RobotStateReceiver


class PolicyController:
    def __init__() -> None:
        pass

    def deactivate(self) -> np.ndarray:
        # Return a view here
        pass

    def override_joint_pos(self, new_pos: np.ndarray) -> None:
        pass

    def shutdown(self) -> None:
        pass