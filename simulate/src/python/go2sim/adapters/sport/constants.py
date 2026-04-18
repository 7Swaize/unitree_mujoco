import numpy as np


SIMULATION_DT = 0.002
DDS_LOW_CMD_TOPIC = "rt/lowcmd"

STAND_UP_JOINT_POS = np.array([
                        0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                        0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
                    ], dtype=float)

STAND_DOWN_JOINT_POS = np.array([
                        0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
                        1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
                    ], dtype=float)