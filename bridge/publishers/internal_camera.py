import cv2
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from bridge.constants import DDS_SIM_CAMERA_TOPIC
from idl.msg._ImageData_ import ImageData_


class InternalCamera:
    def __init__(self) -> None:
        ChannelFactoryInitialize(1, "lo")
        self._sub = ChannelSubscriber(DDS_SIM_CAMERA_TOPIC, ImageData_)
        self._sub.Init(self._subscriber_cb, 10)

    
    def _subscriber_cb(self, msg: ImageData_):
        width = int(msg.res_x)
        height = int(msg.res_y)

        rgb_np = np.frombuffer(bytes(msg.rgb_data), dtype=np.uint8).reshape((height, width, 3))
        rgb_frame = cv2.cvtColor(rgb_np[::-1], cv2.COLOR_RGB2BGR)

        depth_np = np.array(msg.depth_data, dtype=np.float32).reshape((height, width))
        depth_frame = self._depth_to_colormap(depth_np[::-1], msg.depth_min, msg.depth_max)
        print("recieved frame")


        # ---- TEMP VIS (delete block later) ----
        vis = np.hstack((rgb_frame, depth_frame))  # side-by-side
        cv2.imshow("DEBUG_CAMERA", vis)
        cv2.waitKey(1)
        # --------------------------------------


    def _depth_to_colormap(self, depth_np: np.ndarray, depth_min: float, depth_max: float) -> np.ndarray:
        rng = depth_max - depth_min or 1.0
        alpha = 255.0 / rng
        beta = -depth_min * alpha

        u8 = cv2.convertScaleAbs(depth_np, alpha, beta)
        colormap = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO) # TODO: Maybe its COLORMAP_JET -> see what d345i uses.

        # Black out pixels that carried no valid depth return.
        colormap[(depth_np <= 0.0) | ~np.isfinite(depth_np)] = 0

        return colormap


cam = InternalCamera()
while True:
    pass