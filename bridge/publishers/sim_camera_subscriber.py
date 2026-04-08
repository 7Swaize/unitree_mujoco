import os
import sys
import ctypes
import threading
import cv2
import numpy as np
from typing_extensions import override
import iceoryx2 as iox2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from iceoryx.msg.DepthFrame_ import DepthFrame_
from iceoryx.msg.RGBFrame_ import RGBFrame_
from iceoryx.constants import DDS_TOPIC_SIM_CAMERA_RGB, DDS_TOPIC_SIM_CAMERA_DEPTH


class SimCameraSubscriber():
    def __init__(self):
        super().__init__()
        iox2.set_log_level_from_env_or(iox2.LogLevel.Debug)

        self._node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)

        self._depth_service = self._node.service_builder(
            iox2.ServiceName.new(DDS_TOPIC_SIM_CAMERA_DEPTH)
        ).publish_subscribe(DepthFrame_).open_or_create()

        self._rgb_service = self._node.service_builder(
            iox2.ServiceName.new(DDS_TOPIC_SIM_CAMERA_RGB)
        ).publish_subscribe(RGBFrame_).open_or_create()

        self._depth_sub = self._depth_service.subscriber_builder().create()
        self._rgb_sub = self._rgb_service.subscriber_builder().create()

        self._cycle_time = iox2.Duration.from_millis(1)
        self._running = True

        # cache last frames so we can display side-by-side
        self._last_rgb = None
        self._last_depth = None

    @override
    def run(self):
        while self._running:
            self._node.wait(self._cycle_time)

            # ---- DEPTH ----
            while True:
                sample = self._depth_sub.receive()
                if sample is None:
                    break

                msg = sample.payload().contents

                depth_np = np.ctypeslib.as_array(msg.data)
                depth_np = depth_np.reshape((msg.height, msg.width))

                depth_frame = self._depth_to_colormap(
                    depth_np[::-1],
                    msg.depth_min,
                    msg.depth_max
                )

                self._last_depth = depth_frame
                print("received depth")

            # ---- RGB ----
            while True:
                sample = self._rgb_sub.receive()
                if sample is None:
                    break

                msg = sample.payload().contents

                rgb_np = np.ctypeslib.as_array(msg.data)
                rgb_np = rgb_np.reshape((msg.height, msg.width, 3))

                rgb_frame = cv2.cvtColor(rgb_np[::-1], cv2.COLOR_RGB2BGR)

                self._last_rgb = rgb_frame
                print("received rgb")

            # ---- VISUALIZE ----
            if self._last_rgb is not None and self._last_depth is not None:
                vis = np.hstack((self._last_rgb, self._last_depth))
                cv2.imshow("DEBUG_CAMERA", vis)
                cv2.waitKey(1)

    def stop(self):
        self._running = False

    def _depth_to_colormap(self, depth_np, depth_min, depth_max):
        rng = depth_max - depth_min or 1.0
        alpha = 255.0 / rng
        beta = -depth_min * alpha

        u8 = cv2.convertScaleAbs(depth_np, alpha=alpha, beta=beta)
        colormap = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

        colormap[(depth_np <= 0.0) | ~np.isfinite(depth_np)] = 0
        return colormap


main = SimCameraSubscriber()
main.run()