import os
import sys
import threading
import cv2
import numpy as np
import iceoryx2 as iox2
from typing_extensions import override

# sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from .iceoryx.msg.DepthFrame_ import DepthFrame_
from .iceoryx.msg.RGBFrame_ import RGBFrame_
from .iceoryx.constants import *

class SimCameraSubscriber(threading.Thread):
    def __init__(self):
        super().__init__()
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)

        self._node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self._depth_service = self._node.service_builder(iox2.ServiceName.new(IOX2_TOPIC_SIM_CAMERA_DEPTH)) \
                                        .publish_subscribe(DepthFrame_) \
                                        .max_publishers(IOX2_MAX_PUBLISHERS) \
                                        .max_subscribers(IOX2_MAX_SUBSCRIBERS) \
                                        .subscriber_max_buffer_size(IOX2_SUBSCRIBER_MAX_BUFFER_SIZE) \
                                        .subscriber_max_borrowed_samples(IOX2_SUBSCRIBER_MAX_BORROWED_SAMPLES) \
                                        .history_size(IOX2_HISTORY_SIZE) \
                                        .open_or_create()

        self._rgb_service = self._node.service_builder(iox2.ServiceName.new(IOX2_TOPIC_SIM_CAMERA_RGB)) \
                                        .publish_subscribe(RGBFrame_) \
                                        .max_publishers(IOX2_MAX_PUBLISHERS) \
                                        .max_subscribers(IOX2_MAX_SUBSCRIBERS) \
                                        .subscriber_max_buffer_size(IOX2_SUBSCRIBER_MAX_BUFFER_SIZE) \
                                        .subscriber_max_borrowed_samples(IOX2_SUBSCRIBER_MAX_BORROWED_SAMPLES) \
                                        .history_size(IOX2_HISTORY_SIZE) \
                                        .open_or_create()

        self._depth_sub = self._depth_service.subscriber_builder().create()
        self._rgb_sub = self._rgb_service.subscriber_builder().create()
        self._cycle_time = iox2.Duration.from_millis(1)
        self._running = True

        # ---- DEBUG BUFFER ----
        self._latest_rgb = None
        self._latest_depth = None
        # ---------------------

    
    @override
    def run(self):
        while self._running:
            self._node.wait(self._cycle_time)

            while True:
                sample = self._depth_sub.receive()
                if sample is None:
                    break

                msg = sample.payload().contents
                self._add_depth_to_buffer(
                    np.array(msg.data, copy=True, dtype=np.float32).reshape((msg.height, msg.width)),
                    msg.depth_min,
                    msg.depth_max
                )

            while True:
                sample = self._rgb_sub.receive()
                if sample is None:
                    break

                msg = sample.payload().contents
                self._add_rgb_to_buffer(
                    np.array(msg.data, copy=True, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                )

            # ---- DEBUG VISUALIZATION (ISOLATED BLOCK) ----
            if self._latest_rgb is not None and self._latest_depth is not None:
                vis = np.hstack((self._latest_rgb, self._latest_depth))
                cv2.imshow("DEBUG_CAMERA", vis)
                cv2.waitKey(1)
            # ------------------------------------------------


    def _add_depth_to_buffer(self, depth_np: np.ndarray, depth_min: float, depth_max: float) -> None:
        depth_frame = self._depth_to_colormap(depth_np[::-1], depth_min, depth_max)

        # TODO: Add to buffer

        # ---- DEBUG BUFFER ----
        self._latest_depth = depth_frame
        # ---------------------


    def _add_rgb_to_buffer(self, rgb_np: np.ndarray) -> None:
        rgb_frame = cv2.cvtColor(rgb_np[::-1], cv2.COLOR_RGB2BGR)

        # TODO: Add to buffer

        # ---- DEBUG BUFFER ----
        self._latest_rgb = rgb_frame
        # ---------------------


    def _depth_to_colormap(self, depth_np: np.ndarray, depth_min: float, depth_max: float) -> np.ndarray:
        rng = depth_max - depth_min or 1.0
        alpha = 255.0 / rng
        beta = -depth_min * alpha

        u8 = cv2.convertScaleAbs(depth_np, alpha=alpha, beta=beta)
        colormap = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO) # TODO: Maybe its COLORMAP_JET -> see what d345i uses.

        # Black out pixels that carried no valid depth return.
        colormap[(depth_np <= 0.0) | ~np.isfinite(depth_np)] = 0

        return colormap


    def stop(self):
        self._running = False


main = SimCameraSubscriber()
main.start()