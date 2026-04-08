import os
import sys
import threading
from typing_extensions import override
import iceoryx2 as iox2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from iceoryx.msg.DepthFrame_ import DepthFrame_
from iceoryx.msg.RGBFrame_ import RGBFrame_
from iceoryx.constants import DDS_TOPIC_SIM_CAMERA_RGB, DDS_TOPIC_SIM_CAMERA_DEPTH

class SimCameraSubscriber(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        iox2.set_log_level_from_env_or(iox2.LogLevel.Debug)

        self._node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self._depth_service = self._node.service_builder(iox2.ServiceName.new(DDS_TOPIC_SIM_CAMERA_DEPTH)) \
                                        .publish_subscribe(DepthFrame_) \
                                        .max_subscribers(1) \
                                        .max_publishers(1) \
                                        .subscriber_max_buffer_size(2) \
                                        .subscriber_max_borrowed_samples(1) \
                                        .history_size(1) \
                                        .open_or_create()

        self._rgb_service = self._node.service_builder(iox2.ServiceName.new(DDS_TOPIC_SIM_CAMERA_RGB)) \
                                        .publish_subscribe(RGBFrame_) \
                                        .max_subscribers(1) \
                                        .max_publishers(1) \
                                        .subscriber_max_buffer_size(2) \
                                        .subscriber_max_borrowed_samples(1) \
                                        .history_size(1) \
                                        .open_or_create()

        self._depth_sub = self._depth_service.subscriber_builder().create()
        self._rgb_sub = self._rgb_service.subscriber_builder().create()
        self._cycle_time = iox2.Duration.from_millis(1)
        self._running = True

    
    @override
    def run(self):
        while self._running:
            self._node.wait(self._cycle_time)

            while True:
                sample = self._depth_service.receive()
                if sample is None:
                    break
                print("recieved depth")

            while True:
                sample = self._rgb_sub.receive()
                if sample is None:
                    break
                print("recieved rgb")


    def stop(self):
        self._running = False


main = SimCameraSubscriber()
main.start()