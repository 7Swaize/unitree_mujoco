import ctypes
import threading
import numpy as np
import iceoryx2 as iox2

from typing import Optional
from iceoryx_interfaces.qos import HeightmapQoS
from iceoryx_interfaces.heightmap_data import HeightmapHeader_


class HeightmapReceiver(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True)
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
        self._node = iox2.NodeBuilder.new() \
                        .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
                        .create(iox2.ServiceType.Ipc)

        self._service = self._node.service_builder(iox2.ServiceName.new(HeightmapQoS.TOPIC_HEIGHTMAP)) \
                            .publish_subscribe(iox2.Slice[ctypes.c_float]) \
                            .user_header(HeightmapHeader_) \
                            .open_or_create()

        self._sub = self._service.subscriber_builder().create()
        self._cycle_time = iox2.Duration.from_millis(50) # 20 hz polling
        self._shutdown_event: threading.Event = threading.Event()

        self._latest_z_normal: Optional[np.ndarray] = None
        

    def run(self) -> None:
        while not self._shutdown_event.is_set():
            self._node.wait(self._cycle_time)

            while True:
                sample = self._sub.receive()
                if sample is None:
                    break

                extent_x = int(sample.user_header().grid_extent_x)
                extent_y = int(sample.user_header().grid_extent_y)
                data_ptr = ctypes.cast(sample.payload().as_ptr(), ctypes.POINTER(ctypes.c_float))
                itemsize = np.dtype(np.float32).itemsize

                self._latest_z_normal = np.ndarray(
                    shape=(extent_x, extent_y),
                    dtype=np.float32,
                    buffer=(ctypes.c_float * (extent_x * extent_y)).from_address(ctypes.addressof(data_ptr.contents)),
                    strides=(itemsize, extent_y * itemsize),
                ).copy(order='C')


    def latest_z_normal(self) -> np.ndarray:
        return self._latest_z_normal


    def shutdown(self) -> None:
        self._shutdown_event.set()