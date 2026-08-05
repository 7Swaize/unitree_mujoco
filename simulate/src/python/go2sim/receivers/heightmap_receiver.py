import threading
import numpy as np
import iceoryx2 as iox2

from iceoryx_interfaces.qos import HeightmapQoS
from iceoryx_interfaces.heightmap_data import HeightmapHeader_, HeightmapData_

DATA_BUFFER_ELEMENTS_DEFAULT = np.zeros(99)

class HeightmapReceiver(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True)
        iox2.set_log_level_from_env_or(iox2.LogLevel.Error)
        self._node = iox2.NodeBuilder.new() \
                        .signal_handling_mode(iox2.SignalHandlingMode.Disabled) \
                        .create(iox2.ServiceType.Ipc)

        self._service = self._node.service_builder(iox2.ServiceName.new(HeightmapQoS.TOPIC_HEIGHTMAP)) \
                            .publish_subscribe(HeightmapData_) \
                            .user_header(HeightmapHeader_) \
                            .open_or_create()

        self._sub = self._service.subscriber_builder().create()
        self._cycle_time = iox2.Duration.from_millis(50) # 20 hz polling
        self._shutdown_event: threading.Event = threading.Event()

        self._latest_z_normal: np.ndarray = DATA_BUFFER_ELEMENTS_DEFAULT

    def run(self) -> None:
        while not self._shutdown_event.is_set():
            self._node.wait(self._cycle_time)

            while True:
                sample = self._sub.receive()
                if sample is None:
                    break

                self._latest_z_normal = np.asarray(sample.payload().contents.data, dtype=np.float32)

    def latest_z_normal(self) -> np.ndarray:
        z = self._latest_z_normal
        return z - z.min()

    def shutdown(self) -> None:
        self._shutdown_event.set()