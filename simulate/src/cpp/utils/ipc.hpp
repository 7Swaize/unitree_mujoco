#pragma once

#include "iox2/iceoryx2.hpp"
#include "lidar_data/LidarHeader_.hpp"
#include "camera_data/FrameData_.hpp"
#include "qos/lidar_qos.hpp"
#include "qos/camera_qos.hpp"


namespace ipc {
    namespace lidar = iceoryx_interfaces::lidar;
    namespace camera = iceoryx_interfaces::camera;

    using LidarHeader = iceoryx_interfaces::lidar::LidarHeader_;
    using LidarData = iox2::bb::Slice<double>;
    using FrameData = iceoryx_interfaces::camera::FrameData_;
    using Node = iox2::Node<iox2::ServiceType::Ipc>;

    template <typename TPayload, typename THeader = void>
    using PubSubFactory = iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, TPayload, THeader>;

    template <typename TPayload, typename THeader = void>
    using Publisher = iox2::Publisher<iox2::ServiceType::Ipc, TPayload, THeader>;

    struct ServiceQoS {
        int max_publishers;
        int max_subscribers;
        int subscriber_max_buffer_size;
        int subscriber_max_borrowed_samples;
        int history_size;
    };

    inline Node MakeNode() {
        return iox2::NodeBuilder()
            .signal_handling_mode(iox2::SignalHandlingMode::Disabled)
            .create<iox2::ServiceType::Ipc>()
            .value();
    }

    template <typename TPayload, typename THeader = void>
    PubSubFactory<TPayload, THeader> MakeService(Node& node, const char* topic_name, const ServiceQoS& qos) {
        if constexpr (std::is_void_v<THeader>) {
            return node.service_builder(iox2::ServiceName::create(topic_name).value())
                .template publish_subscribe<TPayload>()
                .max_publishers(qos.max_publishers)
                .max_subscribers(qos.max_subscribers)
                .subscriber_max_buffer_size(qos.subscriber_max_buffer_size)
                .subscriber_max_borrowed_samples(qos.subscriber_max_borrowed_samples)
                .history_size(qos.history_size)
                .open_or_create()
                .value();
        } else {
            return node.service_builder(iox2::ServiceName::create(topic_name).value())
                .template publish_subscribe<TPayload>()
                .template user_header<THeader>()
                .max_publishers(qos.max_publishers)
                .max_subscribers(qos.max_subscribers)
                .subscriber_max_buffer_size(qos.subscriber_max_buffer_size)
                .subscriber_max_borrowed_samples(qos.subscriber_max_borrowed_samples)
                .history_size(qos.history_size)
                .open_or_create()
                .value();
        }
    }

    template <typename TPayload, typename THeader = void>
    Publisher<TPayload, THeader> MakePublisher(PubSubFactory<TPayload, THeader>& service) {
        return service.publisher_builder().create().value();
    }

    template <typename TPayload, typename THeader, iox2::AllocationStrategy AllocationStrategy>
    Publisher<TPayload, THeader> MakePublisherDynamicData(
        PubSubFactory<TPayload, THeader>& service,
        const uint64_t init_slice_len)
    {
        return service
            .publisher_builder()
            .initial_max_slice_len(init_slice_len)
            .allocation_strategy(AllocationStrategy)
            .create()
            .value();
    }

} // namespace ipc
