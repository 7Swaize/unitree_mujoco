#pragma once

#include "iox2/iceoryx2.hpp"
#include "lidar_data/LidarHeader_.hpp"
#include "camera_data/FrameData_.hpp"
#include "qos/lidar_qos.hpp"
#include "qos/camera_qos.hpp"


namespace ipc {
    namespace lidar = iceoryx_interfaces::lidar;
    namespace camera = iceoryx_interfaces::camera;

    using Node = iox2::Node<iox2::ServiceType::Ipc>;

    template <typename TPayload, typename THeader = void>
    using PubSubFactory = iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, TPayload, THeader>;

    template <typename TPayload, typename THeader = void>
    using PubSubPublisher = iox2::Publisher<iox2::ServiceType::Ipc, TPayload, THeader>;

    inline Node MakeNode() {
        return iox2::NodeBuilder()
            .signal_handling_mode(iox2::SignalHandlingMode::Disabled)
            .create<iox2::ServiceType::Ipc>()
            .value();
    }

    template <typename TPayload, typename THeader = void>
    PubSubFactory<TPayload, THeader> MakeService(Node& node, const char* topic_name) {
        if constexpr (std::is_void_v<THeader>) {
            return node.service_builder(iox2::ServiceName::create(topic_name).value())
                .template publish_subscribe<TPayload>()
                .open_or_create()
                .value();
        } else {
            return node.service_builder(iox2::ServiceName::create(topic_name).value())
                .template publish_subscribe<TPayload>()
                .template user_header<THeader>()
                .open_or_create()
                .value();
        }
    }

    template <typename TPayload, typename THeader = void>
    PubSubPublisher<TPayload, THeader> MakePublisher(PubSubFactory<TPayload, THeader>& service) {
        return service.publisher_builder().create().value();
    }



    template <typename TRequest, typename TResponse, typename TRequestHeader = void, typename TResponseHeader = void>
    using RRFactory =
        iox2::PortFactoryRequestResponse<iox2::ServiceType::Ipc, TRequest, TRequestHeader, TResponse, TResponseHeader>;

    template <typename TRequest, typename TResponse, typename TRequestHeader = void, typename TResponseHeader = void>
    using RRServer =
        iox2::Server<iox2::ServiceType::Ipc, TRequest, TRequestHeader, TResponse, TResponseHeader>;


    template <typename TRequest, typename TResponse, typename TRequestHeader = void, typename TResponseHeader = void>
    RRFactory<TRequest, TResponse, TRequestHeader, TResponseHeader>
    MakeRequestResponseFactory(Node& node, const char* service_name) {
        auto builder = node.service_builder(iox2::ServiceName::create(service_name).value())
            .template request_response<TRequest, TResponse>();

        auto with_req_header = []<typename B>(B&& b) {
            if constexpr (!std::is_same_v<TRequestHeader, void>) {
                return std::forward<B>(b).template request_user_header<TRequestHeader>();
            } else {
                return std::forward<B>(b);
            }
        }(std::move(builder));

        auto with_resp_header = []<typename B>(B&& b) {
            if constexpr (!std::is_same_v<TResponseHeader, void>) {
                return std::forward<B>(b).template response_user_header<TResponseHeader>();
            } else {
                return std::forward<B>(b);
            }
        }(std::move(builder));

        return std::move(with_resp_header).open_or_create().value();
    }

    template <typename TRequest,
            typename TResponse,
            typename TRequestHeader,
            typename TResponseHeader,
            iox2::AllocationStrategy AllocationStrategy>
    RRServer<TRequest, TResponse, TRequestHeader, TResponseHeader>
    MakeRequestResponseServerDynamicData(
        RRFactory<TRequest, TResponse, TRequestHeader, TResponseHeader>& service,
        const uint64_t init_slice_len)
    {
        return service
            .server_builder()
            .initial_max_slice_len(init_slice_len)
            .allocation_strategy(AllocationStrategy)
            .create()
            .value();
    }

} // namespace ipc