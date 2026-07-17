#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <thread>
#include <typeinfo>

#include "sensors/common/lidar_sensor.hpp"
#include "utils/ipc.hpp"
#include "utils/container_utils.hpp"
#include "utils/debug.hpp"


class LidarPublisher {
private:
    using Request = uint32_t;
    using Response = iox2::bb::Slice<float>;
    using RequestHeader = void;
    using ResponseHeader = ipc::lidar::LidarHeader_;
    using ActiveRequestOptional = iox2::bb::stl::Optional<
        iox2::ActiveRequest<iox2::ServiceType::Ipc, Request, RequestHeader, Response, ResponseHeader>
    >;

public:
    LidarPublisher(mjModel* model,
                   mjData* data,
                   const LidarConfig& config,
                   mujoco::Simulate* sim,
                   mujoco::SimulateMutex& sim_mutex)
        : model_(model),
        data_(data),
            config_(config),
            sim_(sim),
            sim_mutex_(sim_mutex),
            sensor_(LidarSensor{config}),
            iox2_node_(ipc::MakeNode()),
            lidar_rr_factory_ipc_(ipc::MakeRequestResponseFactory<Request, Response, RequestHeader, ResponseHeader>(
                iox2_node_, ipc::lidar::kLidarTopicName
            )),
            lidar_rr_server_ipc_(ipc::MakeRequestResponseServerDynamicData<
                Request, Response, RequestHeader, ResponseHeader, iox2::AllocationStrategy::PowerOfTwo
            >(lidar_rr_factory_ipc_, ipc::lidar::kResponseAllocationInitialSizeHint))
        { }


    void Run();

private:
    const mjModel* model_;
    mjData* data_;
    mujoco::Simulate* sim_;
    mujoco::SimulateMutex& sim_mutex_;

    const LidarConfig config_;
    LidarSensor sensor_;

    ipc::Node iox2_node_;
    ipc::RRFactory<Request, Response, RequestHeader, ResponseHeader> lidar_rr_factory_ipc_;
    ipc::RRServer<Request, Response, RequestHeader, ResponseHeader> lidar_rr_server_ipc_;
    ActiveRequestOptional active_request_ipc_;

    std::vector<float> ipc_conversion_scratch_;

    static constexpr float kIPCAwaitRequestNodeCycleTime = 0.02;
    uint32_t publish_hz_ = 0;

    void LoopInternalIPCAwaitRequest();
    void LoopInternalIPCPublish();

    template <typename T = mjtNum>
    void PublishCloud(int64_t stamp_ns);
};