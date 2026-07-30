#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <thread>
#include <typeinfo>

#include "sensors/common/heightmap_sensor.hpp"
#include "utils/ipc.hpp"

class HeightmapPublisher {
private:
    using Payload = iox2::bb::Slice<float>;
    using PayloadHeader = ipc::heightmap::HeightmapHeader_;

public:
    HeightmapPublisher(mjModel* model,
                   mjData* data,
                   const HeightmapConfig& config,
                   mujoco::Simulate* sim,
                   mujoco::SimulateMutex& sim_mutex)
        : model_(model),
        data_(data),
        config_(config),
        sim_(sim),
        sim_mutex_(sim_mutex),
        sensor_(HeightmapConfig{config}),
        iox2_node_(ipc::MakeNode()),
        heightmap_factory_ipc_(ipc::MakePubSubService<Payload, PayloadHeader>(iox2_node_, ipc::heightmap::kHeightmapTopicName)),
        heightmap_pub_ipc(ipc::MakePublisherDynamicData<Payload, PayloadHeader, iox2::AllocationStrategy::PowerOfTwo>(
            heightmap_factory_ipc_, ipc::heightmap::kPublishAllocationInitialSizeHint
        ))
    { }

    void Run();

private:
    const mjModel* model_;
    mjData* data_;
    mujoco::Simulate* sim_;
    mujoco::SimulateMutex& sim_mutex_;

    const HeightmapConfig config_;
    HeightmapSensor sensor_;

    // 5 hz
    static constexpr float kIPCPublishNodeCycleTime = 0.20;

    ipc::Node iox2_node_;
    ipc::PubSubFactory<Payload, PayloadHeader> heightmap_factory_ipc_;
    ipc::PubSubPublisher<Payload, PayloadHeader> heightmap_pub_ipc;

    void LoopInternalIPCPublish();
    void PublishHeightmap();
};