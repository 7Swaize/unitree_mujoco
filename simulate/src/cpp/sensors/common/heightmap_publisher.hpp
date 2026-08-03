#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <thread>
#include <typeinfo>
#include <cassert>

#include "sensors/common/heightmap_sensor.hpp"
#include "utils/ipc.hpp"

class HeightmapPublisher {
private:
    using Payload = ipc::heightmap::HeightmapData_;
    using PayloadHeader = ipc::heightmap::HeightmapHeader_;

public:
    HeightmapPublisher(mjModel* model,
                       mjData* data,
                       mujoco::Simulate* sim,
                       mujoco::SimulateMutex& sim_mutex)
        : model_(model),
        data_(data),
        sim_(sim),
        sim_mutex_(sim_mutex),
        sensor_(model),
        iox2_node_(ipc::MakeNode()),
        heightmap_factory_ipc_(ipc::MakePubSubService<Payload, PayloadHeader>(iox2_node_, ipc::heightmap::kHeightmapTopicName)),
        heightmap_pub_ipc(ipc::MakePublisher<Payload, PayloadHeader>(heightmap_factory_ipc_))
    { }

    void Run();

private:
    const mjModel* model_;
    mjData* data_;
    mujoco::Simulate* sim_;
    mujoco::SimulateMutex& sim_mutex_;

    HeightmapSensor sensor_;

    // 50 hz. This is what PGTT was trained on, so I don't want to take chances.
    // TODO: See if this rate can be reduced
    static constexpr float kIPCPublishNodeCycleTime = 0.02;

    ipc::Node iox2_node_;
    ipc::PubSubFactory<Payload, PayloadHeader> heightmap_factory_ipc_;
    ipc::PubSubPublisher<Payload, PayloadHeader> heightmap_pub_ipc;

    void LoopInternalIPCPublish();
    void PublishHeightmap();
};