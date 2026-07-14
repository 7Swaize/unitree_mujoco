#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <thread>

#include "sensors/common/lidar_sensor.hpp"
#include "utils/ipc.hpp"


class LidarPublisher {
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
        lidar_service_(ipc::MakeService<ipc::LidarData, ipc::LidarHeader>(
            iox2_node_,
            ipc::lidar::kLidarTopicName,
            {
                .max_publishers = ipc::lidar::kMaxPublishers,
                .max_subscribers = ipc::lidar::kMaxSubscribers,
                .subscriber_max_buffer_size = ipc::lidar::kSubscriberMaxBufferSize,
                .subscriber_max_borrowed_samples = ipc::lidar::kSubscriberMaxBorrowedSamples,
                .history_size = ipc::lidar::kHistorySize
            })
        ),
        lidar_pub_(ipc::MakePublisherDynamicData<ipc::LidarData, ipc::LidarHeader,iox2::AllocationStrategy::PowerOfTwo>(
            lidar_service_,
            kAllocationInitialSizeHint)
        )
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
    ipc::PubSubFactory<ipc::LidarData, ipc::LidarHeader> lidar_service_;
    ipc::Publisher<ipc::LidarData, ipc::LidarHeader> lidar_pub_;

    static constexpr uint64_t kAllocationInitialSizeHint = 20000;

    void LoopInternal();
    void PublishCloud(double stamp_sec);
};