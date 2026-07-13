#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <thread>

#include "sensors/common/lidar_sensor.hpp"


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
          sensor_(LidarSensor{config})
    { }

    void Run();

private:
    const mjModel* model_;
    mjData* data_;
    mujoco::Simulate* sim_;
    mujoco::SimulateMutex& sim_mutex_;

    const LidarConfig config_;
    LidarSensor sensor_;

    void LoopInternal();
    void PublishCloud(double stamp_sec);
};