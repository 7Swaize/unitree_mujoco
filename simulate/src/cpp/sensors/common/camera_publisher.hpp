#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "simulate.h"

#include <atomic>
#include <cassert>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>
#include <stdexcept>

#include "utils/simd.hpp"
#include "utils/aligned_allocator.hpp"
#include "utils/ipc.hpp"

class CameraPublisher {
private:
    using FrameData = ipc::camera::FrameData_;

public:
    CameraPublisher(mjModel* model,
                    mjData* data,
                    GLFWwindow* share_window,
                    mujoco::Simulate* sim,
                    mujoco::SimulateMutex& sim_mutex);

    ~CameraPublisher();

    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator=(const CameraPublisher&) = delete;

    void Run();

private:
    static constexpr int kPublishFps = 60;
    static constexpr int kResX = 620;
    static constexpr int kResY = 480;
    static constexpr float kFarClip = 30.0f;
    static constexpr float kNearClip = 0.1f;

    mjModel* model_;
    mjData* data_;
    GLFWwindow* offscreen_window_ = nullptr;

    mujoco::Simulate* sim_;
    mujoco::SimulateMutex& sim_mutex_;

    ipc::Node iox2_node_;
    ipc::PubSubFactory<FrameData> camera_service_;
    ipc::PubSubPublisher<FrameData> camera_pub_;

    void PublishFrames(unsigned char* rgb_data, uint16_t* depth_data);

    class GLFWRenderHandler {
    public:
        explicit GLFWRenderHandler(CameraPublisher* outer);
        void operator()();

    private:
        CameraPublisher* outer_;

        void RenderLoop();
        void DepthTransformHyperbolicToLinear(float* in, uint16_t* out, const std::size_t size);
    };
};