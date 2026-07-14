#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "simulate.h"

#include <atomic>
#include <cassert>
#include <cstring>
#include <filesystem>
#include <memory>
#include <thread>
#include <vector>
#include <stdexcept>

#include "utils/yaml_utils.hpp"
#include "utils/simd.hpp"
#include "utils/aligned_allocator.hpp"
#include "utils/ipc.hpp"


struct CameraConfig {
    int res_x = 620;
    int res_y = 480;
    int crop_left = 8;
    float far_clip = 2.0f;
    float near_clip = 0.175f;
    int publish_fps = 60;

    void Load(const std::filesystem::path& path);
};


class CameraPublisher {
public:
    CameraPublisher(mjModel* model,
                    mjData* data,
                    GLFWwindow* share_window,
                    const CameraConfig& cam_cfg,
                    mujoco::Simulate* sim,
                    mujoco::SimulateMutex& sim_mutex);

    ~CameraPublisher();

    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator=(const CameraPublisher&) = delete;

    void Run();

private:
    mjModel* model_;
    mjData* data_;
    const CameraConfig cfg_;
    GLFWwindow* offscreen_window_ = nullptr;

    mujoco::Simulate* sim_;
    mujoco::SimulateMutex& sim_mutex_;

    ipc::Node iox2_node_;
    ipc::PubSubFactory<ipc::FrameData> camera_service_;
    ipc::Publisher<ipc::FrameData> camera_pub_;

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