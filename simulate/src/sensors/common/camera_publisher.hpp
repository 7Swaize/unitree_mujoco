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
#include <yaml-cpp/yaml.h>

#include "utils/simd.hpp"
#include "utils/aligned_allocator.hpp"

#include "iox2/iceoryx2.hpp"
#include "iceoryx/msg/DepthFrame_.hpp"
#include "iceoryx/msg/RGBFrame_.hpp"
#include "iceoryx/constants.hpp"


struct DDSPublisherConfig {
    const int domain_id;
    const std::string interface;
};


struct CameraConfig {
    int res_x = 620;
    int res_y = 480;
    int crop_left = 8;
    float far_clip = 2.0f;
    float near_clip = 0.175f;
    double publish_fps = 0.1;
 
    void load(const std::filesystem::path &path);
};
 
 
class CameraPublisher {
public:
    CameraPublisher(mjModel* model,
                    mjData* data, 
                    GLFWwindow* share_window,
                    const CameraConfig& cam_cfg,
                    const DDSPublisherConfig& dds_cfg,
                    mujoco::SimulateMutex& sim_mutex);
 
    ~CameraPublisher();
    
    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator =(const CameraPublisher&) = delete;
 
    void start();
    void stop();
 
private:
    mjModel* model_;
    mjData* data_;
    CameraConfig cfg_;
    GLFWwindow* offscreen_window_ = nullptr;
    mujoco::SimulateMutex& sim_mutex_;
    
    std::atomic<bool> running_{false};
    std::thread thread_;
 
    iox2::Node<iox2::ServiceType::Ipc> iox2_node_;
    iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, ipc_msgs::DepthFrame_, void> depth_service_;
    iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, ipc_msgs::RGBFrame_, void> rgb_service_;
    iox2::Publisher<iox2::ServiceType::Ipc, ipc_msgs::DepthFrame_, void> depth_pub_;
    iox2::Publisher<iox2::ServiceType::Ipc, ipc_msgs::RGBFrame_, void> rgb_pub_;
 
    void publish_depth(float* data, const size_t size);
    void publish_rgb(unsigned char* data, const size_t size);
 
    class GLFWRenderHandler {
    public:
        explicit GLFWRenderHandler(CameraPublisher* outer);
        void operator()();
 
    private:
        CameraPublisher* outer_;
 
        void renderLoop();
        void depth_transform_hyperbolic_to_linear(float* data, const size_t size);
    };
};