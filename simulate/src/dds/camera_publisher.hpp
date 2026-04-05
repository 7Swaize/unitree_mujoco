#pragma once
 
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "simulate.h"
 
#include <atomic>
#include <cstring>
#include <filesystem>
#include <memory>
#include <thread>
#include <vector>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "utils/simd.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include "idl/ImageData.hpp"
 
 
struct DDSPublisherConfig {
    int domain_id;
    std::string interface;
    std::string topic;
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
                    const DDSPublisherConfig& dds_cfg);
 
    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator =(const CameraPublisher&) = delete;
 
    void start();
    void stop();
 
private:
    mjModel* model_;
    mjData* data_;
    CameraConfig cfg_;
    GLFWwindow* offscreen_window_;
    
    std::atomic<bool> running_;
    std::thread thread_;
 
    sim_msgs::ImageData msg_;
    std::unique_ptr<unitree::robot::ChannelPublisher<sim_msgs::ImageData>> publisher_;
 
    void init_msg();
    void publish(const std::vector<unsigned char>& rgb_buf, const std::vector<float>& depth_buf);
 
    class GLFWRenderHandler {
    public:
        GLFWRenderHandler(CameraPublisher* outer);
 
        void operator()();
 
        ~GLFWRenderHandler();
 
    private:
        CameraPublisher* outer_;
 
        mjvScene scn_;
        mjrContext con_;
 
        void renderLoop();
        void depth_transform_hyperbolic_to_linear(std::vector<float>& depth_buf) const;
    };
};