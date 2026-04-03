#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
 
#include <atomic>
#include <cstring>
#include <filesystem>
#include <memory>
#include <thread>
#include <vector>
 
#include <yaml-cpp/yaml.h>

#include "idl/ImageData.hpp"


struct CameraConfig {
    int width = 80;
    int height = 60;
    int crop_left = 8;
    float far_clip = 2.0f;
    float near_clip = 0.175f;
    double depth_dt = 0.1;

    void load(const std::filesystem::path &path) {
        YAML::Node cfg = YAML::LoadFile(path.string());
        
        width = cfg["width"].as<int>(width);
        height = cfg["height"].as<int>(height);
        crop_left = cfg["crop_left"].as<int>(crop_left);
        far_clip = cfg["far_clip"].as<float>(far_clip);
        near_clip = cfg["near_clip"].as<float>(near_clip);
        depth_dt = cfg["depth_dt"].as<double>(depth_dt);
    }
};


class CameraPublisher {
public:
    CameraPublisher(mjModel* model,
                    mjData* data, 
                    GLFWwindow* share_window,
                    const CameraConfig& cfg,
                    const std::string& topic = "rt/sim/camera")
        : _m(model), _d(data), _offscreenWindow(nullptr), _running(false)
    {
        
    }

private:
    mjModel* _m;
    mjData* _d;
    CameraConfig _cfg;
    GLFWwindow* _offscreenWindow;
    
    std::atomic<bool> _running;
    std::thread _thread;
    std::unique_ptr<unitree::robot::ChannelPublisher<sim_msgs::ImageData>> _publisher;
};