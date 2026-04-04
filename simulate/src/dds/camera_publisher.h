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

#include <unitree/robot/channel/channel_publisher.hpp>
#include "idl/ImageData.hpp"


struct CameraConfig {
    int res_x = 620;
    int res_y = 480;
    int crop_left = 8;
    float far_clip = 2.0f;
    float near_clip = 0.175f;
    double publish_fps = 0.1;

    void load(const std::filesystem::path &path) {
        YAML::Node cfg = YAML::LoadFile(path.string());
        
        res_x = cfg["res_x"].as<int>(res_x);
        res_y = cfg["res_y"].as<int>(res_y);
        far_clip = cfg["far_clip"].as<float>(far_clip);
        near_clip = cfg["near_clip"].as<float>(near_clip);
        publish_fps = cfg["publish_fps"].as<double>(publish_fps);
    }
};


class CameraPublisher {
public:
    CameraPublisher(mjModel* model,
                    mjData* data, 
                    GLFWwindow* share_window,
                    const CameraConfig& cfg,
                    const std::string& topic) // use "rt/sim/camera"
        : model_(model), data_(data), offscreen_window_(nullptr), cfg_(cfg), running_(false)
    {
        // https://github.com/google-deepmind/mujoco/blob/main/sample/record.cc
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);

        offscreen_window_ = glfwCreateWindow(cfg_.res_x, cfg_.res_y, "go2_camera_offscreen", nullptr, share_window);
        glfwDefaultWindowHints();

        publisher_ = std::make_unique<unitree::robot::ChannelPublisher<sim_msgs::ImageData>>(topic);
        publisher_->InitChannel();
    }

    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator =(const CameraPublisher&) = delete;

    void start() {
        if (!offscreen_window_ || !publisher_) {
            return;
        }

        running_ = true;
        thread_ = std::thread(GLFWRenderHandler{this});
    }

    void stop() {
        running_ = false;

        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    mjModel* model_;
    mjData* data_;
    CameraConfig cfg_;
    GLFWwindow* offscreen_window_;
    
    std::atomic<bool> running_;
    std::thread thread_;
    std::unique_ptr<unitree::robot::ChannelPublisher<sim_msgs::ImageData>> publisher_;

    void publish(const std::vector<unsigned char>& rgb_buf, const std::vector<float>& depth_buf) {
        // TODO
    }

    class GLFWRenderHandler {
    public:
        GLFWRenderHandler(CameraPublisher* outer) : outer_(outer) {}

        void operator()() {
            renderLoop();
        }

        ~GLFWRenderHandler() {
            mjv_freeScene(&scn_);
            mjr_freeContext(&con_);
            glfwMakeContextCurrent(nullptr);
        }

    private:
        CameraPublisher* outer_;

        mjvScene scn_;
        mjrContext con_;

        void renderLoop() {
            glfwMakeContextCurrent(outer_->offscreen_window_);

            mjvCamera cam;
            mjvOption opt;

            mjv_defaultScene(&scn_);
            mjv_makeScene(outer_->model_, &scn_, mujoco::Simulate::kMaxGeom); // 100000 geom seems too eager?
            mjv_defaultCamera(&cam);
            mjv_defaultOption(&opt);
            mjr_defaultContext(&con_);
            mjr_makeContext(outer_->model_, &con_, mjFONTSCALE_50);

            int cam_id = mj_name2id(outer_->model_, mjOBJ_CAMERA, "Internal Camera");
            if (cam_id < 0) {
                throw std::runtime_error("[SIMULATOR] Internal camera not found in model");
            }

            cam.type = mjCAMERA_FIXED;
            cam.fixedcamid = cam_id;

            outer_->model_->vis.map.znear = outer_->cfg_.near_clip / outer_->cfg_.far_clip;
            outer_->model_->vis.map.zfar = 1.0;

            mjrRect viewport = {0, 0, outer_->cfg_.res_x, outer_->cfg_.res_y};
            
            std::vector<unsigned char> rgb_buf(3 * outer_->cfg_.res_x * outer_->cfg_.res_y);
            std::vector<float> depth_buf(outer_->cfg_.res_x * outer_->cfg_.res_y);

            auto next_time = std::chrono::steady_clock::now();

            while (outer_->running_) {
                auto now = std::chrono::steady_clock::now();
                if (now < next_time) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                next_time += std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(1.0 / outer_->cfg_.publish_fps)
                );

                // https://github.com/google-deepmind/mujoco/issues/583
                // lock data because of possible race condition with physics sim
                mjv_updateScene(outer_->model_, outer_->data_, &opt, nullptr, &cam, mjCAT_ALL, &scn_);
                mjr_render(viewport, &scn_, &con_);
                mjr_readPixels(rgb_buf.data(), depth_buf.data(), viewport, &con_);
            }
        }

        void filter() {
            // TODO
        }
    };
};