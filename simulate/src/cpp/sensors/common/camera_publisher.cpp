#include "camera_publisher.hpp"

using namespace iox2;
using namespace iceoryx_interfaces::camera;

// CameraConfig

void CameraConfig::load(const std::filesystem::path& path) {
    YAML::Node cfg = YAML::LoadFile(path.string());
    
    far_clip = cfg["far_clip"].as<float>(far_clip);
    near_clip = cfg["near_clip"].as<float>(near_clip);
    publish_fps = cfg["publish_fps"].as<double>(publish_fps);
}


// CameraPublisher

CameraPublisher::CameraPublisher(mjModel* model,
                                 mjData* data,
                                 GLFWwindow* share_window,
                                 const CameraConfig& cam_cfg,
                                 const DDSPublisherConfig& dds_cfg,
                                 mujoco::SimulateMutex& sim_mutex)
    : model_(model),
      data_(data), 
      cfg_(cam_cfg), 
      sim_mutex_(sim_mutex),
      iox2_node_(NodeBuilder().signal_handling_mode(SignalHandlingMode::Disabled).create<ServiceType::Ipc>().value()),
      depth_service_(iox2_node_.service_builder(ServiceName::create(kTopicSimCameraDepth).value())
                         .publish_subscribe<DepthFrameData_>()
                         .max_publishers(kMaxPublishers)
                         .max_subscribers(kMaxSubscribers)
                         .subscriber_max_buffer_size(kSubscriberMaxBufferSize)
                         .subscriber_max_borrowed_samples(kSubscriberMaxBorrowedSamples)
                         .history_size(kHistorySize)
                         .open_or_create()
                         .value()),
      rgb_service_(iox2_node_.service_builder(ServiceName::create(kTopicSimCameraRgb).value())
                       .publish_subscribe<RGBFrameData_>()
                       .max_publishers(kMaxPublishers)
                       .max_subscribers(kMaxSubscribers)
                       .subscriber_max_buffer_size(kSubscriberMaxBufferSize)
                       .subscriber_max_borrowed_samples(kSubscriberMaxBorrowedSamples)
                       .history_size(kHistorySize)
                       .open_or_create()
                       .value()),
      depth_pub_(depth_service_.publisher_builder().create().value()),
      rgb_pub_(rgb_service_.publisher_builder().create().value())
{   
    // https://github.com/google-deepmind/mujoco/blob/main/sample/record.cc
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
 
    offscreen_window_ = glfwCreateWindow(kFrameWidth, kFrameHeight, "go2_camera_offscreen", nullptr, share_window);
    glfwDefaultWindowHints();
}

CameraPublisher::~CameraPublisher() {
    stop();

    if (offscreen_window_) {
        glfwDestroyWindow(offscreen_window_);
    }
}

void CameraPublisher::start() {
    if (!offscreen_window_ || running_) return;
  
    running_ = true;
    thread_ = std::thread(GLFWRenderHandler{this});
}
 
void CameraPublisher::stop() {
    running_ = false;
 
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CameraPublisher::publish_depth(uint16_t* data) {
    auto sample = depth_pub_.loan_uninit().value();
    new (&sample.payload_mut()) DepthFrameData_{};
    
    auto& payload = sample.payload_mut();
    payload.depth_min = cfg_.near_clip;
    payload.depth_max = cfg_.far_clip;
    std::memcpy(payload.data, data, kFrameBufferElementsDepth * sizeof(uint16_t));

#ifndef __INTELLISENSE__
    auto initialized = assume_init(std::move(sample));
    send(std::move(initialized)).value();
#endif
}

void CameraPublisher::publish_rgb(unsigned char* data) {
    auto sample = rgb_pub_.loan_uninit().value();
    new (&sample.payload_mut()) RGBFrameData_{};

    auto& payload = sample.payload_mut();
    std::memcpy(payload.data, reinterpret_cast<uint8_t*>(data), kFrameBufferElementsRgb * sizeof(uint8_t));

#ifndef __INTELLISENSE__
    auto initialized = assume_init(std::move(sample));
    send(std::move(initialized)).value();
#endif
}


// CameraPublisher::GLFWRenderHandler

CameraPublisher::GLFWRenderHandler::GLFWRenderHandler(CameraPublisher* outer)
    : outer_(outer) {}
 
void CameraPublisher::GLFWRenderHandler::operator()() {
    try {
        renderLoop();
    } catch (const std::exception& e) {
        std::cerr << "[Simulator] Exception in render loop: " << e.what() << '\n';
    }
}

void CameraPublisher::GLFWRenderHandler::renderLoop() {
    // To my understanding two threads do not share the same rendering buffer (and current context in this situation).
    // Therefore, we only need to direct rendering to the off screen buffer ONCE. And only make the context current ONCE.
    glfwMakeContextCurrent(outer_->offscreen_window_);
 
    mjvScene scn;
    mjrContext con;
    mjvCamera cam;
    mjvOption opt;
 
    mjv_defaultScene(&scn);
    mjv_makeScene(outer_->model_, &scn, mujoco::Simulate::kMaxGeom); // 100000 geom seems too eager?
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjr_makeContext(outer_->model_, &con, mjFONTSCALE_50);
    mjr_setBuffer(mjFB_OFFSCREEN, &con);

    struct RenderCleanup {
        mjvScene* s;
        mjrContext* c;

        ~RenderCleanup() { 
            mjv_freeScene(s); 
            mjr_freeContext(c); 
            glfwMakeContextCurrent(nullptr); 
        }
    } cleanup{&scn, &con};
 
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = mj_name2id(outer_->model_, mjOBJ_CAMERA, "Internal Camera");

    // TODO: This changes clipping for all cams. Find a way to only modify this cam. I put a note about this in the yaml.
    outer_->model_->vis.map.znear = outer_->cfg_.near_clip;
    outer_->model_->vis.map.zfar = outer_->cfg_.far_clip;
 
    mjrRect viewport = {0, 0, kFrameWidth, kFrameHeight};
    
    std::vector<unsigned char> rgb_buf(kFrameBufferElementsRgb);
    std::vector<float, aligned_allocator<float, SIMD_ALIGNMENT>> depth_buf(kFrameBufferElementsDepth);
    std::vector<uint16_t, aligned_allocator<uint16_t, SIMD_ALIGNMENT>> depth_buf_ret(kFrameBufferElementsDepth);

    {
        assert(is_aligned(reinterpret_cast<std::size_t>(depth_buf.data()), SIMD_ALIGNMENT));
        assert(is_aligned(reinterpret_cast<std::size_t>(depth_buf_ret.data()), SIMD_ALIGNMENT));
    }

    auto frame_duration = std::chrono::duration<double>(1.0 / outer_->cfg_.publish_fps);
    auto next_time = std::chrono::steady_clock::now();

    while (outer_->running_) {
        auto now = std::chrono::steady_clock::now();
        if (now < next_time) {
            std::this_thread::sleep_until(next_time);
            continue;
        }

        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_duration);

        // shared data with main sim -> lock
        {
            mujoco::MutexLock lock(outer_->sim_mutex_);
            mjv_updateScene(outer_->model_, outer_->data_, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        }

        mjr_render(viewport, &scn, &con);
        mjr_readPixels(rgb_buf.data(), depth_buf.data(), viewport, &con);

        depth_transform_hyperbolic_to_linear(depth_buf.data(), depth_buf_ret.data(), depth_buf.size());
        outer_->publish_depth(depth_buf_ret.data());
        outer_->publish_rgb(rgb_buf.data());
    }
}

void CameraPublisher::GLFWRenderHandler::depth_transform_hyperbolic_to_linear(float* in, uint16_t* out, const size_t size) {
    // see: https://github.com/openai/mujoco-py/issues/520#issuecomment-1254452252
    // see: https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer/6657284#6657284
    // avx: https://stackoverflow.com/questions/66260651/mm256-fmadd-ps-is-slower-than-mm256-mul-ps-mm256-add-ps

    simd::transform(in, out, size, simd::operations::ToLinDistMap{outer_->cfg_.near_clip, outer_->cfg_.far_clip});
}