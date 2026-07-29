#include "camera_publisher.hpp"

using namespace ipc::camera;

void CameraConfig::Load(const std::filesystem::path& path) {
    YAML::Node cfg = YAML::LoadFile(path.string());

    far_clip = utils::YamlRequireField<float>(cfg, "far_clip");
    near_clip = utils::YamlRequireField<float>(cfg, "near_clip");
    publish_fps = utils::YamlRequireField<int>(cfg, "publish_fps");
}

CameraPublisher::CameraPublisher(mjModel* model,
                                 mjData* data,
                                 GLFWwindow* share_window,
                                 const CameraConfig& cam_cfg,
                                 mujoco::Simulate* sim,
                                 mujoco::SimulateMutex& sim_mutex)
    : model_(model),
    data_(data),
    cfg_(cam_cfg),
    sim_(sim),
    sim_mutex_(sim_mutex),
    iox2_node_(ipc::MakeNode()),
    camera_service_(ipc::MakeService<FrameData>(iox2_node_, kCameraTopicName)),
    camera_pub_(ipc::MakePublisher<FrameData>(camera_service_))
{
    // Reference: https://github.com/google-deepmind/mujoco/blob/main/sample/record.cc
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);

    offscreen_window_ = glfwCreateWindow(kFrameWidth, kFrameHeight, "go2_camera_offscreen", nullptr, share_window);
    glfwDefaultWindowHints();
}

CameraPublisher::~CameraPublisher() {
    if (offscreen_window_) {
        glfwDestroyWindow(offscreen_window_);
    }
}

void CameraPublisher::Run() {
    GLFWRenderHandler{this}();
}

void CameraPublisher::PublishFrames(unsigned char* rgb_data, uint16_t* depth_data) {
    auto sample = camera_pub_.loan_uninit().value();
    new (&sample.payload_mut()) FrameData_{};

    auto& payload = sample.payload_mut();

    payload.depth_min = cfg_.near_clip;
    payload.depth_max = cfg_.far_clip;

    std::memcpy(payload.rgb_data, reinterpret_cast<uint8_t*>(rgb_data), kRgbBufferElementCount * sizeof(uint8_t));
    std::memcpy(payload.depth_data, depth_data, kDepthBufferElementCount * sizeof(uint16_t));

    auto initialized = assume_init(std::move(sample));
    send(std::move(initialized)).value();
}

CameraPublisher::GLFWRenderHandler::GLFWRenderHandler(CameraPublisher* outer) : outer_(outer) {}

void CameraPublisher::GLFWRenderHandler::operator()() {
    try {
        RenderLoop();
    } catch (const std::exception& e) {
        std::cerr << "[Simulator] Exception in render loop: " << e.what() << '\n';
    }
}

void CameraPublisher::GLFWRenderHandler::RenderLoop() {
    // To my understanding two threads do not share the same rendering buffer (and current context in this situation).
    // Therefore, we only need to direct rendering to the off screen buffer ONCE. And only make the context current ONCE.
    glfwMakeContextCurrent(outer_->offscreen_window_);

    mjvScene scn;
    mjrContext con;
    mjvCamera cam;
    mjvOption opt;

    mjv_defaultScene(&scn);
    mjv_makeScene(outer_->model_, &scn, mujoco::Simulate::kMaxGeom);
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

    // Note: This changes clipping for all cameras. Consider using per-camera clipping if available.
    outer_->model_->vis.map.znear = outer_->cfg_.near_clip;
    outer_->model_->vis.map.zfar = outer_->cfg_.far_clip;

    mjrRect viewport = {0, 0, kFrameWidth, kFrameHeight};

    std::vector<unsigned char> rgb_buf(kRgbBufferElementCount);
    std::vector<float, utils::AlignedAllocator<float, SIMD_ALIGNMENT>> depth_buf(kDepthBufferElementCount);
    std::vector<uint16_t, utils::AlignedAllocator<uint16_t, SIMD_ALIGNMENT>> depth_buf_ret(kDepthBufferElementCount);

    {
        assert(utils::IsAligned(reinterpret_cast<std::size_t>(depth_buf.data()), SIMD_ALIGNMENT));
        assert(utils::IsAligned(reinterpret_cast<std::size_t>(depth_buf_ret.data()), SIMD_ALIGNMENT));
    }

    auto frame_duration = std::chrono::duration<double>(1.0 / outer_->cfg_.publish_fps);
    auto next_time = std::chrono::steady_clock::now();

    while (!outer_->sim_->exitrequest.load(std::memory_order_acquire)) {
        auto now = std::chrono::steady_clock::now();
        if (now < next_time) {
            std::this_thread::sleep_until(next_time);
            continue;
        }

        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_duration);

        // Shared data with main simulator - acquire lock
        {
            mujoco::MutexLock lock(outer_->sim_mutex_);
            mjv_updateScene(outer_->model_, outer_->data_, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        }

        mjr_render(viewport, &scn, &con);
        mjr_readPixels(rgb_buf.data(), depth_buf.data(), viewport, &con);

        DepthTransformHyperbolicToLinear(depth_buf.data(), depth_buf_ret.data(), depth_buf.size());
        outer_->PublishFrames(rgb_buf.data(), depth_buf_ret.data());
    }
}

void CameraPublisher::GLFWRenderHandler::DepthTransformHyperbolicToLinear(float* in, uint16_t* out, const std::size_t size) {
    // References:
    // - https://github.com/openai/mujoco-py/issues/520#issuecomment-1254452252
    // - https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
    // - SIMD optimization: https://stackoverflow.com/questions/66260651/mm256-fmadd-ps-is-slower

    simd::Transform(in, out, simd::operations::ToLinDistMap{outer_->cfg_.near_clip, outer_->cfg_.far_clip}, size);
}