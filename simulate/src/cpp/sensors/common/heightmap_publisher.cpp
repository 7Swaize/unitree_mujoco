#include "heightmap_publisher.hpp"

void HeightmapPublisher::Run() {
    try {
        LoopInternalIPCPublish();
    } catch (const std::exception& e) {
        std::cerr << "[Simulator] Exception in heightmap loop: " << e.what() << '\n';
    }
}

void HeightmapPublisher::LoopInternalIPCPublish() {
    auto frame_duration = std::chrono::duration<double>(kIPCPublishNodeCycleTime);
    auto next_time = std::chrono::steady_clock::now();

    while (!sim_->exitrequest.load(std::memory_order_acquire)) {
        auto now = std::chrono::steady_clock::now();
        if (now < next_time) {
            std::this_thread::sleep_until(next_time);
            continue;
        }
        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_duration);

        {
            mujoco::MutexLock lock(sim_mutex_);
            sensor_.Scan(model_, data_);
        }

        PublishHeightmap();
    }
}

void HeightmapPublisher::PublishHeightmap() {
    std::span<const float> view = sensor_.ZView();

    auto sample = heightmap_pub_ipc.loan_uninit().value();
    auto& header = sample.user_header_mut();

    header.num_heightscans = HeightmapSensor::kNumHeightscans;
    header.num_widthscans = HeightmapSensor::kNumWidthscans;
    header.dist_x = HeightmapSensor::kDistX;
    header.dist_y = HeightmapSensor::kDistY;
    header.base_x = sensor_.BaseX();
    header.base_y = sensor_.BaseY();
    header.yaw = sensor_.Yaw();

    new (&sample.payload_mut()) Payload;
    auto& payload = sample.payload_mut();

    std::memcpy(payload.data, view.data(), HeightmapSensor::kNRays * sizeof(float));
    auto initialized_sample = assume_init(std::move(sample));

    send(std::move(initialized_sample)).value();
}