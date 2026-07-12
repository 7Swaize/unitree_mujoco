#include "lidar_publisher.hpp"

void LidarPublisher::Run() {
    try {
        LoopInternal();
    } catch (const std::exception& e) {
        std::cerr << "[Simulator] Exception in lidar loop: " << e.what() << '\n';
    }
}

void LidarPublisher::LoopInternal() {
    auto frame_duration = std::chrono::duration<double>(1.0 / config_.publish_hz);
    auto next_time = std::chrono::steady_clock::now();
    auto last_tick = std::chrono::steady_clock::now();

    while (!sim_->exitrequest.load(std::memory_order_acquire)) {
        auto now = std::chrono::steady_clock::now();
        if (now < next_time) {
            std::this_thread::sleep_until(next_time);
            continue;
        }
        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_duration);

        double wall_dt = std::chrono::duration<double>(now - last_tick).count();
        last_tick = now;

        double stamp_sec = 0.0;
        {
            mujoco::MutexLock lock(sim_mutex_);
            sensor_.Scan(model_, data_, wall_dt);
            stamp_sec = static_cast<double>(data_->time);
        }

        PublishCloud(stamp_sec);
    }
}

void LidarPublisher::PublishCloud(double stamp_sec) {
    const auto& points = sensor_.LatestPoints();
}