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
        }

        auto wall_now = std::chrono::system_clock::now();
        int64_t stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_now.time_since_epoch()).count();

        PublishCloud(stamp_ns);
    }
}

template <typename T = mjtNum>
void LidarPublisher::PublishCloud(int64_t stamp_ns) {
    const LidarSensor::LidarScanView& view = sensor_.LatestScan();
    const uint64_t N = static_cast<uint64_t>(view.size());

    auto sample = lidar_pub_.loan_slice_uninit(N).value();
    sample.user_header_mut().cols = static_cast<uint32_t>(view.cols());
    sample.user_header_mut().rows = static_cast<uint32_t>(view.rows());
    sample.user_header_mut().stamp_ns = stamp_ns;

    if constexpr (std::is_same_v<T, double>) {
        iox2::bb::ImmutableSlice<double> src_slice(view.data(), N);
        auto initialized_sample = sample.write_from_slice(src_slice);
        send(std::move(initialized_sample)).value();
    } 
    else {
        utils::ResizeLazy(conversion_scratch_, N);
        std::transform(view.data(), view.data() + N,
                        conversion_scratch_.begin(),
                        [](T v) { return static_cast<double>(v); });

        iox2::bb::ImmutableSlice<double> src_slice(conversion_scratch_.data(), N);
        auto initialized_sample = sample.write_from_slice(src_slice);
        send(std::move(initialized_sample)).value();
    }
}