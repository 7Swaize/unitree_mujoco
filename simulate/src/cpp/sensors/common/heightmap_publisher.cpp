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
    std::span<const HeightmapPoint> view = sensor_.HeightmapView();
    std::span<const float> view_flat{reinterpret_cast<const float*>(view.data()), view.size() * 3};

    auto sample = heightmap_pub_ipc.loan_slice_uninit(view_flat.size()).value();
    sample.user_header_mut().grid_extent_x = config_.grid_extent_x;
    sample.user_header_mut().grid_extent_y = config_.grid_extent_y;

    iox2::bb::ImmutableSlice<float> src_slice(view_flat.data(), view_flat.size());
    auto initialized_sample = sample.write_from_slice(src_slice);
    send(std::move(initialized_sample)).value();
}