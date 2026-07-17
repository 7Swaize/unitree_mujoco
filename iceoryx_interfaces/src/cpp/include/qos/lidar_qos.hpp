#pragma once

#include <string_view>

namespace iceoryx_interfaces::lidar {
    inline constexpr const char* kLidarTopicName = "control/lidar_decoded";

    inline constexpr uint64_t kResponseAllocationInitialSizeHint = 20000 * sizeof(double);
} // namespace iceoryx_interfaces::lidar
