#pragma once

#include <string_view>

namespace iceoryx_interfaces::lidar {
    inline constexpr const char* kLidarTopicName = "control/lidar_decoded";

    inline constexpr int kMaxPublishers = 1;
    inline constexpr int kMaxSubscribers = 1;
    inline constexpr int kSubscriberMaxBufferSize = 3;
    inline constexpr int kSubscriberMaxBorrowedSamples = 2;
    inline constexpr int kHistorySize = 1;
} // namespace iceoryx_interfaces::lidar
