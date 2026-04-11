#pragma once 
#include <string_view>

namespace iceoryx_interfaces::camera {
    inline constexpr const char* kTopicSimCameraDepth = "rt/sim/depth";
    inline constexpr const char* kTopicSimCameraRgb   = "rt/sim/rgb";

    inline constexpr int kMaxPublishers = 1;
    inline constexpr int kMaxSubscribers = 1;
    inline constexpr int kSubscriberMaxBufferSize = 3;
    inline constexpr int kSubscriberMaxBorrowedSamples = 2;
    inline constexpr int kHistorySize = 1;
}