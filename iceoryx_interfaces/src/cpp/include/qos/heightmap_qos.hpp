#pragma once
#include <string_view>

namespace iceoryx_interfaces::heightmap {
    inline constexpr const char* kHeightmapTopicName = "sim/heightmap";

    inline constexpr uint64_t kPublishAllocationInitialSizeHint = 300;
}