#pragma once

#include <cstdint>

namespace iceoryx_interfaces::lidar {
    struct LidarHeader_ {
        static constexpr const char* IOX2_TYPE_NAME = "LidarHeader_";

        uint32_t rows;
        uint32_t cols;
        int64_t stamp_ns;
    };
}