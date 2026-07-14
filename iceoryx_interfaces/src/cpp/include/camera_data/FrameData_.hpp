#pragma once

#include <cstdint>
#include "constants.hpp"


namespace iceoryx_interfaces::camera {
    struct FrameData_ {
        static constexpr const char* IOX2_TYPE_NAME = "FrameData_";

        uint32_t width = kFrameWidth;
        uint32_t height = kFrameHeight;

        float depth_min;
        float depth_max;

        uint8_t rgb_data[kRgbBufferElementCount];
        uint16_t depth_data[kDepthBufferElementCount];
    };
}