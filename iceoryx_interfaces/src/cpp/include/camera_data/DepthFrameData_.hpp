#pragma once

#include <cstdint>
#include "constants.hpp"


namespace iceoryx_interfaces::camera {
    struct DepthFrameData_ {
        static constexpr const char* IOX2_TYPE_NAME = "DepthFrameData_";

        uint32_t width = kFrameWidth;
        uint32_t height = kFrameHeight;

        float depth_min;
        float depth_max;

        uint16_t data[kFrameBufferElementsDepth];
    };
}