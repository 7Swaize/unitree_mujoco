#pragma once

#include <cstdint>
#include "../constants.hpp"


namespace iceoryx_interfaces {

struct DepthFrameData_ {
    static constexpr const char* IOX2_TYPE_NAME = "DepthFrameData_";

    uint32_t width = FRAME_WIDTH;
    uint32_t height = FRAME_HEIGHT;

    float depth_min;
    float depth_max;

    float data[FRAME_BUFFER_ELEMENTS_DEPTH];
};

}