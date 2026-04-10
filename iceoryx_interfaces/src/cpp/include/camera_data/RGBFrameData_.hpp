#pragma once

#include <cstdint>
#include "../constants.hpp"


namespace iceoryx_interfaces {
    
struct RGBFrameData_ {
    static constexpr const char* IOX2_TYPE_NAME = "RGBFrameData_";

    uint32_t width = FRAME_WIDTH;
    uint32_t height = FRAME_HEIGHT;
    
    uint8_t data[FRAME_BUFFER_ELEMENTS_RGB];
};

}
