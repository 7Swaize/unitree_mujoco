#pragma once

#include <cstdint>
#include "iceoryx/constants.hpp"


struct RGBFrame {
    static constexpr const char* IOX2_TYPE_NAME = "RGBFrame";

    uint32_t width = FRAME_WIDTH;
    uint32_t height = FRAME_HEIGHT;
    
    uint8_t data[FRAME_BUFFER_ELEMENTS_RGB];
};
