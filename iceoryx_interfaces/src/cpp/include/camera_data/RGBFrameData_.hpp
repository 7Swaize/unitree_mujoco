#pragma once

#include <cstdint>
#include "constants.hpp"


namespace iceoryx_interfaces::camera {    
    struct RGBFrameData_ {
        static constexpr const char* IOX2_TYPE_NAME = "RGBFrameData_";

        uint32_t width = kFrameWidth;
        uint32_t height = kFrameHeight;
        
        uint8_t data[kFrameBufferElementsRgb];
    };
}
