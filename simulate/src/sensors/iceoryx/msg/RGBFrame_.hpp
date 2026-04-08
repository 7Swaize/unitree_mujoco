#pragma once

#include <cstdint>
#include "iceoryx/constants.hpp"


namespace ipc_msg {

struct RGBFrame_ {
    static constexpr const char* IOX2_TYPE_NAME = "RGBFrame_";

    uint32_t width = FRAME_WIDTH;
    uint32_t height = FRAME_HEIGHT;
    
    std::array<uint8_t, FRAME_BUFFER_ELEMENTS_RGB> data;
};

} // ipc_msg