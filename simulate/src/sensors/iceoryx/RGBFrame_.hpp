#pragma once

#include <cstdint>
#include "iox2/bb/static_string.hpp"
#include "iox2/bb/static_vector.hpp"
#include "iox2/iceoryx2.hpp"

#include "constants.hpp"


namespace ipc_msg {

struct RGBFrame_ {
    static constexpr const char* IOX2_TYPE_NAME = "RGBFrame_";

    uint32_t width = FRAME_WIDTH;
    uint32_t height = FRAME_HEIGHT;
    
    iox2::bb::StaticVector<float, FRAME_BUFFER_ELEMENTS_RGB> data;
};

} // ipc_msg