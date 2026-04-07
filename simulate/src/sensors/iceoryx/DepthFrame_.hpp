#pragma once

#include <cstdint>
#include "iox2/bb/static_string.hpp"
#include "iox2/bb/static_vector.hpp"
#include "iox2/iceoryx2.hpp"

#include "constants.hpp"

// see: https://github.com/eclipse-iceoryx/iceoryx2/blob/main/examples/cxx/complex_data_types/src/complex_data_types.cpp

namespace ipc_msg {

struct DepthFrame_ {
    static constexpr const char* IOX2_TYPE_NAME = "DepthFrame_";

    uint32_t width = FRAME_WIDTH;
    uint32_t height = FRAME_HEIGHT;

    float depth_min;
    float depth_max;

    iox2::bb::StaticVector<float, FRAME_BUFFER_ELEMENTS_DEPTH> data;
};

} // ipc_msg