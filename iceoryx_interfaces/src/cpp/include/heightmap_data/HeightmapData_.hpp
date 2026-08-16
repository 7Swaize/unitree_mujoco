#pragma once

#include <cstdint>

#include "constants.hpp"

namespace iceoryx_interfaces::heightmap {
    struct HeightmapData_ {
        static constexpr const char* IOX2_TYPE_NAME = "HeightmapData_";

        float data[kBufferSize];
    };
} // namespace iceoryx_interfaces::heightmap
