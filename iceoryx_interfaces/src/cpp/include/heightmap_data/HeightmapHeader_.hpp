#pragma once

#include <cstdint>

namespace iceoryx_interfaces::heightmap {
    struct HeightmapHeader_ {
        static constexpr const char* IOX2_TYPE_NAME = "HeightmapHeader_";

        uint32_t grid_extent_x;
        uint32_t grid_extent_y;
    };
} // namespace iceoryx_interfaces::heightmap
