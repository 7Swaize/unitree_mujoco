#pragma once

#include <cstdint>

namespace iceoryx_interfaces::heightmap {
    struct HeightmapHeader_ {
        static constexpr const char* IOX2_TYPE_NAME = "HeightmapHeader_";

        uint32_t num_heightscans;
        uint32_t num_widthscans;
        float dist_x;
        float dist_y;

        // For vis
        float base_x;
        float base_y;
        float yaw;

    };
} // namespace iceoryx_interfaces::heightmap
