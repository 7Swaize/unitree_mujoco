#pragma once
#include <array>
#include <cstddef>

namespace lidar_data {
    inline constexpr std::size_t TotalElements = 2400000;
    inline constexpr std::size_t TotalRows = 800000;
    alignas(64) extern const std::array<float, 2400000> Mid360ScanPatternData;
}
