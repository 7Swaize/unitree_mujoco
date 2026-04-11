#pragma once

namespace iceoryx_interfaces::camera {
    inline constexpr int kFrameWidth = 640;
    inline constexpr int kFrameHeight = 480;
    inline constexpr int kFrameBufferElementsRgb = kFrameWidth * kFrameHeight * 3;
    inline constexpr int kFrameBufferElementsDepth = kFrameWidth * kFrameHeight;
}