#pragma once

namespace iceoryx_interfaces::camera {
    inline constexpr int kFrameWidth = 640;
    inline constexpr int kFrameHeight = 480;
    inline constexpr int kRgbBufferElementCount = kFrameWidth * kFrameHeight * 3;
    inline constexpr int kDepthBufferElementCount = kFrameWidth * kFrameHeight;
}