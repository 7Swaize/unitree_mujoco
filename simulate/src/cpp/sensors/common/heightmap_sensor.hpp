#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <vector>
#include <span>

#include "utils/simd.hpp"


class HeightmapSensor {
public:
    static constexpr uint32_t kNumHeightscans = 11;
    static constexpr uint32_t kNumWidthscans = 9;
    static constexpr uint32_t kNRays = kNumHeightscans * kNumWidthscans;
    static constexpr float kDistX = 0.1;
    static constexpr float kDistY = 0.1;

    explicit HeightmapSensor(const mjModel* m) : z_buffer_(kNRays) {
        exclude_body_id_ = mj_name2id(m, mjOBJ_BODY, kExcludeBodyStr);
    }

    [[nodiscard]]
    FORCE_INLINE std::span<const float> ZView() const {
        return std::span<const float>(z_buffer_.data(), kNRays);
    }

    [[nodiscard]] FORCE_INLINE double BaseX() const { return base_x_; }
    [[nodiscard]] FORCE_INLINE double BaseY() const { return base_y_; }
    [[nodiscard]] FORCE_INLINE double Yaw() const { return yaw_; }

    void Scan(const mjModel* m, mjData* data);

private:
    static constexpr const char* kExcludeBodyStr = "base_link";
    
    int exclude_body_id_;

    double base_x_;
    double base_y_;
    double yaw_;

    std::vector<float> z_buffer_;

    [[nodiscard]] mjtNum CastGroundRay(const mjModel* m, mjData* d, double world_x, double world_y) const;
};