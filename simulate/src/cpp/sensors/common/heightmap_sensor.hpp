#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <filesystem>
#include <vector>
#include <span>
#include <Eigen/Core>

#include "utils/yaml_utils.hpp"
#include "utils/simd.hpp"


class HeightmapConfig {
public:
    uint32_t grid_extent_x;
    uint32_t grid_extent_y;
    uint32_t rays_per_unit;
    int site_id;
    int exclude_body_id;

    explicit HeightmapConfig(const mjModel* model) : model_(model) { }
    
    void Load(const std::filesystem::path& path);

private:
    const mjModel* model_;
};

struct HeightmapPoint {
    float x;
    float y;
    float z;
};

class HeightmapSensor {
public:
    using Vec3m = Eigen::Vector<mjtNum, 3>;
    using Matrix3x3RMm = Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>;

    explicit HeightmapSensor(const HeightmapConfig& config)
        : config_(config),
        nx_(std::max<uint32_t>(1u, static_cast<uint32_t>(std::round(config.grid_extent_x * config.rays_per_unit)) + 1)),
        ny_(std::max<uint32_t>(1u, static_cast<uint32_t>(std::round(config.grid_extent_y * config.rays_per_unit)) + 1)),
        half_extent_x_(config.grid_extent_x * 0.5),
        half_extent_y_(config.grid_extent_y * 0.5),
        buffer_(nx_ * ny_)
    { }

    [[nodiscard]]
    FORCE_INLINE std::span<const HeightmapPoint> HeightmapView() const {
        return std::span<const HeightmapPoint>(buffer_.data(), valid_count_);
    }
    
    void Scan(const mjModel* m, mjData* data);

private:
    const HeightmapConfig config_;
    const uint32_t nx_;
    const uint32_t ny_;
    const double half_extent_x_;
    const double half_extent_y_;

    std::vector<HeightmapPoint> buffer_;
    std::size_t valid_count_;
};