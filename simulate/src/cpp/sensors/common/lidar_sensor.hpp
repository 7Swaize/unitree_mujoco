#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <filesystem>
#include <vector>
#include <Eigen/Core>

#include "utils/container_utils.hpp"
#include "utils/yaml_utils.hpp"
#include "utils/simd.hpp"
#include "sensors/data/generated_scan_pattern.hpp"

class LidarConfig {
public:
    float min_range;
    float max_range;
    int points_per_second;
    int site_id;
    int exclude_body_id;

    explicit LidarConfig(const mjModel* model) : model_(model) {}

    void Load(const std::filesystem::path& path);

private:
    const mjModel* model_;
};


class LidarSensor {
public:
    using Vec3m = Eigen::Vector<mjtNum, 3>;
    using Matrix3xXm = Eigen::Matrix<mjtNum, 3, Eigen::Dynamic>;
    using Matrix3x3RMm = Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>;
    using LidarScanView = Eigen::Block<const Matrix3xXm, 3, Eigen::Dynamic, true>;

    explicit LidarSensor(const LidarConfig& config)
        : config_(config),
        points_world_(3, lidar_data::kTotalVecs),
        world_dirs_scratch_(3, lidar_data::kTotalVecs) { }

    [[nodiscard]]
    FORCE_INLINE const LidarScanView LatestScan() const noexcept {
        return points_world_.leftCols(valid_count_);
    }
    
    void Scan(const mjModel* m, mjData* d, const double dt);

private:
    static constexpr float kResizeLazyMultiplier = 1.5f;
    const LidarConfig config_;

    std::size_t pattern_cursor_ = 0;
    std::vector<int> ray_geomid_scratch_;
    std::vector<mjtNum> ray_dist_scratch_;

    Matrix3xXm points_world_;
    Matrix3xXm world_dirs_scratch_;
    std::size_t valid_count_ = 0;

    void TransformLocalToWorldSpace(
        const Eigen::Map<const Matrix3x3RMm>& R,
        const std::size_t start,
        const std::size_t n_rays);
};