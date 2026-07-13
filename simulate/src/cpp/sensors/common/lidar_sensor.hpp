#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <filesystem>
#include <bitset>
#include <vector>
#include <span>
#include <Eigen/Core>

#include "utils/container_utils.hpp"
#include "utils/yaml_utils.hpp"
#include "utils/simd.hpp"
#include "sensors/data/generated_scan_pattern.hpp"

class LidarConfig {
public:
    float min_range = 0.10f;
    float max_range = 40.0f;

    int publish_hz = 10;
    int points_per_second = 200000;

    int site_id = -1;
    int exclude_body_id = -1;

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

    struct LidarScanView {
        Eigen::Block<const Matrix3xXm, 3, Eigen::Dynamic, true> points;
        const std::bitset<lidar_data::kTotalVecs>& valid;
        const std::size_t n_rays;
    };

    explicit LidarSensor(const LidarConfig& config)
        : config_(config),
          points_world_(3, lidar_data::kTotalVecs)
    { }

    FORCE_INLINE LidarScanView LatestScan() const noexcept {
        return LidarScanView {
            points_world_.leftCols(n_rays_),
            valid_,
            n_rays_
        };
    }
    
    void Scan(const mjModel* m, mjData* data, const double dt);

private:
    static constexpr float kResizeLazyMultiplier = 1.5f;
    const LidarConfig config_;

    std::size_t pattern_cursor_ = 0;
    std::size_t n_rays_ = 0;
    std::vector<int> ray_geomid_stratch_;
    std::vector<mjtNum> ray_dist_scratch_;

    Matrix3xXm points_world_; 
    std::bitset<lidar_data::kTotalVecs> valid_;

    Matrix3xXm TransformLocalToWorldSpace(
        const Eigen::Map<const Matrix3x3RMm> R,
        const std::size_t start,
        const std::size_t n_rays)
        const;
};