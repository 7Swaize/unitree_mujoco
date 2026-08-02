#pragma once

#include <mujoco/mujoco.h>
#include "simulate.h"

#include <vector>
#include <Eigen/Core>

#include "utils/container_utils.hpp"
#include "utils/simd.hpp"
#include "sensors/data/generated_scan_pattern.hpp"

class LidarSensor {
public:
    using Vec3m = Eigen::Vector<mjtNum, 3>;
    using Matrix3xXm = Eigen::Matrix<mjtNum, 3, Eigen::Dynamic>;
    using Matrix3x3RMm = Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>;
    using LidarScanView = Eigen::Block<const Matrix3xXm, 3, Eigen::Dynamic, true>;

public:
    explicit LidarSensor(const mjModel* m)
        : points_world_(3, lidar_data::kTotalVecs),
          world_dirs_scratch_(3, lidar_data::kTotalVecs)
    {
        site_id_ = mj_name2id(m, mjOBJ_SITE, kSiteStr);
        exclude_body_id_ = mj_name2id(m, mjOBJ_BODY, kExcludeBodyStr);
    }

    [[nodiscard]]
    FORCE_INLINE const LidarScanView LatestScan() const noexcept {
        return points_world_.leftCols(valid_count_);
    }
    
    void Scan(const mjModel* m, mjData* d, const double dt);

private:
    static constexpr float kResizeLazyMultiplier = 1.5f;
    static constexpr float kMinRange = 0.10;
    static constexpr float kMaxRange = 40;
    static constexpr float kPointsPerSecond = 200000;
    static constexpr const char* kSiteStr = "mid360_lidar_site";
    static constexpr const char* kExcludeBodyStr = "base_link";
    
    int site_id_;
    int exclude_body_id_;

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