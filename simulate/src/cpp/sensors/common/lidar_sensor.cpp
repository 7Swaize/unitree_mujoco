#include "lidar_sensor.hpp"


void LidarSensor::Scan(const mjModel* m, mjData* d, const double dt) {
    if (dt <= 0) return; 

    std::size_t n_rays = static_cast<std::size_t>(std::round(kPointsPerSecond * dt));
    if (n_rays == 0) n_rays = 1;
    if (n_rays > lidar_data::kTotalVecs) n_rays = lidar_data::kTotalVecs;

    Eigen::Map<const Matrix3x3RMm> R(d->site_xmat + 9 * site_id_);
    Eigen::Map<const Vec3m> origin(d->site_xpos + 3 * site_id_);

    TransformLocalToWorldSpace(R, pattern_cursor_, n_rays);
    const auto world = world_dirs_scratch_.leftCols(n_rays);

    utils::ResizeLazy(ray_geomid_scratch_, n_rays * kResizeLazyMultiplier);
    utils::ResizeLazy(ray_dist_scratch_, n_rays * kResizeLazyMultiplier);

    mj_multiRay(m, d, origin.data(), world.data(),
                /*geomgroup=*/ nullptr, /*flg_static=*/ 1, exclude_body_id_,
                ray_geomid_scratch_.data(), ray_dist_scratch_.data(), n_rays, kMaxRange);

    std::size_t j = 0;
    for (std::size_t i = 0; i < n_rays; ++i) {
        const mjtNum dist = ray_dist_scratch_[i];

        if (dist >= kMinRange && dist <= kMaxRange) {
            points_world_.col(j++) = origin + dist * world.col(i);
        }
    }

    valid_count_ = j;
    pattern_cursor_ = (pattern_cursor_ + n_rays) % lidar_data::kTotalVecs;
}


void LidarSensor::TransformLocalToWorldSpace(
    const Eigen::Map<const Matrix3x3RMm>& R,
    const std::size_t start,
    const std::size_t n_rays)
{
    const auto& pattern = lidar_data::ScanPatternDirectionsMap();
    const std::size_t first_chunk = std::min(n_rays, lidar_data::kTotalVecs - start);
    auto dst = world_dirs_scratch_.leftCols(n_rays);

    if (first_chunk == n_rays) {
        dst.noalias() = R * pattern(Eigen::all, Eigen::seq(start, start + n_rays - 1));
        return;
    }
 
    const std::size_t remaining = n_rays - first_chunk;
    dst.leftCols(first_chunk).noalias() = R * pattern(Eigen::all, Eigen::seq(start, start + first_chunk - 1));
    dst.rightCols(remaining).noalias() = R * pattern(Eigen::all, Eigen::seq(0, remaining - 1));
}
