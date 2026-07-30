#include "lidar_sensor.hpp"


void LidarSensor::Scan(const mjModel* m, mjData* d, const double dt) {
    if (dt <= 0) return; 

    std::size_t n_rays = static_cast<std::size_t>(std::round(config_.points_per_second * dt));
    if (n_rays == 0) n_rays = 1;
    if (n_rays > lidar_data::kTotalVecs) n_rays = lidar_data::kTotalVecs;

    Eigen::Map<const Matrix3x3RMm> R(d->site_xmat + 9 * config_.site_id);
    Eigen::Map<const Vec3m> origin(d->site_xpos + 3 * config_.site_id);

    TransformLocalToWorldSpace(R, pattern_cursor_, n_rays);
    const auto world = world_dirs_scratch_.leftCols(n_rays);

    utils::ResizeLazy(ray_geomid_scratch_, n_rays * kResizeLazyMultiplier);
    utils::ResizeLazy(ray_dist_scratch_, n_rays * kResizeLazyMultiplier);

    mj_multiRay(m, d, origin.data(), world.data(),
                /*geomgroup=*/ nullptr, /*flg_static=*/ 1, config_.exclude_body_id,
                ray_geomid_scratch_.data(), ray_dist_scratch_.data(), n_rays, config_.max_range);

    std::size_t j = 0;
    for (std::size_t i = 0; i < n_rays; ++i) {
        const mjtNum dist = ray_dist_scratch_[i];

        if (dist >= config_.min_range && dist <= config_.max_range) {
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



void LidarConfig::Load(const std::filesystem::path& path) {
    YAML::Node cfg = YAML::LoadFile(path.string());

    min_range = utils::YamlRequireField<float>(cfg, "min_range");
    max_range = utils::YamlRequireField<float>(cfg, "max_range");
    points_per_second = utils::YamlRequireField<int>(cfg, "points_per_second");

    utils::YamlRequireLess(min_range, max_range, "min_range", "max_range");
    utils::YamlRequirePositive(points_per_second, "points_per_second");

    std::string site_name = utils::YamlRequireField<std::string>(cfg, "site_name");
    site_id = mj_name2id(model_, mjOBJ_SITE, site_name.c_str());

    std::string exclude_body = utils::YamlRequireField<std::string>(cfg, "exclude_body");
    exclude_body_id = mj_name2id(model_, mjOBJ_BODY, exclude_body.c_str());
}
