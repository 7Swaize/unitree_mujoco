#include "lidar_sensor.hpp"


void LidarSensor::Scan(const mjModel* m, mjData* d, const double dt) {
    if (dt <= 0) return; 

    std::size_t n_rays = static_cast<std::size_t>(std::round(config_.points_per_second * dt));
    if (n_rays == 0) n_rays = 1;
    if (n_rays > lidar_data::kTotalVecs) n_rays = lidar_data::kTotalVecs;

    Eigen::Map<const Matrix3x3RMm> R(d->site_xmat + 9 * config_.site_id);
    Eigen::Map<const Vec3m> origin(d->site_xpos + 3 * config_.site_id);
    Matrix3Xm world = TransformLocalToWorldSpace(R, pattern_cursor_, n_rays);

    utils::ResizeLazy(ray_geomid_, n_rays);
    utils::ResizeLazy(ray_dist_, n_rays);

    mj_multiRay(m, d, origin.data(), world.data(),
                /*geomgroup=*/ nullptr, /*flg_static=*/ 1, config_.exclude_body_id,
                ray_geomid_.data(), ray_dist_.data(), n_rays, config_.max_range);

    for (std::size_t i = 0; i < n_rays; ++i) {
        const std::size_t idx = (pattern_cursor_ + i) % lidar_data::kTotalVecs;
        const mjtNum dist = ray_dist_[i];

        const bool hit = (dist >= 0.0) && (dist >= config_.min_range);
        if (!hit) {
            accumulated_[idx].valid = false;
            continue;
        }

        auto dir = world.col(i);

        LidarPoint p;
        p.x = static_cast<float>(origin[0] + dist * dir[0]);
        p.y = static_cast<float>(origin[1] + dist * dir[1]);
        p.z = static_cast<float>(origin[2] + dist * dir[2]);
        p.distance = static_cast<float>(dist);
        p.geom_id = ray_geomid_[i];
        p.valid = true;

        accumulated_[idx] = p;
    }

    pattern_cursor_ = (pattern_cursor_ + n_rays) % lidar_data::kTotalVecs;
}

LidarSensor::Matrix3Xm LidarSensor::TransformLocalToWorldSpace(
    const Eigen::Map<const Matrix3x3RMm> R,
    const std::size_t start,
    const std::size_t n_rays)
    const
{
    const auto& pattern = lidar_data::ScanPatternDirectionsMap();
    const std::size_t first_chunk = std::min(n_rays, lidar_data::kTotalVecs - start);

    if (first_chunk == n_rays) {
        return R * pattern(Eigen::all, Eigen::seq(start, start + n_rays - 1));
    }

    const std::size_t remaining = n_rays - first_chunk;

    Matrix3Xm local(3, n_rays);
    local.leftCols(first_chunk) = pattern(Eigen::all, Eigen::seq(start, start + first_chunk - 1));
    local.rightCols(remaining) = pattern(Eigen::all, Eigen::seq(0, remaining - 1));

    return R * local;
}


void LidarConfig::Load(const std::filesystem::path& path) {
    YAML::Node cfg = YAML::LoadFile(path.string());

    min_range = utils::YamlRequireField<float>(cfg, "min_range");
    max_range = utils::YamlRequireField<float>(cfg, "max_range");
    publish_hz = utils::YamlRequireField<int>(cfg, "publish_hz");
    points_per_second = utils::YamlRequireField<int>(cfg, "points_per_second");

    std::string site_name = utils::YamlRequireField<std::string>(cfg, "site_name");
    site_id = mj_name2id(model_, mjOBJ_SITE, site_name.c_str());

    std::string exclude_body = utils::YamlRequireField<std::string>(cfg, "exclude_body");
    exclude_body_id = mj_name2id(model_, mjOBJ_BODY, exclude_body.c_str());
}
