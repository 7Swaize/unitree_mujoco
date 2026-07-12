#include "lidar_sensor.hpp"

namespace {
    using Vec3m = Eigen::Vector<mjtNum, 3>;
    using Matrix3Xm = Eigen::Matrix<mjtNum, 3, Eigen::Dynamic>;
}

void LidarSensor::scan(const mjModel* m, mjData* d, double dt) {
    if (dt <= 0) return;

    std::size_t n_rays = static_cast<std::size_t>(std::round(config_.points_per_second * dt));
    if (n_rays == 0) n_rays = 1;
    if (n_rays > lidar_data::TotalVecs) n_rays = lidar_data::TotalVecs;

    Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> R(d->site_xmat + 9 * config_.site_id);
    Eigen::Map<const Vec3m> origin(d->site_xpos + 3 * config_.site_id);

    const auto& pattern = lidar_data::ScanPatternDirectionsMap();
    Matrix3Xm world = R * pattern(Eigen::all, Eigen::seq(pattern_cursor_, n_rays));

    utils::resize_lazy(ray_geomid_, n_rays);
    utils::resize_lazy(ray_dist_, n_rays);

    mj_multiRay(m, d, origin.data(), world.data(),
                /*geomgroup=*/nullptr, /*flg_static=*/1, config_.exclude_body_id,
                ray_geomid_.data(), ray_dist_.data(), n_rays, config_.max_range
    );

    for (std::size_t i = 0; i < n_rays; ++i) {
        const std::size_t idx = (pattern_cursor_ + i) % lidar_data::TotalVecs;
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
}


#ifndef __INTELLISENSE__
void LidarConfig::load(const std::filesystem::path& path) {
    YAML::Node cfg = YAML::LoadFile(path.string());

    min_range = cfg["min_range"].as<float>(min_range);
    max_range = cfg["max_range"].as<float>(max_range);
    publish_hz = cfg["publish_hz"].as<int>(publish_hz);
    points_per_second = cfg["points_per_second"].as<int>(points_per_second);

    const char* site_name = cfg["site_name"].as<const char*>();
    site_id = mj_name2id(model_, mjOBJ_BODY, site_name);

    const char* exclude_body = cfg["exclude_body"].as<const char*>();
    exclude_body_id = mj_name2id(model_, mjOBJ_BODY, exclude_body);
}
#endif