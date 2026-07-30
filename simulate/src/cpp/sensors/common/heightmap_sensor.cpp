#include "heightmap_sensor.hpp"

void HeightmapSensor::Scan(const mjModel* m, mjData* d) {
    Eigen::Map<const Matrix3x3RMm> R(d->site_xmat + 9 * config_.site_id);
    Eigen::Map<const Vec3m> origin(d->site_xpos + 3 * config_.site_id);
    const Vec3m z_unit_world_down(0, 0, -1.0);

    valid_count_ = 0;
    
    for (int iy = 0; iy < ny_; ++iy) {
        double ly = (ny_ == 1) ? 0.0 : -half_extent_y_ + (config_.grid_extent_y * iy) / (ny_ - 1);
        for (int ix = 0; ix < nx_; ++ix) {
            double lx = (nx_ == 1) ? 0.0 : -half_extent_x_ + (config_.grid_extent_x * ix) / (nx_ - 1);

            Vec3m local_pt(lx, ly, 0);
            Vec3m world_pt = R * local_pt + origin;

            mjtNum dist = mj_ray(m, d, world_pt.data(), z_unit_world_down.data(),
                                /*geomgroup=*/ nullptr, /*flg_static=*/ 1, config_.exclude_body_id, 
                                /*geomid=*/ nullptr);

            if (dist == -1) {
                continue;
            }

            buffer_[valid_count_++] = {
                .x = static_cast<float>(world_pt.x()),
                .y = static_cast<float>(world_pt.y()),
                .z = static_cast<float>(world_pt.z() - dist)
            };
        }        
    }
}

void HeightmapConfig::Load(const std::filesystem::path& path) {
    YAML::Node cfg = YAML::LoadFile(path.string());

    grid_extent_x = utils::YamlRequireField<uint32_t>(cfg, "grid_extent_x");
    grid_extent_y = utils::YamlRequireField<uint32_t>(cfg, "grid_extent_y");
    rays_per_unit = utils::YamlRequireField<uint32_t>(cfg, "rays_per_unit");

    utils::YamlRequirePositive(grid_extent_x, "grid_extent_x");
    utils::YamlRequirePositive(grid_extent_y, "grid_extent_y");
    utils::YamlRequirePositive(rays_per_unit, "rays_per_unit");

    std::string site_name = utils::YamlRequireField<std::string>(cfg, "site_name");
    site_id = mj_name2id(model_, mjOBJ_SITE, site_name.c_str());

    std::string exclude_body = utils::YamlRequireField<std::string>(cfg, "exclude_body");
    exclude_body_id = mj_name2id(model_, mjOBJ_BODY, exclude_body.c_str());
}