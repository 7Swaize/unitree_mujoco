#include "heightmap_sensor.hpp"


mjtNum HeightmapSensor::CastGroundRay(const mjModel* m, mjData* d, double world_x, double world_y) const { 
    mjtNum start[3] = {world_x, world_y, d->xpos[3 * exclude_body_id_ + 2]};
    const mjtNum dir[3] = {0.0, 0.0, -1.0};

    mjtNum dist = mj_ray(m, d, start, dir, /*geomgroup=*/nullptr, /*flg_static=*/1, /*bodyexclude=*/-1, nullptr);

    if (dist < 0) {
        return std::numeric_limits<mjtNum>::quiet_NaN();
    }

    return start[2] - dist;
}

void HeightmapSensor::Scan(const mjModel* m, mjData* d) {
    base_x_ = d->xpos[3 * exclude_body_id_ + 0];
    base_y_ = d->xpos[3 * exclude_body_id_ + 1];

    const mjtNum* R = d->xmat + 9 * exclude_body_id_;
    yaw_ = std::atan2(R[3], R[0]);

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double c_h = (kNumHeightscans- 1) / 2.0;
    const double c_w = (kNumWidthscans - 1) / 2.0;


    for (uint32_t idx_h = 0; idx_h < kNumHeightscans; ++idx_h) {
        const double p = c_h - static_cast<double>(idx_h);
        const double rx = p * kDistX;

        for (uint32_t idx_w = 0; idx_w < kNumWidthscans; ++idx_w) {
            const double k = c_w - static_cast<double>(idx_w);
            const double ry = k * kDistY;

            const double world_dx = rx * cos_yaw - ry * sin_yaw;
            const double world_dy = rx * sin_yaw + ry * cos_yaw;

            const std::size_t idx = idx_h * kNumWidthscans + idx_w;
            const mjtNum z = CastGroundRay(m, d, base_x_ + world_dx, base_y_ + world_dy);

            if (!std::isnan(z)) {
                z_buffer_[idx] = static_cast<float>(z);
            }
        }
    }
}