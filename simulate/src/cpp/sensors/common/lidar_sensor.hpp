#pragma once
#include <mujoco/mujoco.h>
#include "simulate.h"

#include <filesystem>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "utils/container_utils.hpp"
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

    explicit LidarConfig(const mjModel* model) : model_(model) { }

    void load(const std::filesystem::path& path);

private:
    const mjModel* model_;
};


struct LidarPoint {
    float x = 0.f, y = 0.f, z = 0.f;
    float distance = 0.f;
    int geom_id = -1;
    bool valid = false;
};


class LidarSensor {
public:
    explicit LidarSensor(const LidarConfig& config): config_(config) {
        accumulated_.reserve(lidar_data::TotalVecs);
    }

    void scan(const mjModel* m, mjData* data, double dt);
    
    FORCE_INLINE const std::vector<LidarPoint>& latest_points() const { return latest_points_; }
    FORCE_INLINE const std::vector<LidarPoint>& accumulated_cloud() const { return accumulated_; }

private:
    const LidarConfig config_;

    std::size_t pattern_cursor_ = 0;

    std::vector<LidarPoint> accumulated_;
    std::vector<LidarPoint> latest_points_;

    std::vector<int> ray_geomid_;
    std::vector<mjtNum> ray_dist_;
};