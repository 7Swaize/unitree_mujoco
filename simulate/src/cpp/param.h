#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <filesystem>

#include "utils/yaml_utils.hpp"

namespace param {

inline struct SimulationConfig {
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    void load_from_yaml(const std::string& filename, const std::filesystem::path& proj_dir) {
        auto cfg = YAML::LoadFile(filename);
        
        robot = utils::YamlRequireField<std::string>(cfg, "robot");
        robot_scene = proj_dir / utils::YamlRequireField<std::string>(cfg, "robot_scene");
        domain_id = utils::YamlRequireField<int>(cfg, "domain_id");
        interface = utils::YamlRequireField<std::string>(cfg, "interface");
        use_joystick = utils::YamlRequireField<int>(cfg, "use_joystick");
        joystick_type = utils::YamlRequireField<std::string>(cfg, "joystick_type");
        joystick_device = utils::YamlRequireField<std::string>(cfg, "joystick_device");
        joystick_bits = utils::YamlRequireField<int>(cfg, "joystick_bits");
        print_scene_information = utils::YamlRequireField<int>(cfg, "print_scene_information");
        enable_elastic_band = utils::YamlRequireField<int>(cfg, "enable_elastic_band");
    }
} config;

} // namespace param