#pragma once
#include <yaml-cpp/yaml.h>

namespace utils {

template<typename T>
[[nodiscard]] T yaml_require_field(const YAML::Node& cfg, const std::string& key) {
    if (!cfg[key]) {
        throw std::runtime_error("Config: Missing required field '" + key + "'.");
    }

    return cfg[key].as<T>();
}

} // utils