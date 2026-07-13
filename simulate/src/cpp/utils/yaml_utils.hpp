#pragma once

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <string>

namespace utils {

template <typename T>
[[nodiscard]] T YamlRequireField(const YAML::Node& cfg, const std::string& key) {
    if (!cfg[key]) {
        throw std::runtime_error("Config: Missing required field '" + key + "'.");
    }
    
    return cfg[key].as<T>();
}

}  // namespace utils