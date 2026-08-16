#pragma once

#include <yaml-cpp/yaml.h>
#include <concepts>
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

template <std::totally_ordered T>
inline void YamlRequireLess(const T& lower, const T& upper, const std::string& lower_name, const std::string& upper_name) {
    if (!(lower < upper)) {
        throw std::runtime_error("Config: Expected '" + lower_name + "' < '" + upper_name + "'.");
    }
}

template <typename T> requires std::integral<T> && std::totally_ordered<T>
inline void YamlRequirePositive(const T& value, const std::string& name) {
    if (!(value > T{0})) {
        throw std::runtime_error("Config: Expected '" + name + "' to be positive.");
    }
}

}  // namespace utils