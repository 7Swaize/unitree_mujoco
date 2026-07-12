#pragma once

#include <cstddef>
#include <vector>

namespace utils {

template <typename T>
inline void ResizeLazy(std::vector<T>& vec, const std::size_t size) {
    if (vec.size() >= size) {
        return;
    }
    vec.resize(size);
}

}  // namespace utils
