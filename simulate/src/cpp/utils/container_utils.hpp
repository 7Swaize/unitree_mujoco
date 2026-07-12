#pragma once

#include <cstddef>
#include <vector>

namespace utils {

template <typename T>
inline void resize_lazy(std::vector<T>& vec, std::size_t size) {
    if (vec.size() >= size) {
        return;
    }

    vec.resize(size);
}

}  // utils
