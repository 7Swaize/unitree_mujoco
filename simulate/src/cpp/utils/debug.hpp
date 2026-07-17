#include <iostream>
#include <cxxabi.h>

namespace utils {

template <typename T>
[[nodiscard]] std::string GetTypeName(const T& obj) {
    int status = 0;
    std::unique_ptr<char, void(*)(void*)> res(
        abi::__cxa_demangle(typeid(obj).name(), nullptr, nullptr, &status),
        std::free
    );

    return status == 0 ? res.get() : typeid(obj).name();
}

} // namspace utils