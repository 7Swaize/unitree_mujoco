#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <typeinfo>

#if defined(__GNUC__) || defined(__clang__)
    #include <cxxabi.h>
    #define UTILS_HAVE_CXXABI 1
#endif

namespace utils {

template <typename T>
[[nodiscard]] std::string GetTypeName(const T& obj) {
#if defined(UTILS_HAVE_CXXABI)
    int status = 0;
    const std::unique_ptr<char, void (*)(void*)> demangled(
        abi::__cxa_demangle(typeid(obj).name(), nullptr, nullptr, &status),
        std::free);

    return status == 0 ? std::string(demangled.get()) : std::string(typeid(obj).name());
#else
    return std::string(typeid(obj).name());
#endif
}

}  // namespace utils