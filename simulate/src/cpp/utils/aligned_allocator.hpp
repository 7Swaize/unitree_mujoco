#pragma once

#include <cstddef>
#include <new>
#include <cstdlib>

namespace utils {

template <typename T, std::size_t Alignment>
struct AlignedAllocator {
    using value_type = T;

    AlignedAllocator() noexcept = default;

    template <class U>
    constexpr AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept {}

    // https://stackoverflow.com/questions/66891368/template-parametric-type-allocator-in-c
    template <class U>
    struct rebind {
        using other = AlignedAllocator<U, Alignment>;
    };

    T* allocate(std::size_t n) {
        void* ptr = nullptr;
#if defined(_MSC_VER)
        ptr = _aligned_malloc(n * sizeof(T), Alignment);
        if (!ptr) {
            throw std::bad_alloc();
        }
#else
        if (posix_memalign(&ptr, Alignment, n * sizeof(T)) != 0) {
            throw std::bad_alloc();
        }
#endif
        return reinterpret_cast<T*>(ptr);
    }

    void deallocate(T* p, std::size_t) noexcept {
#if defined(_MSC_VER)
        _aligned_free(p);
#else
        free(p);
#endif
    }
};

inline bool IsAligned(const std::size_t value, const std::size_t alignment) noexcept {
    return (value & (alignment - 1)) == 0;
}

}  // namespace utils
