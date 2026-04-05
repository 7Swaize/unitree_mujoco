#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <new>
#include <stdexcept>

namespace simd {
#if defined(_M_X64) || defined(__x86_64__) || defined(__i386__)
    #if defined(__AVX2__)
        #define SIMD_AVX2
        #include <immintrin.h>
    #elif defined(__AVX__)
        #define SIMD_AVX
        #include <immintrin.h>
    #elif defined(__SSE2__) || defined(_M_X64) || defined(_M_AMD64)
        #define SIMD_SSE2
        #include <emmintrin.h>
    #else
        #define SIMD_SCALAR
    #endif
#elif defined(__arm__) || defined(__aarch64__)
    #if defined(__ARM_NEON)
        #define SIMD_NEON
        #include <arm_neon.h>
    #else
        #define SIMD_SCALAR
    #endif
#endif

#if defined(SIMD_AVX) || defined(SIMD_AVX2)
using simd_vec_f = __m256;
inline simd_vec_f make_vec(float v) { return _mm256_set1_ps(v); }

#elif defined(SIMD_SSE2)
using simd_vec_f = __m128;
inline simd_vec_f make_vec(float v) { return _mm_set1_ps(v); }

#elif defined(SIMD_NEON)
using simd_vec_f = float32x4_t;
inline simd_vec_f make_vec(float v) { return vdupq_n_f32(v); }

#else
using simd_vec_f = float;
inline simd_vec_f make_vec(float v) { return v; }
#endif

} // simd


namespace simd {

template <typename T, std::size_t Alignment>
struct aligned_allocator {
    using value_type = T;

    aligned_allocator() noexcept = default;

    template <class U>
    constexpr aligned_allocator(const aligned_allocator<U, Alignment>&) noexcept {}

    T* allocate(std::size_t n) {
        void* ptr = nullptr; 

#if defined(__MSC_VER)
        ptr = _aligned_malloc(n * sizeof(T), Alignment);
        if (!ptr) throw std::bad_alloc();
#else
        if (posix_memalign(&ptr, Alignment, n * sizeof(T)) != 0) throw std::bad_alloc();
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

template <typename T, typename Operation>
void transform_inplace(T* data, std::size_t n, Operation operation) {
    std::size_t i = 0;

#if defined(SIMD_AVX) || defined(SIMD_AVX2)
    for (; i + 8 <= n; i += 8) {
        simd::simd_vec_f v = _mm256_load_ps(data + i);
        simd::simd_vec_f r = operation.simd(v);
        _mm256_store_ps(data + i, r);
    }

#elif defined(SIMD_SSE2)
    for (; i + 4 <= n; i += 4) {
        simd::simd_vec_f v = _mm_load_ps(data + i);
        simd::simd_vec_f r = operation.simd(v);
        _mm_store_ps(data + i, r);
    }

#elif defined(SIMD_NEON)
    for (; i + 4 <= n; i += 4) {
        simd::simd_vec_f v = vld1q_f32(data + i);
        simd::simd_vec_f r = operation.simd(v);
        vst1q_f32(data + i, r);
    }
#endif

    for (; i < n; i++) {
        data[i] = operation.simd(data[i]);
    }
}


namespace operations {

struct LinDistMap {
    const simd::simd_vec_f z_far;
    const simd::simd_vec_f z_fn_prod;
    const simd::simd_vec_f z_fn_sub;

    // Maintain float for residues after SIMD
    const float z_far_f;
    const float z_fn_prod_f;
    const float z_fn_sub_f;

    LinDistMap(const float zn, const float zf);

    // Maintain float for residues after SIMD
    inline float scalar(float d) const {
        return z_fn_prod_f / (z_far_f - d * z_fn_sub_f);
    }

#if defined(SIMD_AVX) || defined(SIMD_AVX2) 
    simd::simd_vec_f simd_v256(simd::simd_vec_f d) const;
#endif

#if defined(SIMD_SSE2)
    simd::simd_vec_f simd_v128(simd::simd_vec_f d) const;
#endif

#if defined(SIMD_NEON)
    simd::simd_vec_f simd_neon(simd::simd_vec_f d) const;
#endif
};


} // operations
} // simd