#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <new>
#include <stdexcept>

#define SIMD_ALIGNMENT 32 // 32 bit alignment given by LCM

#if defined(__AVX2__)
    #define SIMD_AVX2
    #include <immintrin.h>
#elif defined(__SSE4_2__)
    #define SIMD_SSE4
    #include <nmmintrin.h>
    #if defined(__FMA__) // FMA3 is under independant header
        #include <immintrin.h>
    #endif
#elif defined(__ARM_NEON)
    #define SIMD_NEON
    #include <arm_neon.h>
#else
    #define SIMD_SCALAR
#endif


#if defined(_MSC_VER)
    #define FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
    #define FORCE_INLINE __attribute__((always_inline)) inline
#else
    #define FORCE_INLINE inline
#endif


namespace simd { 


#if defined(SIMD_AVX2)
    using vec_f32  = __m256;
    using vec_ui16 = __m256i;
    constexpr std::size_t vec_width = 8;

    FORCE_INLINE vec_f32  broadcast_f32 (const float v)                 noexcept { return _mm256_set1_ps(v); }
    FORCE_INLINE vec_f32  vec_load_f32  (const float* p)                noexcept { return _mm256_load_ps(p); }
    FORCE_INLINE void     vec_store_f32 (float* p, const vec_f32 v)     noexcept { _mm256_store_ps(p, v); }
    FORCE_INLINE void     vec_store_ui16(uint16_t* p, const vec_ui16 v) noexcept { _mm256_store_si256(reinterpret_cast<__m256i*>(p), v); }
 
#elif defined(SIMD_SSE4)
    using vec_f32  = __m128;
    using vec_ui16 = __m128i;
    constexpr std::size_t vec_width = 4;

    FORCE_INLINE vec_f32  broadcast_f32 (const float v)                 noexcept { return _mm_set1_ps(v); }
    FORCE_INLINE vec_f32  vec_load_f32  (const float* p)                noexcept { return _mm_load_ps(p); }
    FORCE_INLINE void     vec_store_f32 (float* p, const vec_f32 v)     noexcept { _mm_store_ps(p, v); }
    FORCE_INLINE void     vec_store_ui16(uint16_t* p, const vec_ui16 v) noexcept { _mm_store_si128(reinterpret_cast<__m128i*>(p), v); }
 
#elif defined(SIMD_NEON)
    using vec_f32  = float32x4_t;
    using vec_ui16 = uint16x4_t;
    constexpr std::size_t vec_width = 4;

    FORCE_INLINE vec_f32  broadcast_f32 (const float v)                 noexcept { return vdupq_n_f32(v); }
    FORCE_INLINE vec_f32  vec_load_f32  (const float* p)                noexcept { return vld1q_f32(p); }
    FORCE_INLINE void     vec_store_f32 (float* p, const vec_f32 v)     noexcept { vst1q_f32(p, v); }
    FORCE_INLINE void     vec_store_ui16(uint16_t* p, const vec_ui16 v) noexcept { vst1_u16(p, v); }
 
#else // mock vectors
    using vec_f32  = float;
    using vec_ui16 = uint16_t;
    constexpr std::size_t vec_width = 1;

    FORCE_INLINE vec_f32  broadcast_f32 (const float v)                 noexcept { return v; }
    FORCE_INLINE vec_f32  vec_load_f32  (const float* p)                noexcept { return *p; }
    FORCE_INLINE void     vec_store_f32 (float* p, const vec_f32 v)     noexcept { *p = v; }
    FORCE_INLINE void     vec_store_ui16(uint16_t* p, const vec_ui16 v) noexcept { *p = v; }

#endif

} // simd


namespace simd {

template <typename T> 
struct vec_traits;

template <>
struct vec_traits<float> {
    using vec_type = vec_f32;
    FORCE_INLINE static vec_type load (const float* p)                  noexcept { return vec_load_f32(p); }
    FORCE_INLINE static void     store(float* p, const vec_type v)      noexcept { vec_store_f32(p, v); }
};

template <>
struct vec_traits<uint16_t> {
    using vec_type = vec_ui16;
    FORCE_INLINE static void     store(uint16_t* p, const vec_type v)   noexcept { vec_store_ui16(p, v); }
};

template <typename TIn, typename TOut, typename TOperation>
inline void transform(TIn* in, TOut* out, const TOperation& op, const std::size_t n) {
    std::size_t i = 0;

#if !defined(SIMD_SCALAR)
    for (; i + vec_width <= n; i += vec_width) {
        vec_traits<TOut>::store(out + i, op.apply(vec_traits<TIn>::load(in + i)));
    }
#endif

    for (; i < n; ++i) {
        out[i] = op.scalar(in[i]);
    }
}


namespace operations {

// Converts a normalized depth buffer [0, 1], as specified via OpenGL docs, into a human readable linear distance map 
struct ToLinDistMap {
    
#if !defined(SIMD_SCALAR)
    const vec_f32 z_far_v;
    const vec_f32 z_fn_prod_v;
    const vec_f32 z_fn_sub_v;
    const vec_f32 mm_to_m_conv_v;
#endif
    const float z_far_f;
    const float z_fn_prod_f;
    const float z_fn_sub_f;
    const float mm_to_m_conv_f = 1000; // conversion factor from millimeters to meters

    
    ToLinDistMap(const float zn, const float zf);

    inline uint16_t scalar(const float d) const noexcept {
        return static_cast<uint16_t>((z_fn_prod_f / (z_far_f - d * z_fn_sub_f)) * mm_to_m_conv_f);
    }

#if !defined(SIMD_SCALAR)
    vec_ui16 apply(vec_f32 d) const noexcept;
#endif
};


} // operations
} // simd