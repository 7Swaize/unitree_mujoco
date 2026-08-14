#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <new>
#include <stdexcept>

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

#if defined(SIMD_AVX2)
    #define SIMD_ALIGNMENT 32
#elif defined(SIMD_SSE4) || defined(SIMD_NEON)
    #define SIMD_ALIGNMENT 16
#else
    #define SIMD_ALIGNMENT 1
#endif

#if defined(_MSC_VER)
    #define FORCE_INLINE __forceinline
    #define RESTRICT __restrict
#elif defined(__GNUC__) || defined(__clang__)
    #define FORCE_INLINE __attribute__((always_inline)) inline
    #define RESTRICT __restrict__
#else
    #define FORCE_INLINE inline
    #define RESTRICT
#endif

namespace simd {

#if defined(SIMD_AVX2)
    using VecF32 = __m256;
    using VecUI16 = __m128i;
    constexpr std::size_t kVecWidth = 8;

    FORCE_INLINE VecF32 BroadcastF32(const float v) noexcept { return _mm256_set1_ps(v); }
    FORCE_INLINE VecF32 VecLoadF32(const float* p) noexcept { return _mm256_load_ps(p); }
    FORCE_INLINE void VecStoreF32(float* p, const VecF32 v) noexcept { _mm256_store_ps(p, v); }
    FORCE_INLINE void VecStoreUI16(uint16_t* p, const VecUI16 v) noexcept { _mm_store_si128(reinterpret_cast<__m128i*>(p), v); }
#elif defined(SIMD_SSE4)
    using VecF32 = __m128;
    using VecUI16 = __m128i;
    constexpr std::size_t kVecWidth = 4;

    FORCE_INLINE VecF32 BroadcastF32(const float v) noexcept { return _mm_set1_ps(v); }
    FORCE_INLINE VecF32 VecLoadF32(const float* p) noexcept { return _mm_load_ps(p); }
    FORCE_INLINE void VecStoreF32(float* p, const VecF32 v) noexcept { _mm_store_ps(p, v); }
    // Store the lower half because of the upper 64-bit padding from the conversion
    FORCE_INLINE void VecStoreUI16(uint16_t* p, const VecUI16 v) noexcept { _mm_storel_epi64(reinterpret_cast<__m128i*>(p), v); }
#elif defined(SIMD_NEON)
    using VecF32 = float32x4_t;
    using VecUI16 = uint16x4_t;
    constexpr std::size_t kVecWidth = 4;

    FORCE_INLINE VecF32 BroadcastF32(const float v) noexcept { return vdupq_n_f32(v); }
    FORCE_INLINE VecF32 VecLoadF32(const float* p) noexcept { return vld1q_f32(p); }
    FORCE_INLINE void VecStoreF32(float* p, const VecF32 v) noexcept { vst1q_f32(p, v); }
    FORCE_INLINE void VecStoreUI16(uint16_t* p, const VecUI16 v) noexcept { vst1_u16(p, v); }
#else
    using VecF32 = float;
    using VecUI16 = uint16_t;
    constexpr std::size_t kVecWidth = 1;

    FORCE_INLINE VecF32 BroadcastF32(const float v) noexcept { return v; }
    FORCE_INLINE VecF32 VecLoadF32(const float* p) noexcept { return *p; }
    FORCE_INLINE void VecStoreF32(float* p, const VecF32 v) noexcept { *p = v; }
    FORCE_INLINE void VecStoreUI16(uint16_t* p, const VecUI16 v) noexcept { *p = v; }
#endif

}  // namespace simd

namespace simd {

template <typename T>
struct VecTraits;

template <>
struct VecTraits<float> {
    using VecType = VecF32;

    FORCE_INLINE static VecType Load(const float* p) noexcept { return VecLoadF32(p); }
    FORCE_INLINE static void Store(float* p, const VecType v) noexcept { VecStoreF32(p, v); }
};

template <>
struct VecTraits<uint16_t> {
    using VecType = VecUI16;

    FORCE_INLINE static void Store(uint16_t* p, const VecType v) noexcept { VecStoreUI16(p, v); }
};

template <typename TIn, typename TOut, typename TOperation>
inline void Transform(const TIn* RESTRICT in, TOut* RESTRICT out, const TOperation& op, const std::size_t n) {
    std::size_t i = 0;

#if !defined(SIMD_SCALAR)
    for (; i + kVecWidth <= n; i += kVecWidth) {
        VecTraits<TOut>::Store(out + i, op.apply(VecTraits<TIn>::Load(in + i)));
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
    const VecF32 kZFarV;
    const VecF32 kZFnProdV;
    const VecF32 kZFnSubV;
#endif
    const float kZFarF;
    const float kZFnProdF;
    const float kZFnSubF;

    static constexpr float kMmToMConv = 1000.0f; // conversion factor from millimeters to meters

    ToLinDistMap(const float z_near, const float z_far);

    [[nodiscard]] inline uint16_t scalar(const float d) const noexcept {
        const float lin_dist = (kZFnProdF / (kZFarF - d * kZFnSubF)) * kMmToMConv;
        const float rounded = lin_dist + 0.5f;
        const float clamped = std::clamp(rounded, 0.0f, static_cast<float>(std::numeric_limits<uint16_t>::max()));
        return static_cast<uint16_t>(clamped);
    }

#if !defined(SIMD_SCALAR)
    [[nodiscard]] VecUI16 apply(VecF32 d) const noexcept;
#endif
};

}  // namespace operations
}  // namespace simd