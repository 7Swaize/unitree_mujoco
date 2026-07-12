#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <new>
#include <stdexcept>

#define SIMD_ALIGNMENT 32 // 32 byte alignment given by LCM

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
    using VecF32 = __m256;
    using VecUI16 = __m256i;
    constexpr std::size_t kVecWidth = 8;

    FORCE_INLINE VecF32 BroadcastF32(const float v) noexcept { return _mm256_set1_ps(v); }
    FORCE_INLINE VecF32 VecLoadF32(const float* p) noexcept { return _mm256_load_ps(p); }
    FORCE_INLINE void VecStoreF32(float* p, const VecF32 v) noexcept { _mm256_store_ps(p, v); }
    FORCE_INLINE void VecStoreUI16(uint16_t* p, const VecUI16 v) noexcept { _mm256_store_si256(reinterpret_cast<__m256i*>(p), v); }
#elif defined(SIMD_SSE4)
    using VecF32 = __m128;
    using VecUI16 = __m128i;
    constexpr std::size_t kVecWidth = 4;

    FORCE_INLINE VecF32 BroadcastF32(const float v) noexcept { return _mm_set1_ps(v); }
    FORCE_INLINE VecF32 VecLoadF32(const float* p) noexcept { return _mm_load_ps(p); }
    FORCE_INLINE void VecStoreF32(float* p, const VecF32 v) noexcept { _mm_store_ps(p, v); }
    FORCE_INLINE void VecStoreUI16(uint16_t* p, const VecUI16 v) noexcept { _mm_store_si128(reinterpret_cast<__m128i*>(p), v); }
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
inline void Transform(TIn* in, TOut* out, const TOperation& op, const std::size_t n) {
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
    VecF32 kZFarV;
    VecF32 kZFnProdV;
    VecF32 kZFnSubV;
#endif
    
    float kZFarF;
    float kZFnProdF;
    float kZFnSubF;

    static constexpr float kMmToMConv = 1000.0f; // conversion factor from millimeters to meters
    
    ToLinDistMap(float z_near, float z_far);

    inline uint16_t scalar(const float d) const noexcept {
        return static_cast<uint16_t>((kZFnProdF / (kZFarF - d * kZFnSubF)) * kMmToMConv);
    }

#if !defined(SIMD_SCALAR)
    VecUI16 apply(VecF32 d) const noexcept;
#endif
};

}  // namespace operations
}  // namespace simd