#include "simd.hpp"

namespace simd {
namespace operations {

ToLinDistMap::ToLinDistMap(float z_near, float z_far)
#if !defined(SIMD_SCALAR)
    : kZFarV(BroadcastF32(z_far)),
      kZFnProdV(BroadcastF32(z_far * z_near)),
      kZFnSubV(BroadcastF32(z_far - z_near)),
      kZFarF(z_far),
      kZFnProdF(z_far * z_near),
      kZFnSubF(z_far - z_near)
#else
    : kZFarF(z_far),
      kZFnProdF(z_far * z_near),
      kZFnSubF(z_far - z_near)
#endif
{ }

#if defined(SIMD_AVX2)

VecUI16 ToLinDistMap::apply(VecF32 d) const noexcept {
    __m256 denom;
#if defined(__FMA__)
    denom = _mm256_fnmadd_ps(d, kZFnSubV, kZFarV);
#else
    denom = _mm256_sub_ps(kZFarV, _mm256_mul_ps(d, kZFnSubV));
#endif
    __m256 lin_dist = _mm256_mul_ps(_mm256_set1_ps(kMmToMConv), _mm256_div_ps(kZFnProdV, denom));
    __m256i v_i32 = _mm256_cvtps_epi32(lin_dist);
    __m128i low = _mm256_castsi256_si128(v_i32);
    __m128i high = _mm256_extractf128_si256(v_i32, 1);

    return _mm_packus_epi32(low, high);
}

#elif defined(SIMD_SSE4)

VecUI16 ToLinDistMap::apply(VecF32 d) const noexcept {
    __m128 denom;
#if defined(__FMA__)
    denom = _mm_fnmadd_ps(d, kZFnSubV, kZFarV);
#else
    denom = _mm_sub_ps(kZFarV, _mm_mul_ps(d, kZFnSubV));
#endif
    __m128 lin_dist = _mm_mul_ps(_mm_set1_ps(kMmToMConv), _mm_div_ps(kZFnProdV, denom));
    __m128i v_i32 = _mm_cvtps_epi32(lin_dist);

    return _mm_packus_epi32(v_i32, _mm_setzero_si128());
}

#elif defined(SIMD_NEON)

VecUI16 ToLinDistMap::apply(float32x4_t d) const noexcept {
    float32x4_t denom = vsubq_f32(kZFarV, vmulq_f32(d, kZFnSubV));
    float32x4_t lin_dist_f = vmulq_f32(
        vdupq_n_f32(kMmToMConv),
        vdivq_f32(kZFnProdV, denom)
    );
    uint32x4_t v_i32 = vcvtaq_u32_f32(lin_dist_f);
    return vqmovn_u32(v_i32);
}

#endif

}  // namespace operations
}  // namespace simd