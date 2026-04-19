#include "simd.hpp"

namespace simd {
namespace operations {

// LinDistMap

ToLinDistMap::ToLinDistMap(float zn, float zf)
#if !defined(SIMD_SCALAR)
    : z_far_v(broadcast_f32(zf)),
      z_fn_prod_v(broadcast_f32(zf * zn)),
      z_fn_sub_v(broadcast_f32(zf - zn)),
      z_far_f(zf), 
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn),
      mm_to_m_conv_v(broadcast_f32(1000))
#else
    : z_far_f(zf),
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn)
#endif
{}


#if defined(SIMD_AVX2)
 
vec_ui16 LinDistMap::apply(__m256 d) const noexcept {
    __m256 denom;
#if defined(__FMA__)
    denom = _mm256_fnmadd_ps(d, z_fn_sub_v, z_far_v);
#else
    denom = _mm256_sub_ps(z_far_v, _mm256_mul_ps(d, z_fn_sub_v));
#endif
    __m256 lin_dist = _mm256_mul_ps(mm_to_m_conv_v, _mm256_div_ps(z_fn_prod_v, denom));
    __m256i v_i32 = _mm256_cvtps_epi32(lin_dist)
    __m128i low = _mm256_castsi256_si128(v_i32);
    __m128i high = _mm256_extractf128_si256(v_i32, 1);

    return _mm_packus_epi32(low, high);
}
 
#elif defined(SIMD_SSE4)

vec_ui16 LinDistMap::apply(__m128 d) const noexcept {
    __m128 denom;
#if defined(__FMA__)
    denom = _mm_fnmadd_ps(d, z_fn_sub_v, z_far_v);
#else
    denom = _mm_sub_ps(z_far_v, _mm_mul_ps(d, z_fn_sub_v));
#endif
    __m128 lin_dist = _mm_mul_ps(mm_to_m_conv_v, _mm_div_ps(z_fn_prod_v, denom));
    __m128i v_i32 = _mm_cvtps_epi32(lin_dist);

    return _mm_packus_epi32(v_i32, _mm_setzero_si128());
}

#elif defined(SIMD_NEON)

vec_ui16 ToLinDistMap::apply(float32x4_t d) const noexcept {
    float32x4_t denom = vsubq_f32(z_far_v, vmulq_f32(d, z_fn_sub_v));
    float32x4_t lin_dist_f = vmulq_f32(mm_to_m_conv_v, vdivq_f32(z_fn_prod_v, denom));
    uint32x4_t v_i32 = vcvtaq_u32_f32(lin_dist_f);
    return vqmovn_u32(v_i32);
}
 
#endif

} // operations
} // simd