#include "simd.hpp"

namespace simd {
namespace operations {

// LinDistMap

LinDistMap::LinDistMap(float zn, float zf)
#if !defined(SIMD_SCALAR)
    : z_far_v(broadcast(zf)),
      z_fn_prod_v(broadcast(zf * zn)),
      z_fn_sub_v(broadcast(zf - zn)),
      z_far_f(zf), 
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn)
#else
    : z_far_f(zf),
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn)
#endif
{}


#if defined(SIMD_AVX) || defined(SIMD_AVX2)
 
vec_f32 LinDistMap::apply(__m256 d) const noexcept {
    __m256 denom;
#if defined(__FMA__)
    denom = _mm256_fnmadd_ps(d, z_fn_sub_v, z_far_v);
#else
    denom = _mm256_sub_ps(z_far_v, _mm256_mul_ps(d, z_fn_sub_v));
#endif
    return _mm256_div_ps(z_fn_prod_v, denom);
}
 
#elif defined(SIMD_SSE2)

vec_f32 LinDistMap::apply(__m128 d) const noexcept {
    __m128 denom;
#if defined(__FMA__)
    denom = _mm_fnmadd_ps(d, z_fn_sub_v, z_far_v);
#else
    denom = _mm_sub_ps(z_far_v, _mm_mul_ps(d, z_fn_sub_v));
#endif
    return _mm_div_ps(z_fn_prod_v, denom);
}

#elif defined(SIMD_NEON)

vec_f32 LinDistMap::apply(float32x4_t d) const noexcept {
    float32x4_t denom = vsubq_f32(z_far_v, vmulq_f32(d, z_fn_sub_v));
    return vdivq_f32(z_fn_prod_v, denom);
}
 
#endif

} // operations
} // simd