#include "simd.hpp"

namespace simd {
namespace operations {

// LinDistMap

LinDistMap::LinDistMap(const float zn, const float zf)
#if defined(SIMD_AVX) || defined(SIMD_AVX2) 
    : z_far(_mm256_set1_ps(zf)),
      z_fn_prod(_mm256_set1_ps(zf * zn)),
      z_fn_sub(_mm256_set1_ps(zf - zn)),
      z_far_f(zf),
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn)
#elif defined(SIMD_SSE2)
    : z_far(_mm_set1_ps(zf)),
      z_fn_prod(_mm_set1_ps(zf * zn)),
      z_fn_sub(_mm_set1_ps(zf - zn)),
      z_far_f(zf),
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn)
#elif defined(SIMD_NEON)
    : z_far(vdupq_n_f32(zf)),
      z_fn_prod(vdupq_n_f32(zf * zn)),
      z_fn_sub(vdupq_n_f32(zf - zn)),
      z_far_f(zf),
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn)
#else
    : z_far_f(zf),
      z_fn_prod_f(zf * zn),
      z_fn_sub_f(zf - zn)
#endif
{ }


#if defined(SIMD_AVX) || defined(SIMD_AVX2) 
    simd::simd_vec_f LinDistMap::simd_v256(simd::simd_vec_f d) const {
        
    }
#endif

#if defined(SIMD_SSE2)
    simd::simd_vec_f LinDistMap::simd_v128(simd::simd_vec_f d) const {

    }
#endif

#if defined(SIMD_NEON)
    simd::simd_vec_f LinDistMap::simd_neon(simd::simd_vec_f d) const {
        
    }
#endif

} // operations
} // simd