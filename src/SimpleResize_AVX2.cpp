
#include <immintrin.h>

#include "SimpleResize.h"


#ifdef _WIN32
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE inline __attribute__((always_inline))
#endif

#define zeroes _mm_setzero_si128()


static FORCE_INLINE void simpleResize_uint8_t_vertical_4px_avx2(uint8_t *workp, const uint8_t *srcp1, const uint8_t *srcp2, int x, const __m128i &dwords_weights) {
    __m128i top = _mm_cvtsi32_si128(*(const int *)&srcp1[x]);
    __m128i bottom = _mm_cvtsi32_si128(*(const int *)&srcp2[x]);
    __m128i pixels = _mm_unpacklo_epi8(_mm_unpacklo_epi8(bottom, top), zeroes);

    __m128i dst = _mm_madd_epi16(pixels, dwords_weights);

    dst = _mm_add_epi32(dst, _mm_set1_epi32(simple_resize_weight_half));
    dst = _mm_srli_epi32(dst, simple_resize_weight_shift);
    dst = _mm_packs_epi32(dst, dst);
    dst = _mm_packus_epi16(dst, dst);
    *(int *)&workp[x] =  _mm_cvtsi128_si32(dst);
}


static FORCE_INLINE void simpleResize_uint8_t_horizontal_8px_avx2(const SimpleResize *simple, uint8_t *dstp, uint8_t *workp, int x, const __m256i &shuffle_mask) {
    __m256i dwords_weights_h = _mm256_loadu_si256((const __m256i *)&simple->horizontal_weights[x]);
    __m256i dwords_offsets = _mm256_loadu_si256((const __m256i *)&simple->horizontal_offsets[x]);
    __m256i pixels = _mm256_i32gather_epi32((const int *)workp, dwords_offsets, sizeof(uint8_t));

    pixels = _mm256_shuffle_epi8(pixels, shuffle_mask);

    pixels = _mm256_madd_epi16(pixels, dwords_weights_h);
    pixels = _mm256_add_epi32(pixels, _mm256_set1_epi32(simple_resize_weight_half));
    pixels = _mm256_srai_epi32(pixels, simple_resize_weight_shift);
    pixels = _mm256_packs_epi32(pixels, pixels);
    pixels = _mm256_permute4x64_epi64(pixels, 0xe8); // 0b11101000
    pixels = _mm256_packus_epi16(pixels, pixels);

    _mm_storel_epi64((__m128i *)&dstp[x], _mm256_castsi256_si128(pixels));
}


// Thread-safe.
void simpleResize_uint8_t_avx2(const SimpleResize *simple, uint8_t *dstp, int dst_stride, const uint8_t *srcp, int src_stride) {
    // Two additional bytes because of vpgatherdd.
    uint8_t *workp = (uint8_t *)malloc(simple->src_width * sizeof(uint8_t) + 2);

#define SHUFFLE_PATTERN -0x80, 13, -0x80, 12, -0x80, 9, -0x80, 8, -0x80, 5, -0x80, 4, -0x80, 1, -0x80, 0
    __m256i shuffle_mask = _mm256_set_epi8(SHUFFLE_PATTERN, SHUFFLE_PATTERN);
#undef SHUFFLE_PATTERN

    for (int y = 0; y < simple->dst_height; y++) {
        int weight_bottom = simple->vertical_weights[y];
        int weight_top = simple_resize_weight_max - weight_bottom;

        const uint8_t *srcp1 = srcp + simple->vertical_offsets[y] * src_stride;
        const uint8_t *srcp2 = srcp1 + src_stride;

        __m128i dwords_weights_v = _mm_set1_epi32((weight_top << 16) | weight_bottom);

        int pixels_per_iteration = 4;
        const int src_width_avx2 = simple->src_width & ~(pixels_per_iteration - 1);

        /* vertical */
        for (int x = 0; x < src_width_avx2; x += pixels_per_iteration)
            simpleResize_uint8_t_vertical_4px_avx2(workp, srcp1, srcp2, x, dwords_weights_v);

        if (src_width_avx2 < simple->src_width)
            simpleResize_uint8_t_vertical_4px_avx2(workp, srcp1, srcp2, simple->src_width - pixels_per_iteration, dwords_weights_v);


        pixels_per_iteration = 8;
        const int dst_width_avx2 = simple->dst_width & ~(pixels_per_iteration - 1);

        /* horizontal */
        for (int x = 0; x < dst_width_avx2; x += pixels_per_iteration)
            simpleResize_uint8_t_horizontal_8px_avx2(simple, dstp, workp, x, shuffle_mask);

        if (dst_width_avx2 < simple->dst_width)
            simpleResize_uint8_t_horizontal_8px_avx2(simple, dstp, workp, simple->dst_width - pixels_per_iteration, shuffle_mask);

        dstp += dst_stride;
    }

    free(workp);
}


static FORCE_INLINE void simpleResize_int16_t_vertical_8px_avx2(int16_t *workp, const int16_t *srcp1, const int16_t *srcp2, int x, const __m128i &dwords_weights) {
    __m128i top = _mm_loadu_si128((const __m128i *)&srcp1[x]);
    __m128i bottom = _mm_loadu_si128((const __m128i *)&srcp2[x]);
    __m128i pixels_lo = _mm_unpacklo_epi16(bottom, top);
    __m128i pixels_hi = _mm_unpackhi_epi16(bottom, top);

    __m128i dst_lo = _mm_madd_epi16(pixels_lo, dwords_weights);
    __m128i dst_hi = _mm_madd_epi16(pixels_hi, dwords_weights);
    dst_lo = _mm_add_epi32(dst_lo, _mm_set1_epi32(simple_resize_weight_half));
    dst_hi = _mm_add_epi32(dst_hi, _mm_set1_epi32(simple_resize_weight_half));
    dst_lo = _mm_srai_epi32(dst_lo, simple_resize_weight_shift);
    dst_hi = _mm_srai_epi32(dst_hi, simple_resize_weight_shift);
    __m128i dst = _mm_packs_epi32(dst_lo, dst_hi);
    _mm_storeu_si128((__m128i *)&workp[x], dst);
}


static FORCE_INLINE void simpleResize_int16_t_horizontal_8px_avx2(const SimpleResize *simple, int16_t *dstp, int16_t *workp, int x) {
    __m256i dwords_weights_h = _mm256_loadu_si256((const __m256i *)&simple->horizontal_weights[x]);
    __m256i dwords_offsets = _mm256_loadu_si256((const __m256i *)&simple->horizontal_offsets[x]);
    __m256i pixels = _mm256_i32gather_epi32((const int *)workp, dwords_offsets, sizeof(int16_t));
    pixels = _mm256_madd_epi16(pixels, dwords_weights_h);
    pixels = _mm256_add_epi32(pixels, _mm256_set1_epi32(simple_resize_weight_half));
    pixels = _mm256_srai_epi32(pixels, simple_resize_weight_shift);
    pixels = _mm256_packs_epi32(pixels, pixels);

    _mm_storeu_si128((__m128i *)&dstp[x], _mm256_castsi256_si128(_mm256_permute4x64_epi64(pixels, 0xe8))); // 0b11101000
}


// Thread-safe.
void simpleResize_int16_t_avx2(const SimpleResize *simple, int16_t *dstp, int dst_stride, const int16_t *srcp, int src_stride) {
    int16_t *workp = (int16_t *)malloc(simple->src_width * sizeof(int16_t));

    for (int y = 0; y < simple->dst_height; y++) {
        int weight_bottom = simple->vertical_weights[y];
        int weight_top = simple_resize_weight_max - weight_bottom;

        const int16_t *srcp1 = srcp + simple->vertical_offsets[y] * src_stride;
        const int16_t *srcp2 = srcp1 + src_stride;

        __m128i dwords_weights_v = _mm_set1_epi32((weight_top << 16) | weight_bottom);

        const int pixels_per_iteration = 8;
        const int src_width_sse2 = simple->src_width & ~(pixels_per_iteration - 1);

        /* vertical */
        for (int x = 0; x < src_width_sse2; x += pixels_per_iteration)
            simpleResize_int16_t_vertical_8px_avx2(workp, srcp1, srcp2, x, dwords_weights_v);

        if (src_width_sse2 < simple->src_width)
            simpleResize_int16_t_vertical_8px_avx2(workp, srcp1, srcp2, simple->src_width - pixels_per_iteration, dwords_weights_v);


        const int dst_width_avx2 = simple->dst_width & ~(pixels_per_iteration - 1);

        /* horizontal */
        for (int x = 0; x < dst_width_avx2; x += pixels_per_iteration)
            simpleResize_int16_t_horizontal_8px_avx2(simple, dstp, workp, x);

        if (dst_width_avx2 < simple->dst_width)
            simpleResize_int16_t_horizontal_8px_avx2(simple, dstp, workp, simple->dst_width - pixels_per_iteration);

        dstp += dst_stride;
    }

    free(workp);
}
