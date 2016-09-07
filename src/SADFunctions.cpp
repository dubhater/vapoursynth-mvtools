#include <cstdlib>
#include <unordered_map>

#include "CPU.h"
#include "SADFunctions.h"


enum InstructionSets {
    Scalar,
    MMX,
    MMX_CACHE64,
    SSE2,
    SSE3,
    SSSE3,
    SSSE3_CACHE64,
    SSE4,
    AVX,
    XOP,
    AVX2,
};


#ifdef _WIN32
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE inline __attribute__((always_inline))
#endif


#if defined(MVTOOLS_X86)

#include <emmintrin.h>


#define zeroes _mm_setzero_si128()


// This version used for width >= 16.
template <unsigned width, unsigned height>
struct SADWrapperU8 {

    static unsigned int sad_u8_sse2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m128i sum = zeroes;

        for (unsigned y = 0; y < height; y++) {
            for (unsigned x = 0; x < width; x += 16) {
                __m128i m2 = _mm_loadu_si128((const __m128i *)&pSrc[x]);
                __m128i m3 = _mm_loadu_si128((const __m128i *)&pRef[x]);

                __m128i diff = _mm_sad_epu8(m2, m3);

                sum = _mm_add_epi64(sum, diff);
            }

            pSrc += nSrcPitch;
            pRef += nRefPitch;
        }

        sum = _mm_add_epi32(sum, _mm_srli_si128(sum, 8));

        return (unsigned)_mm_cvtsi128_si32(sum);
    }

};


template <unsigned height>
struct SADWrapperU8<4, height> {

    static unsigned int sad_u8_sse2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m128i sum = zeroes;

        for (unsigned y = 0; y < height; y++) {
            __m128i m2 = _mm_cvtsi32_si128(*(const int *)pSrc);
            __m128i m3 = _mm_cvtsi32_si128(*(const int *)pRef);

            __m128i diff = _mm_sad_epu8(m2, m3);

            sum = _mm_add_epi64(sum, diff);

            pSrc += nSrcPitch;
            pRef += nRefPitch;
        }

        return (unsigned)_mm_cvtsi128_si32(sum);
    }

};


template <unsigned height>
struct SADWrapperU8<8, height> {

    static unsigned int sad_u8_sse2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m128i sum = zeroes;

        for (unsigned y = 0; y < height; y++) {
            __m128i m2 = _mm_loadl_epi64((const __m128i *)pSrc);
            __m128i m3 = _mm_loadl_epi64((const __m128i *)pRef);

            __m128i diff = _mm_sad_epu8(m2, m3);

            sum = _mm_add_epi64(sum, diff);

            pSrc += nSrcPitch;
            pRef += nRefPitch;
        }

        return (unsigned)_mm_cvtsi128_si32(sum);
    }

};


static FORCE_INLINE __m128i abs_diff_epu16(const __m128i &a, const __m128i &b) {
    return _mm_or_si128(_mm_subs_epu16(a, b),
                        _mm_subs_epu16(b, a));
}


static FORCE_INLINE __m128i hsum_epi32(const __m128i &a) {
    __m128i sum = a;
    __m128i m0 = _mm_srli_si128(sum, 8);
    sum = _mm_add_epi32(sum, m0);
    m0 = _mm_srli_epi64(sum, 32);
    sum = _mm_add_epi32(sum, m0);

    return sum;
}


// This version used for width >= 8.
template <unsigned width, unsigned height>
struct SADWrapperU16 {

    static unsigned int sad_u16_sse2(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
        __m128i sum = zeroes;

        for (unsigned y = 0; y < height; y++) {
            for (unsigned x = 0; x < width; x += 8) {
                const uint16_t *pSrc = (const uint16_t *)pSrc8;
                const uint16_t *pRef = (const uint16_t *)pRef8;

                __m128i m2 = _mm_loadu_si128((const __m128i *)&pSrc[x]);
                __m128i m3 = _mm_loadu_si128((const __m128i *)&pRef[x]);

                __m128i diff = abs_diff_epu16(m2, m3);

                sum = _mm_add_epi32(sum, _mm_unpacklo_epi16(diff, zeroes));
                sum = _mm_add_epi32(sum, _mm_unpackhi_epi16(diff, zeroes));
            }

            pSrc8 += nSrcPitch;
            pRef8 += nRefPitch;
        }

        return (unsigned)_mm_cvtsi128_si32(hsum_epi32(sum));
    }

};


template <>
struct SADWrapperU16<2, 2> {

    static unsigned int sad_u16_sse2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m128i m2 = _mm_cvtsi32_si128(*(const int *)(pSrc));
        __m128i m4 = _mm_cvtsi32_si128(*(const int *)(pSrc + nSrcPitch));
        m2 = _mm_unpacklo_epi16(m2, m4);
        __m128i m3 = _mm_cvtsi32_si128(*(const int *)(pRef));
        __m128i m5 = _mm_cvtsi32_si128(*(const int *)(pRef + nRefPitch));
        m3 = _mm_unpacklo_epi16(m3, m5);

        m2 = abs_diff_epu16(m2, m3);

        m2 = _mm_unpacklo_epi16(m2, zeroes);

        return (unsigned)_mm_cvtsi128_si32(hsum_epi32(m2));
    }

};


template <>
struct SADWrapperU16<2, 4> {

    static unsigned int sad_u16_sse2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m128i m2 = _mm_cvtsi32_si128(*(const int *)(pSrc));
        __m128i m4 = _mm_cvtsi32_si128(*(const int *)(pSrc + nSrcPitch));
        m2 = _mm_unpacklo_epi16(m2, m4);
        __m128i m3 = _mm_cvtsi32_si128(*(const int *)(pRef));
        __m128i m5 = _mm_cvtsi32_si128(*(const int *)(pRef + nRefPitch));
        m3 = _mm_unpacklo_epi16(m3, m5);

        __m128i m6 = _mm_cvtsi32_si128(*(const int *)(pSrc + nSrcPitch * 2));
        __m128i m8 = _mm_cvtsi32_si128(*(const int *)(pSrc + nSrcPitch * 3));
        m6 = _mm_unpacklo_epi16(m6, m8);
        __m128i m7 = _mm_cvtsi32_si128(*(const int *)(pRef + nRefPitch * 2));
        __m128i m9 = _mm_cvtsi32_si128(*(const int *)(pRef + nRefPitch * 3));
        m7 = _mm_unpacklo_epi16(m7, m9);

        m2 = _mm_unpacklo_epi16(m2, m6);
        m3 = _mm_unpacklo_epi16(m3, m7);

        m2 = abs_diff_epu16(m2, m3);

        m2 = _mm_add_epi32(_mm_unpacklo_epi16(m2, zeroes),
                           _mm_unpackhi_epi16(m2, zeroes));

        return (unsigned)_mm_cvtsi128_si32(hsum_epi32(m2));
    }

};


template <unsigned height>
struct SADWrapperU16<4, height> {

    static unsigned int sad_u16_sse2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m128i sum = zeroes;

        for (unsigned y = 0; y < height; y++) {
            __m128i m2 = _mm_loadl_epi64((const __m128i *)pSrc);
            __m128i m3 = _mm_loadl_epi64((const __m128i *)pRef);

            __m128i diff = abs_diff_epu16(m2, m3);

            sum = _mm_add_epi32(sum, _mm_unpacklo_epi16(diff, zeroes));

            pSrc += nSrcPitch;
            pRef += nRefPitch;
        }

        return (unsigned)_mm_cvtsi128_si32(hsum_epi32(sum));
    }

};


#undef zeroes


#define MK_CFUNC(functionname) extern "C" unsigned int functionname(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch)

// From sad-a.asm - stolen from x264
MK_CFUNC(mvtools_pixel_sad_4x4_mmx2);
MK_CFUNC(mvtools_pixel_sad_4x8_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x4_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x8_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x16_mmx2);
MK_CFUNC(mvtools_pixel_sad_16x8_mmx2);
MK_CFUNC(mvtools_pixel_sad_16x16_mmx2);

MK_CFUNC(mvtools_pixel_sad_8x4_cache64_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x8_cache64_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x16_cache64_mmx2);

MK_CFUNC(mvtools_pixel_sad_8x16_sse2);
MK_CFUNC(mvtools_pixel_sad_16x8_sse2);  //non optimized cache access, for AMD?
MK_CFUNC(mvtools_pixel_sad_16x16_sse2); //non optimized cache access, for AMD?

MK_CFUNC(mvtools_pixel_sad_16x8_sse3);  //LDDQU Pentium4E (Core1?), not for Core2!
MK_CFUNC(mvtools_pixel_sad_16x16_sse3); //LDDQU Pentium4E (Core1?), not for Core2!

MK_CFUNC(mvtools_pixel_sad_16x8_cache64_ssse3);  //core2 optimized
MK_CFUNC(mvtools_pixel_sad_16x16_cache64_ssse3); //core2 optimized

/* SATD: Sum of Absolute Transformed Differences, more sensitive to noise, frequency domain based - replacement to dct/SAD */

// From sad-a.asm - stolen from x264
MK_CFUNC(mvtools_pixel_satd_4x4_mmx2);

MK_CFUNC(mvtools_pixel_satd_8x4_sse2);
MK_CFUNC(mvtools_pixel_satd_8x8_sse2);
MK_CFUNC(mvtools_pixel_satd_16x8_sse2);
MK_CFUNC(mvtools_pixel_satd_16x16_sse2);

MK_CFUNC(mvtools_pixel_satd_4x4_ssse3);
MK_CFUNC(mvtools_pixel_satd_8x4_ssse3);
MK_CFUNC(mvtools_pixel_satd_8x8_ssse3);
MK_CFUNC(mvtools_pixel_satd_16x8_ssse3);
MK_CFUNC(mvtools_pixel_satd_16x16_ssse3);

MK_CFUNC(mvtools_pixel_satd_4x4_sse4);
MK_CFUNC(mvtools_pixel_satd_8x4_sse4);
MK_CFUNC(mvtools_pixel_satd_8x8_sse4);
MK_CFUNC(mvtools_pixel_satd_16x8_sse4);
MK_CFUNC(mvtools_pixel_satd_16x16_sse4);

MK_CFUNC(mvtools_pixel_satd_4x4_avx);
MK_CFUNC(mvtools_pixel_satd_8x4_avx);
MK_CFUNC(mvtools_pixel_satd_8x8_avx);
MK_CFUNC(mvtools_pixel_satd_16x8_avx);
MK_CFUNC(mvtools_pixel_satd_16x16_avx);

MK_CFUNC(mvtools_pixel_satd_4x4_xop);
MK_CFUNC(mvtools_pixel_satd_8x4_xop);
MK_CFUNC(mvtools_pixel_satd_8x8_xop);
MK_CFUNC(mvtools_pixel_satd_16x8_xop);
MK_CFUNC(mvtools_pixel_satd_16x16_xop);

MK_CFUNC(mvtools_pixel_satd_8x8_avx2);
MK_CFUNC(mvtools_pixel_satd_16x8_avx2);
MK_CFUNC(mvtools_pixel_satd_16x16_avx2);

#undef MK_CFUNC

#endif // MVTOOLS_X86


template <unsigned width, unsigned height, typename PixelType>
unsigned int sad_c(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
    unsigned int sum = 0;
    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x++) {
            const PixelType *pSrc = (const PixelType *)pSrc8;
            const PixelType *pRef = (const PixelType *)pRef8;
            sum += (unsigned)std::abs(pSrc[x] - pRef[x]);
        }
        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }
    return sum;
}


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)


#if defined(MVTOOLS_X86)
#define SAD_X264_U8_MMX(width, height) \
    { KEY(width, height, 8, MMX), mvtools_pixel_sad_##width##x##height##_mmx2 },

#define SAD_X264_U8_MMX_CACHE64(width, height) \
    { KEY(width, height, 8, MMX_CACHE64), mvtools_pixel_sad_##width##x##height##_cache64_mmx2 },

#define SAD_X264_U8_SSE2(width, height) \
    { KEY(width, height, 8, SSE2), mvtools_pixel_sad_##width##x##height##_sse2 },

#define SAD_X264_U8_SSE3(width, height) \
    { KEY(width, height, 8, SSE3), mvtools_pixel_sad_##width##x##height##_sse3 },

#define SAD_X264_U8_SSSE3_CACHE64(width, height) \
    { KEY(width, height, 8, SSSE3_CACHE64), mvtools_pixel_sad_##width##x##height##_cache64_ssse3 },

#define SAD_U8_SSE2(width, height) \
    { KEY(width, height, 8, SSE2), SADWrapperU8<width, height>::sad_u8_sse2 },

#define SAD_U16_SSE2(width, height) \
    { KEY(width, height, 16, SSE2), SADWrapperU16<width, height>::sad_u16_sse2 },
#else
#define SAD_X264_U8_MMX(width, height)
#define SAD_X264_U8_MMX_CACHE64(width, height)
#define SAD_X264_U8_SSE2(width, height)
#define SAD_X264_U8_SSE3(width, height)
#define SAD_X264_U8_SSSE3_CACHE64(width, height)
#define SAD_U8_SSE2(width, height)
#define SAD_U16_SSE2(width, height)
#endif

#define SAD(width, height) \
    { KEY(width, height, 8, Scalar), sad_c<width, height, uint8_t> }, \
    { KEY(width, height, 16, Scalar), sad_c<width, height, uint16_t> },

static const std::unordered_map<uint32_t, SADFunction> sad_functions = {
    SAD(2, 2)
    SAD(2, 4)
    SAD(4, 2)
    SAD(4, 4)
    SAD(4, 8)
    SAD(8, 1)
    SAD(8, 2)
    SAD(8, 4)
    SAD(8, 8)
    SAD(8, 16)
    SAD(16, 1)
    SAD(16, 2)
    SAD(16, 4)
    SAD(16, 8)
    SAD(16, 16)
    SAD(16, 32)
    SAD(32, 8)
    SAD(32, 16)
    SAD(32, 32)
    SAD(32, 64)
    SAD(64, 16)
    SAD(64, 32)
    SAD(64, 64)
    SAD(64, 128)
    SAD(128, 32)
    SAD(128, 64)
    SAD(128, 128)
    SAD_X264_U8_MMX(4, 4)
    SAD_X264_U8_MMX(4, 8)
    SAD_X264_U8_MMX(8, 4)
    SAD_X264_U8_MMX(8, 8)
    SAD_X264_U8_MMX_CACHE64(8, 4)
    SAD_X264_U8_MMX_CACHE64(8, 8)
    SAD_X264_U8_SSE2(8, 16)
    SAD_X264_U8_SSE2(16, 8)
    SAD_X264_U8_SSE2(16, 16)
    SAD_X264_U8_SSE3(16, 8)
    SAD_X264_U8_SSE3(16, 16)
    SAD_X264_U8_SSSE3_CACHE64(16, 8)
    SAD_X264_U8_SSSE3_CACHE64(16, 16)
    SAD_U8_SSE2(4, 2)
    SAD_U8_SSE2(8, 1)
    SAD_U8_SSE2(8, 2)
    SAD_U8_SSE2(16, 1)
    SAD_U8_SSE2(16, 2)
    SAD_U8_SSE2(16, 4)
    SAD_U8_SSE2(16, 32)
    SAD_U8_SSE2(32, 8)
    SAD_U8_SSE2(32, 16)
    SAD_U8_SSE2(32, 32)
    SAD_U8_SSE2(32, 64)
    SAD_U8_SSE2(64, 16)
    SAD_U8_SSE2(64, 32)
    SAD_U8_SSE2(64, 64)
    SAD_U8_SSE2(64, 128)
    SAD_U8_SSE2(128, 32)
    SAD_U8_SSE2(128, 64)
    SAD_U8_SSE2(128, 128)
    SAD_U16_SSE2(2, 2)
    SAD_U16_SSE2(2, 4)
    SAD_U16_SSE2(4, 2)
    SAD_U16_SSE2(4, 4)
    SAD_U16_SSE2(4, 8)
    SAD_U16_SSE2(8, 1)
    SAD_U16_SSE2(8, 2)
    SAD_U16_SSE2(8, 4)
    SAD_U16_SSE2(8, 8)
    SAD_U16_SSE2(8, 16)
    SAD_U16_SSE2(16, 1)
    SAD_U16_SSE2(16, 2)
    SAD_U16_SSE2(16, 4)
    SAD_U16_SSE2(16, 8)
    SAD_U16_SSE2(16, 16)
    SAD_U16_SSE2(16, 32)
    SAD_U16_SSE2(32, 8)
    SAD_U16_SSE2(32, 16)
    SAD_U16_SSE2(32, 32)
    SAD_U16_SSE2(32, 64)
    SAD_U16_SSE2(64, 16)
    SAD_U16_SSE2(64, 32)
    SAD_U16_SSE2(64, 64)
    SAD_U16_SSE2(64, 128)
    SAD_U16_SSE2(128, 32)
    SAD_U16_SSE2(128, 64)
    SAD_U16_SSE2(128, 128)
};

SADFunction selectSADFunction(unsigned width, unsigned height, unsigned bits, int opt, unsigned cpu) {
    SADFunction sad = sad_functions.at(KEY(width, height, bits, Scalar));

#if defined(MVTOOLS_X86)
    if (opt) {
        try {
            sad = sad_functions.at(KEY(width, height, bits, MMX));
        } catch (std::out_of_range &) { }

        if (cpu & X264_CPU_CACHELINE_64) {
            try {
                sad = sad_functions.at(KEY(width, height, bits, MMX_CACHE64));
            } catch (std::out_of_range &) { }
        }

        try {
            sad = sad_functions.at(KEY(width, height, bits, SSE2));
        } catch (std::out_of_range &) { }

        if (cpu & X264_CPU_SSE3) {
            try {
                sad = sad_functions.at(KEY(width, height, bits, SSE3));
            } catch (std::out_of_range &) { }
        }

        if ((cpu & X264_CPU_SSSE3) && (cpu & X264_CPU_CACHELINE_64)) {
            try {
                sad = sad_functions.at(KEY(width, height, bits, SSSE3_CACHE64));
            } catch (std::out_of_range &) { }
        }
    }
#endif

    return sad;
}

#undef SAD_X264_U8_MMX
#undef SAD_X264_U8_MMX_CACHE64
#undef SAD_X264_U8_SSE2
#undef SAD_X264_U8_SSE3
#undef SAD_X264_U8_SSSE3_CACHE64
#undef SAD_U8_SSE2
#undef SAD_U16_SSE2
#undef SAD



#define HADAMARD4(d0, d1, d2, d3, s0, s1, s2, s3) \
    {                                             \
        SumType2 t0 = s0 + s1;                    \
        SumType2 t1 = s0 - s1;                    \
        SumType2 t2 = s2 + s3;                    \
        SumType2 t3 = s2 - s3;                    \
        d0 = t0 + t2;                             \
        d2 = t0 - t2;                             \
        d1 = t1 + t3;                             \
        d3 = t1 - t3;                             \
    }

#define HADAMARD8(d0, d1, d2, d3, d4, d5, d6, d7, s0, s1, s2, s3, s4, s5, s6, s7) \
    {                                                                             \
        SumType2 t0 = s0 + s1;                                                    \
        SumType2 t1 = s0 - s1;                                                    \
        SumType2 t2 = s2 + s3;                                                    \
        SumType2 t3 = s2 - s3;                                                    \
        SumType2 t4 = s4 + s5;                                                    \
        SumType2 t5 = s4 - s5;                                                    \
        SumType2 t6 = s6 + s7;                                                    \
        SumType2 t7 = s6 - s7;                                                    \
        d0 = t0 + t4;                                                             \
        d2 = t0 - t4;                                                             \
        d1 = t1 + t5;                                                             \
        d3 = t1 - t5;                                                             \
        d4 = t2 + t6;                                                             \
        d5 = t2 - t6;                                                             \
        d6 = t3 + t7;                                                             \
        d7 = t3 - t7;                                                             \
    }

#define HADAMARD16(d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15) \
    {                                                                                                                                                          \
        SumType2 t0 = s0 + s1;                                                                                                                                 \
        SumType2 t1 = s0 - s1;                                                                                                                                 \
        SumType2 t2 = s2 + s3;                                                                                                                                 \
        SumType2 t3 = s2 - s3;                                                                                                                                 \
        SumType2 t4 = s4 + s5;                                                                                                                                 \
        SumType2 t5 = s4 - s5;                                                                                                                                 \
        SumType2 t6 = s6 + s7;                                                                                                                                 \
        SumType2 t7 = s6 - s7;                                                                                                                                 \
        SumType2 t8 = s8 + s9;                                                                                                                                 \
        SumType2 t9 = s8 - s9;                                                                                                                                 \
        SumType2 t10 = s10 + s11;                                                                                                                              \
        SumType2 t11 = s10 - s11;                                                                                                                              \
        SumType2 t12 = s12 + s13;                                                                                                                              \
        SumType2 t13 = s12 - s13;                                                                                                                              \
        SumType2 t14 = s14 + s15;                                                                                                                              \
        SumType2 t15 = s14 - s15;                                                                                                                              \
        d0 = t0 + t8;                                                                                                                                          \
        d2 = t0 - t8;                                                                                                                                          \
        d1 = t1 + t9;                                                                                                                                          \
        d3 = t1 - t9;                                                                                                                                          \
        d4 = t2 + t10;                                                                                                                                         \
        d5 = t2 - t10;                                                                                                                                         \
        d6 = t3 + t11;                                                                                                                                         \
        d7 = t3 - t11;                                                                                                                                         \
        d8 = t4 + t12;                                                                                                                                         \
        d9 = t4 - t12;                                                                                                                                         \
        d10 = t5 + t13;                                                                                                                                        \
        d11 = t5 - t13;                                                                                                                                        \
        d12 = t6 + t14;                                                                                                                                        \
        d13 = t6 - t14;                                                                                                                                        \
        d14 = t7 + t15;                                                                                                                                        \
        d15 = t7 - t15;                                                                                                                                        \
    }


// in: a pseudo-simd number of the form x+(y<<16)
// return: abs(x)+(abs(y)<<16)
template <typename SumType, typename SumType2>
static FORCE_INLINE SumType2 abs2(SumType2 a) {
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 s = ((a >> (bitsPerSum - 1)) & (((SumType2)1 << bitsPerSum) + 1)) * ((SumType)-1);
    return (a + s) ^ s;
}


template <typename PixelType, typename SumType, typename SumType2>
static FORCE_INLINE unsigned int Real_Satd_4x4_C(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 tmp[4][2];
    SumType2 a0, a1, a2, a3, b0, b1;
    SumType2 sum = 0;

    for (int i = 0; i < 4; i++) {
        const PixelType *pSrc = (const PixelType *)pSrc8;
        const PixelType *pRef = (const PixelType *)pRef8;

        a0 = pSrc[0] - pRef[0];
        a1 = pSrc[1] - pRef[1];
        b0 = (a0 + a1) + ((a0 - a1) << bitsPerSum);
        a2 = pSrc[2] - pRef[2];
        a3 = pSrc[3] - pRef[3];
        b1 = (a2 + a3) + ((a2 - a3) << bitsPerSum);
        tmp[i][0] = b0 + b1;
        tmp[i][1] = b0 - b1;

        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }

    for (int i = 0; i < 2; i++) {
        HADAMARD4(a0, a1, a2, a3, tmp[0][i], tmp[1][i], tmp[2][i], tmp[3][i]);
        a0 = abs2<SumType, SumType2>(a0) + abs2<SumType, SumType2>(a1) + abs2<SumType, SumType2>(a2) + abs2<SumType, SumType2>(a3);
        sum += ((SumType)a0) + (a0 >> bitsPerSum);
    }

    return (unsigned int)(sum >> 1);
}


template <typename PixelType>
static FORCE_INLINE unsigned int Satd_4x4_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (sizeof(PixelType) == 1)
        return Real_Satd_4x4_C<PixelType, uint16_t, uint32_t>(pSrc, nSrcPitch, pRef, nRefPitch);
    else
        return Real_Satd_4x4_C<PixelType, uint32_t, uint64_t>(pSrc, nSrcPitch, pRef, nRefPitch);
}


template <typename PixelType, typename SumType, typename SumType2>
static FORCE_INLINE unsigned int Real_Satd_8x4_C(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 tmp[4][4];
    SumType2 a0, a1, a2, a3;
    SumType2 sum = 0;

    for (int i = 0; i < 4; i++) {
        const PixelType *pSrc = (const PixelType *)pSrc8;
        const PixelType *pRef = (const PixelType *)pRef8;

        a0 = (pSrc[0] - pRef[0]) + ((SumType2)(pSrc[4] - pRef[4]) << bitsPerSum);
        a1 = (pSrc[1] - pRef[1]) + ((SumType2)(pSrc[5] - pRef[5]) << bitsPerSum);
        a2 = (pSrc[2] - pRef[2]) + ((SumType2)(pSrc[6] - pRef[6]) << bitsPerSum);
        a3 = (pSrc[3] - pRef[3]) + ((SumType2)(pSrc[7] - pRef[7]) << bitsPerSum);
        HADAMARD4(tmp[i][0], tmp[i][1], tmp[i][2], tmp[i][3], a0, a1, a2, a3);

        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }
    for (int i = 0; i < 4; i++) {
        HADAMARD4(a0, a1, a2, a3, tmp[0][i], tmp[1][i], tmp[2][i], tmp[3][i]);
        sum += abs2<SumType, SumType2>(a0) + abs2<SumType, SumType2>(a1) + abs2<SumType, SumType2>(a2) + abs2<SumType, SumType2>(a3);
    }

    return (unsigned int)((((SumType)sum) + (sum >> bitsPerSum)) >> 1);
}

template <typename PixelType>
static FORCE_INLINE unsigned int Satd_8x4_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (sizeof(PixelType) == 1)
        return Real_Satd_8x4_C<PixelType, uint16_t, uint32_t>(pSrc, nSrcPitch, pRef, nRefPitch);
    else
        return Real_Satd_8x4_C<PixelType, uint32_t, uint64_t>(pSrc, nSrcPitch, pRef, nRefPitch);
}

template <typename PixelType, typename SumType, typename SumType2>
static FORCE_INLINE unsigned int Real_Satd_8x8_C(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 tmp[8][4];
    SumType2 a0, a1, a2, a3, a4, a5, a6, a7, b0, b1, b2, b3;
    SumType2 sum = 0;

    for (int i = 0; i < 8; i++) {
        const PixelType *pSrc = (const PixelType *)pSrc8;
        const PixelType *pRef = (const PixelType *)pRef8;

        a0 = pSrc[0] - pRef[0];
        a1 = pSrc[1] - pRef[1];
        b0 = (a0 + a1) + ((a0 - a1) << bitsPerSum);
        a2 = pSrc[2] - pRef[2];
        a3 = pSrc[3] - pRef[3];
        b1 = (a2 + a3) + ((a2 - a3) << bitsPerSum);
        a4 = pSrc[4] - pRef[4];
        a5 = pSrc[5] - pRef[5];
        b2 = (a4 + a5) + ((a4 - a5) << bitsPerSum);
        a6 = pSrc[6] - pRef[6];
        a7 = pSrc[7] - pRef[7];
        b3 = (a6 + a7) + ((a6 - a7) << bitsPerSum);
        tmp[i][0] = b0 + b2;
        tmp[i][1] = b0 - b2;
        tmp[i][2] = b1 + b3;
        tmp[i][3] = b1 - b3;

        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }

    for (int i = 0; i < 4; i++) {
        HADAMARD8(a0, a1, a2, a3, a4, a5, a6, a7, tmp[0][i], tmp[1][i], tmp[2][i], tmp[3][i], tmp[4][i], tmp[5][i], tmp[6][i], tmp[7][i]);
        a0 = abs2<SumType, SumType2>(a0) + abs2<SumType, SumType2>(a1) + abs2<SumType, SumType2>(a2) + abs2<SumType, SumType2>(a3) + abs2<SumType, SumType2>(a4) + abs2<SumType, SumType2>(a5) + abs2<SumType, SumType2>(a6) + abs2<SumType, SumType2>(a7);
        sum += ((SumType)a0) + (a0 >> bitsPerSum);
    }

    return (unsigned int)(sum >> 1);
}

template <typename PixelType>
static FORCE_INLINE unsigned int Satd_8x8_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (sizeof(PixelType) == 1)
        return Real_Satd_8x8_C<PixelType, uint16_t, uint32_t>(pSrc, nSrcPitch, pRef, nRefPitch);
    else
        return Real_Satd_8x8_C<PixelType, uint32_t, uint64_t>(pSrc, nSrcPitch, pRef, nRefPitch);
}

template <typename PixelType, typename SumType, typename SumType2>
static FORCE_INLINE unsigned int Real_Satd_16x16_C(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 tmp[16][8];
    SumType2 a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, b0, b1, b2, b3, b4, b5, b6, b7;
    SumType2 sum = 0;

    for (int i = 0; i < 16; i++) {
        const PixelType *pSrc = (const PixelType *)pSrc8;
        const PixelType *pRef = (const PixelType *)pRef8;

        a0 = pSrc[0] - pRef[0];
        a1 = pSrc[1] - pRef[1];
        b0 = (a0 + a1) + ((a0 - a1) << bitsPerSum);
        a2 = pSrc[2] - pRef[2];
        a3 = pSrc[3] - pRef[3];
        b1 = (a2 + a3) + ((a2 - a3) << bitsPerSum);
        a4 = pSrc[4] - pRef[4];
        a5 = pSrc[5] - pRef[5];
        b2 = (a4 + a5) + ((a4 - a5) << bitsPerSum);
        a6 = pSrc[6] - pRef[6];
        a7 = pSrc[7] - pRef[7];
        b3 = (a6 + a7) + ((a6 - a7) << bitsPerSum);
        a8 = pSrc[8] - pRef[8];
        a9 = pSrc[9] - pRef[9];
        b4 = (a8 + a9) + ((a8 - a9) << bitsPerSum);
        a10 = pSrc[10] - pRef[10];
        a11 = pSrc[11] - pRef[11];
        b5 = (a10 + a11) + ((a10 - a11) << bitsPerSum);
        a12 = pSrc[12] - pRef[12];
        a13 = pSrc[13] - pRef[13];
        b6 = (a12 + a13) + ((a12 - a13) << bitsPerSum);
        a14 = pSrc[14] - pRef[14];
        a15 = pSrc[15] - pRef[15];
        b7 = (a14 + a15) + ((a14 - a15) << bitsPerSum);
        tmp[i][0] = b0 + b4;
        tmp[i][1] = b0 - b4;
        tmp[i][2] = b1 + b5;
        tmp[i][3] = b1 - b5;
        tmp[i][4] = b2 + b6;
        tmp[i][5] = b2 - b6;
        tmp[i][6] = b3 + b7;
        tmp[i][7] = b3 - b7;

        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }

    for (int i = 0; i < 8; i++) {
        HADAMARD16(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, tmp[0][i], tmp[1][i], tmp[2][i], tmp[3][i], tmp[4][i], tmp[5][i], tmp[6][i], tmp[7][i], tmp[8][i], tmp[9][i], tmp[10][i], tmp[11][i], tmp[12][i], tmp[13][i], tmp[14][i], tmp[15][i]);
        a0 = abs2<SumType, SumType2>(a0) + abs2<SumType, SumType2>(a1) + abs2<SumType, SumType2>(a2) + abs2<SumType, SumType2>(a3) + abs2<SumType, SumType2>(a4) + abs2<SumType, SumType2>(a5) + abs2<SumType, SumType2>(a6) + abs2<SumType, SumType2>(a7) + abs2<SumType, SumType2>(a8) + abs2<SumType, SumType2>(a9) + abs2<SumType, SumType2>(a10) + abs2<SumType, SumType2>(a11) + abs2<SumType, SumType2>(a12) + abs2<SumType, SumType2>(a13) + abs2<SumType, SumType2>(a14) + abs2<SumType, SumType2>(a15);
        sum += ((SumType)a0) + (a0 >> bitsPerSum);
    }

    return (unsigned int)(sum >> 1);
}

template <typename PixelType>
static FORCE_INLINE unsigned int Satd_16x16_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (sizeof(PixelType) == 1)
        return Real_Satd_16x16_C<PixelType, uint16_t, uint32_t>(pSrc, nSrcPitch, pRef, nRefPitch);
    else
        return Real_Satd_16x16_C<PixelType, uint32_t, uint64_t>(pSrc, nSrcPitch, pRef, nRefPitch);
}

// Only handles 4x4, 8x4, 8x8, 8x16, 16x8, and 16x16.
template <int nBlkWidth, int nBlkHeight, typename PixelType>
static FORCE_INLINE unsigned int Satd_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (nBlkWidth == 4 && nBlkHeight == 4)
        return Satd_4x4_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch);
    else if (nBlkWidth == 8 && nBlkHeight == 4)
        return Satd_8x4_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch);
    else if (nBlkWidth == 8 && nBlkHeight == 8)
        return Satd_8x8_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch);
    else if (nBlkWidth == 16 && nBlkHeight == 16)
        return Satd_16x16_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch);
    else {
        int bytesPerSample = sizeof(PixelType);

        unsigned int sum = Satd_8x4_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch) + Satd_8x4_C<PixelType>(pSrc + 4 * nSrcPitch, nSrcPitch, pRef + 4 * nRefPitch, nRefPitch);

        if (nBlkWidth == 16)
            sum += Satd_8x4_C<PixelType>(pSrc + 8 * bytesPerSample, nSrcPitch, pRef + 8 * bytesPerSample, nRefPitch) + Satd_8x4_C<PixelType>(pSrc + 8 * bytesPerSample + 4 * nSrcPitch, nSrcPitch, pRef + 8 * bytesPerSample + 4 * nSrcPitch, nRefPitch);

        if (nBlkHeight == 16)
            sum += Satd_8x4_C<PixelType>(pSrc + 8 * nSrcPitch, nSrcPitch, pRef + 8 * nRefPitch, nRefPitch) + Satd_8x4_C<PixelType>(pSrc + 12 * nSrcPitch, nSrcPitch, pRef + 12 * nRefPitch, nRefPitch);

        return sum;
    }
}


#if defined(MVTOOLS_X86)
#define SATD_X264_U8_MMX(width, height) \
    { KEY(width, height, 8, MMX), mvtools_pixel_satd_##width##x##height##_mmx2 },

#define SATD_X264_U8_SSE2(width, height) \
    { KEY(width, height, 8, SSE2), mvtools_pixel_satd_##width##x##height##_sse2 },

#define SATD_X264_U8_SSSE3(width, height) \
    { KEY(width, height, 8, SSSE3), mvtools_pixel_satd_##width##x##height##_ssse3 },

#define SATD_X264_U8_SSE4(width, height) \
    { KEY(width, height, 8, SSE4), mvtools_pixel_satd_##width##x##height##_sse4 },

#define SATD_X264_U8_AVX(width, height) \
    { KEY(width, height, 8, AVX), mvtools_pixel_satd_##width##x##height##_avx },

#define SATD_X264_U8_XOP(width, height) \
    { KEY(width, height, 8, XOP), mvtools_pixel_satd_##width##x##height##_xop },

#define SATD_X264_U8_AVX2(width, height) \
    { KEY(width, height, 8, AVX2), mvtools_pixel_satd_##width##x##height##_avx2 },

#else
#define SATD_X264_U8_MMX(width, height)
#define SATD_X264_U8_SSE2(width, height)
#define SATD_X264_U8_SSSE3(width, height)
#define SATD_X264_U8_SSE4(width, height)
#define SATD_X264_U8_AVX(width, height)
#define SATD_X264_U8_XOP(width, height)
#define SATD_X264_U8_AVX2(width, height)
#endif

#define SATD(width, height) \
    { KEY(width, height, 8, Scalar), Satd_C<width, height, uint8_t> }, \
    { KEY(width, height, 16, Scalar), Satd_C<width, height, uint16_t> }, \
    SATD_X264_U8_SSSE3(width, height) \
    SATD_X264_U8_SSE4(width, height) \
    SATD_X264_U8_AVX(width, height) \
    SATD_X264_U8_XOP(width, height)

static const std::unordered_map<uint32_t, SADFunction> satd_functions = {
    SATD(4, 4)
    SATD(8, 4)
    SATD(8, 8)
    SATD(16, 8)
    SATD(16, 16)
    SATD_X264_U8_MMX(4, 4)
    SATD_X264_U8_SSE2(8, 4)
    SATD_X264_U8_SSE2(8, 8)
    SATD_X264_U8_SSE2(16, 8)
    SATD_X264_U8_SSE2(16, 16)
    SATD_X264_U8_AVX2(8, 8)
    SATD_X264_U8_AVX2(16, 8)
    SATD_X264_U8_AVX2(16, 16)
};

SADFunction selectSATDFunction(unsigned width, unsigned height, unsigned bits, int opt, unsigned cpu) {
    SADFunction satd = satd_functions.at(KEY(width, height, bits, Scalar));

#if defined(MVTOOLS_X86)
    if (opt) {
        try {
            satd = satd_functions.at(KEY(width, height, bits, MMX));
        } catch (std::out_of_range &) { }

        try {
            satd = satd_functions.at(KEY(width, height, bits, SSE2));
        } catch (std::out_of_range &) { }

        if (cpu & X264_CPU_SSSE3) {
            try {
                satd = satd_functions.at(KEY(width, height, bits, SSSE3));
            } catch (std::out_of_range &) { }
        }

        if (cpu & X264_CPU_SSE4) {
            try {
                satd = satd_functions.at(KEY(width, height, bits, SSE4));
            } catch (std::out_of_range &) { }
        }

        if (cpu & X264_CPU_AVX) {
            try {
                satd = satd_functions.at(KEY(width, height, bits, AVX));
            } catch (std::out_of_range &) { }
        }

        if (cpu & X264_CPU_XOP) {
            try {
                satd = satd_functions.at(KEY(width, height, bits, XOP));
            } catch (std::out_of_range &) { }
        }

        if (cpu & X264_CPU_AVX2) {
            try {
                satd = satd_functions.at(KEY(width, height, bits, AVX2));
            } catch (std::out_of_range &) { }
        }
    }
#endif

    return satd;
}

#undef SATD_X264_U8_MMX
#undef SATD_X264_U8_SSE2
#undef SATD_X264_U8_SSSE3
#undef SATD_X264_U8_SSE4
#undef SATD_X264_U8_AVX
#undef SATD_X264_U8_XOP
#undef SATD_X264_U8_AVX2
#undef SATD

#undef KEY
