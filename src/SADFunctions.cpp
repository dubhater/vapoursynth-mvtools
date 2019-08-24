#include <cstdlib>
#include <stdexcept>
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


// Doesn't handle 16x2 blocks.
template <int nBlkWidth, int nBlkHeight, typename PixelType>
static unsigned int Satd_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (nBlkWidth == 4 && nBlkHeight == 4)
        return Satd_4x4_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch);
    else {
        const int bytesPerSample = sizeof(PixelType);
        const int partition_width = 8;
        const int partition_height = 4;

        unsigned sum = 0;

        for (int y = 0; y < nBlkHeight; y += partition_height) {
            for (int x = 0; x < nBlkWidth; x += partition_width)
                sum += Satd_8x4_C<PixelType>(pSrc + x * bytesPerSample, nSrcPitch,
                                             pRef + x * bytesPerSample, nRefPitch);

            pSrc += nSrcPitch * partition_height;
            pRef += nRefPitch * partition_height;
        }

        return sum;
    }
}


template <unsigned nBlkWidth, unsigned nBlkHeight, InstructionSets opt>
static unsigned int Satd_SIMD(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    const unsigned partition_width = 16;
    const unsigned partition_height = 16;

    unsigned sum = 0;

    for (unsigned y = 0; y < nBlkHeight; y += partition_height) {
        for (unsigned x = 0; x < nBlkWidth; x += partition_width) {
            if (opt == SSE2)
                sum += mvtools_pixel_satd_16x16_sse2(pSrc + x, nSrcPitch, pRef + x, nRefPitch);
            else if (opt == SSSE3)
                sum += mvtools_pixel_satd_16x16_ssse3(pSrc + x, nSrcPitch, pRef + x, nRefPitch);
            else if (opt == SSE4)
                sum += mvtools_pixel_satd_16x16_sse4(pSrc + x, nSrcPitch, pRef + x, nRefPitch);
            else if (opt == AVX)
                sum += mvtools_pixel_satd_16x16_avx(pSrc + x, nSrcPitch, pRef + x, nRefPitch);
            else if (opt == XOP)
                sum += mvtools_pixel_satd_16x16_xop(pSrc + x, nSrcPitch, pRef + x, nRefPitch);
            else if (opt == AVX2)
                sum += mvtools_pixel_satd_16x16_avx2(pSrc + x, nSrcPitch, pRef + x, nRefPitch);
        }

        pSrc += nSrcPitch * partition_height;
        pRef += nRefPitch * partition_height;
    }

    return sum;
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
    { KEY(width, height, 16, Scalar), Satd_C<width, height, uint16_t> },

#define SATD_X264_U8(width, height) \
    SATD_X264_U8_SSSE3(width, height) \
    SATD_X264_U8_SSE4(width, height) \
    SATD_X264_U8_AVX(width, height) \
    SATD_X264_U8_XOP(width, height)

#define SATD_U8_SIMD(width, height) \
    { KEY(width, height, 8, SSE2), Satd_SIMD<width, height, SSE2> }, \
    { KEY(width, height, 8, SSSE3), Satd_SIMD<width, height, SSSE3> }, \
    { KEY(width, height, 8, SSE4), Satd_SIMD<width, height, SSE4> }, \
    { KEY(width, height, 8, AVX), Satd_SIMD<width, height, AVX> }, \
    { KEY(width, height, 8, XOP), Satd_SIMD<width, height, XOP> }, \
    { KEY(width, height, 8, AVX2), Satd_SIMD<width, height, AVX2> },

static const std::unordered_map<uint32_t, SADFunction> satd_functions = {
    SATD(4, 4)
    SATD(8, 4)
    SATD(8, 8)
    SATD(16, 8)
    SATD(16, 16)
    SATD(32, 16)
    SATD(32, 32)
    SATD(64, 32)
    SATD(64, 64)
    SATD(128, 64)
    SATD(128, 128)
    SATD_X264_U8(4, 4)
    SATD_X264_U8(8, 4)
    SATD_X264_U8(8, 8)
    SATD_X264_U8(16, 8)
    SATD_X264_U8(16, 16)
    SATD_X264_U8_MMX(4, 4)
    SATD_X264_U8_SSE2(8, 4)
    SATD_X264_U8_SSE2(8, 8)
    SATD_X264_U8_SSE2(16, 8)
    SATD_X264_U8_SSE2(16, 16)
    SATD_X264_U8_AVX2(8, 8)
    SATD_X264_U8_AVX2(16, 8)
    SATD_X264_U8_AVX2(16, 16)
    SATD_U8_SIMD(32, 16)
    SATD_U8_SIMD(32, 32)
    SATD_U8_SIMD(64, 32)
    SATD_U8_SIMD(64, 64)
    SATD_U8_SIMD(128, 64)
    SATD_U8_SIMD(128, 128)
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
