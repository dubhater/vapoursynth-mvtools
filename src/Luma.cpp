#include <cstdint>
#include <stdexcept>
#include <unordered_map>

#include "Luma.h"


enum InstructionSets {
    Scalar,
    SSE2,
};


template <unsigned width, unsigned height, typename PixelType>
unsigned int luma_c(const uint8_t *pSrc8, intptr_t nSrcPitch) {
    unsigned int meanLuma = 0;
    for (unsigned j = 0; j < height; j++) {
        for (unsigned i = 0; i < width; i++) {
            const PixelType *pSrc = (const PixelType *)pSrc8;
            meanLuma += pSrc[i];
        }
        pSrc8 += nSrcPitch;
    }
    return meanLuma;
}


#if defined(MVTOOLS_X86) || defined(MVTOOLS_ARM)

#if defined(MVTOOLS_ARM)
#include "sse2neon.h"
#else
#include <emmintrin.h>
#endif

#define zeroes _mm_setzero_si128()


template <unsigned width, unsigned height>
unsigned int luma_sse2(const uint8_t *pSrc, intptr_t nSrcPitch) {
    __m128i sum = zeroes;

    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x += 16) {
            __m128i src;
            if (width == 4)
                src = _mm_cvtsi32_si128(*(const int *)pSrc);
            else if (width == 8)
                src = _mm_loadl_epi64((const __m128i *)pSrc);
            else
                src = _mm_loadu_si128((const __m128i *)&pSrc[x]);

            sum = _mm_add_epi64(sum, _mm_sad_epu8(src, zeroes));
        }

        pSrc += nSrcPitch;
    }

    if (width >= 16)
        sum = _mm_add_epi64(sum, _mm_srli_si128(sum, 8));

    return (unsigned)_mm_cvtsi128_si32(sum);
}


#undef zeroes


#endif // MVTOOLS_X86


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86)  || defined(MVTOOLS_ARM)
#define LUMA_SSE2(width, height) \
    { KEY(width, height, 8, SSE2), luma_sse2<width, height> },
#else
#define LUMA_SSE2(width, height)
#endif

#define LUMA(width, height) \
    { KEY(width, height, 8, Scalar), luma_c<width, height, uint8_t> }, \
    { KEY(width, height, 16, Scalar), luma_c<width, height, uint16_t> },

static const std::unordered_map<uint32_t, LUMAFunction> luma_functions = {
    LUMA(4, 4)
    LUMA(8, 4)
    LUMA(8, 8)
    LUMA(16, 2)
    LUMA(16, 8)
    LUMA(16, 16)
    LUMA(32, 16)
    LUMA(32, 32)
    LUMA(64, 32)
    LUMA(64, 64)
    LUMA(128, 64)
    LUMA(128, 128)
    LUMA_SSE2(4, 4)
    LUMA_SSE2(8, 4)
    LUMA_SSE2(8, 8)
    LUMA_SSE2(16, 2)
    LUMA_SSE2(16, 8)
    LUMA_SSE2(16, 16)
    LUMA_SSE2(32, 16)
    LUMA_SSE2(32, 32)
    LUMA_SSE2(64, 32)
    LUMA_SSE2(64, 64)
    LUMA_SSE2(128, 64)
    LUMA_SSE2(128, 128)
};

LUMAFunction selectLumaFunction(unsigned width, unsigned height, unsigned bits, int opt) {
    LUMAFunction luma = luma_functions.at(KEY(width, height, bits, Scalar));

#if defined(MVTOOLS_X86) || defined(MVTOOLS_ARM)
    if (opt) {
        try {
            luma = luma_functions.at(KEY(width, height, bits, SSE2));
        } catch (std::out_of_range &) { }
    }
#endif

    return luma;
}

#undef LUMA
#undef LUMA_SSE2
#undef KEY
