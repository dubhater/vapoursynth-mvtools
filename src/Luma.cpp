
#include <cstdint>
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


#if defined(MVTOOLS_X86)

#define DECLARE_LUMA(width, height, bits, opt) extern "C" unsigned int mvtools_luma_##width##x##height##_u##bits##_##opt(const uint8_t *pSrc, intptr_t nSrcPitch);

DECLARE_LUMA(4, 4, 8, sse2)
DECLARE_LUMA(8, 4, 8, sse2)
DECLARE_LUMA(8, 8, 8, sse2)
DECLARE_LUMA(16, 2, 8, sse2)
DECLARE_LUMA(16, 8, 8, sse2)
DECLARE_LUMA(16, 16, 8, sse2)
DECLARE_LUMA(32, 16, 8, sse2)
DECLARE_LUMA(32, 32, 8, sse2)

#undef DECLARE_LUMA

#endif // MVTOOLS_X86


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86)
#define LUMA_SSE2(width, height) \
    { KEY(width, height, 8, SSE2), mvtools_luma_##width##x##height##_u8_sse2 },
#else
#define LUMA_SSE2(width, height)
#endif

#define LUMA(width, height) \
    { KEY(width, height, 8, Scalar), luma_c<width, height, uint8_t> }, \
    { KEY(width, height, 16, Scalar), luma_c<width, height, uint16_t> }, \
    LUMA_SSE2(width, height)

static const std::unordered_map<uint32_t, LUMAFunction> luma_functions = {
    LUMA(4, 4)
    LUMA(8, 4)
    LUMA(8, 8)
    LUMA(16, 2)
    LUMA(16, 8)
    LUMA(16, 16)
    LUMA(32, 16)
    LUMA(32, 32)
};

LUMAFunction selectLumaFunction(unsigned width, unsigned height, unsigned bits, int opt) {
    LUMAFunction luma = luma_functions.at(KEY(width, height, bits, Scalar));

#if defined(MVTOOLS_X86)
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
