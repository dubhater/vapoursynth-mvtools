
#include <stdint.h>


#define DEFINE_LUMA(width, height, bits) \
unsigned int mvtools_luma_##width##x##height##_u##bits##_c(const uint8_t *pSrc8, intptr_t nSrcPitch) { \
    unsigned int meanLuma = 0;                      \
    for (int j = 0; j < height; j++) {              \
        for (int i = 0; i < width; i++) {           \
            const uint##bits##_t *pSrc = (const uint##bits##_t *)pSrc8;     \
            meanLuma += pSrc[i];                    \
        }                                           \
        pSrc8 += nSrcPitch;                         \
    }                                               \
    return meanLuma;                                \
}

DEFINE_LUMA(4, 4, 8)
DEFINE_LUMA(8, 4, 8)
DEFINE_LUMA(8, 8, 8)
DEFINE_LUMA(16, 2, 8)
DEFINE_LUMA(16, 8, 8)
DEFINE_LUMA(16, 16, 8)
DEFINE_LUMA(32, 16, 8)
DEFINE_LUMA(32, 32, 8)

DEFINE_LUMA(4, 4, 16)
DEFINE_LUMA(8, 4, 16)
DEFINE_LUMA(8, 8, 16)
DEFINE_LUMA(16, 2, 16)
DEFINE_LUMA(16, 8, 16)
DEFINE_LUMA(16, 16, 16)
DEFINE_LUMA(32, 16, 16)
DEFINE_LUMA(32, 32, 16)
