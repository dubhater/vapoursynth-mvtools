
#include <stdint.h>
#include <string.h>


#define DEFINE_COPY(width, height, bits) \
void mvtools_copy_##width##x##height##_u##bits##_c(uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch) { \
    for (int j = 0; j < (height); j++) {                    \
        memcpy(pDst, pSrc, (width) * ((bits) / 8));         \
        pDst += nDstPitch;                                  \
        pSrc += nSrcPitch;                                  \
    }                                                       \
}

DEFINE_COPY(2, 2, 8)
DEFINE_COPY(2, 4, 8)
DEFINE_COPY(4, 2, 8)
DEFINE_COPY(4, 4, 8)
DEFINE_COPY(4, 8, 8)
DEFINE_COPY(8, 1, 8)
DEFINE_COPY(8, 2, 8)
DEFINE_COPY(8, 4, 8)
DEFINE_COPY(8, 8, 8)
DEFINE_COPY(8, 16, 8)
DEFINE_COPY(16, 1, 8)
DEFINE_COPY(16, 2, 8)
DEFINE_COPY(16, 4, 8)
DEFINE_COPY(16, 8, 8)
DEFINE_COPY(16, 16, 8)
DEFINE_COPY(16, 32, 8)
DEFINE_COPY(32, 8, 8)
DEFINE_COPY(32, 16, 8)
DEFINE_COPY(32, 32, 8)

DEFINE_COPY(2, 2, 16)
DEFINE_COPY(2, 4, 16)
DEFINE_COPY(4, 2, 16)
DEFINE_COPY(4, 4, 16)
DEFINE_COPY(4, 8, 16)
DEFINE_COPY(8, 1, 16)
DEFINE_COPY(8, 2, 16)
DEFINE_COPY(8, 4, 16)
DEFINE_COPY(8, 8, 16)
DEFINE_COPY(8, 16, 16)
DEFINE_COPY(16, 1, 16)
DEFINE_COPY(16, 2, 16)
DEFINE_COPY(16, 4, 16)
DEFINE_COPY(16, 8, 16)
DEFINE_COPY(16, 16, 16)
DEFINE_COPY(16, 32, 16)
DEFINE_COPY(32, 8, 16)
DEFINE_COPY(32, 16, 16)
DEFINE_COPY(32, 32, 16)

#undef DEFINE_COPY
