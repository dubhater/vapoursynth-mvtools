#ifndef __COPYCODE_H__
#define __COPYCODE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef void (*COPYFunction)(uint8_t *pDst, intptr_t nDstPitch,
                             const uint8_t *pSrc, intptr_t nSrcPitch);


#define DECLARE_COPY(width, height, bits) void mvtools_copy_##width##x##height##_u##bits##_c(uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch);

DECLARE_COPY(2, 2, 8)
DECLARE_COPY(2, 4, 8)
DECLARE_COPY(4, 2, 8)
DECLARE_COPY(4, 4, 8)
DECLARE_COPY(4, 8, 8)
DECLARE_COPY(8, 1, 8)
DECLARE_COPY(8, 2, 8)
DECLARE_COPY(8, 4, 8)
DECLARE_COPY(8, 8, 8)
DECLARE_COPY(8, 16, 8)
DECLARE_COPY(16, 1, 8)
DECLARE_COPY(16, 2, 8)
DECLARE_COPY(16, 4, 8)
DECLARE_COPY(16, 8, 8)
DECLARE_COPY(16, 16, 8)
DECLARE_COPY(16, 32, 8)
DECLARE_COPY(32, 8, 8)
DECLARE_COPY(32, 16, 8)
DECLARE_COPY(32, 32, 8)

DECLARE_COPY(2, 2, 16)
DECLARE_COPY(2, 4, 16)
DECLARE_COPY(4, 2, 16)
DECLARE_COPY(4, 4, 16)
DECLARE_COPY(4, 8, 16)
DECLARE_COPY(8, 1, 16)
DECLARE_COPY(8, 2, 16)
DECLARE_COPY(8, 4, 16)
DECLARE_COPY(8, 8, 16)
DECLARE_COPY(8, 16, 16)
DECLARE_COPY(16, 1, 16)
DECLARE_COPY(16, 2, 16)
DECLARE_COPY(16, 4, 16)
DECLARE_COPY(16, 8, 16)
DECLARE_COPY(16, 16, 16)
DECLARE_COPY(16, 32, 16)
DECLARE_COPY(32, 8, 16)
DECLARE_COPY(32, 16, 16)
DECLARE_COPY(32, 32, 16)

#undef DECLARE_COPY


#ifdef __cplusplus
} // extern "C"
#endif

#endif
