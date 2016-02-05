#ifndef __VARIANCE_H__
#define __VARIANCE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef unsigned int (*LUMAFunction)(const uint8_t *pSrc, intptr_t nSrcPitch);


#define DECLARE_LUMA(width, height, bits, opt) unsigned int mvtools_luma_##width##x##height##_u##bits##_##opt(const uint8_t *pSrc, intptr_t nSrcPitch);

DECLARE_LUMA(4, 4, 8, c)
DECLARE_LUMA(8, 4, 8, c)
DECLARE_LUMA(8, 8, 8, c)
DECLARE_LUMA(16, 2, 8, c)
DECLARE_LUMA(16, 8, 8, c)
DECLARE_LUMA(16, 16, 8, c)
DECLARE_LUMA(32, 16, 8, c)
DECLARE_LUMA(32, 32, 8, c)

DECLARE_LUMA(4, 4, 16, c)
DECLARE_LUMA(8, 4, 16, c)
DECLARE_LUMA(8, 8, 16, c)
DECLARE_LUMA(16, 2, 16, c)
DECLARE_LUMA(16, 8, 16, c)
DECLARE_LUMA(16, 16, 16, c)
DECLARE_LUMA(32, 16, 16, c)
DECLARE_LUMA(32, 32, 16, c)

DECLARE_LUMA(4, 4, 8, sse2)
DECLARE_LUMA(8, 4, 8, sse2)
DECLARE_LUMA(8, 8, 8, sse2)
DECLARE_LUMA(16, 2, 8, sse2)
DECLARE_LUMA(16, 8, 8, sse2)
DECLARE_LUMA(16, 16, 8, sse2)
DECLARE_LUMA(32, 16, 8, sse2)
DECLARE_LUMA(32, 32, 8, sse2)

#undef DECLARE_LUMA


#ifdef __cplusplus
} // extern "C"
#endif

#endif
