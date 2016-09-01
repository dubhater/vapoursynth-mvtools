#ifndef LUMA_H
#define LUMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef unsigned int (*LUMAFunction)(const uint8_t *pSrc, intptr_t nSrcPitch);


LUMAFunction selectLumaFunction(unsigned width, unsigned height, unsigned bits, int opt);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // LUMA_H
