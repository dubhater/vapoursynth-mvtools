#ifndef __VARIANCE_H__
#define __VARIANCE_H__

#include <cstdint>


typedef unsigned int (*LUMAFunction)(const uint8_t *pSrc, intptr_t nSrcPitch);


template<int nBlkWidth, int nBlkHeight, typename PixelType>
unsigned int Luma_C(const uint8_t *pSrc8, intptr_t nSrcPitch)
{
    unsigned int meanLuma = 0;
    for ( int j = 0; j < nBlkHeight; j++ )
    {
        for ( int i = 0; i < nBlkWidth; i++ ) {
            const PixelType *pSrc = (const PixelType *)pSrc8;
            meanLuma += pSrc[i];
        }
        pSrc8 += nSrcPitch;
    }
    return meanLuma;
}


extern "C" unsigned int mvtools_Luma32x32_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x32_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma32x16_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x16_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma8x8_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma4x4_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma8x4_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x8_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x2_sse2(const uint8_t *pSrc, intptr_t nSrcPitch);

#endif
