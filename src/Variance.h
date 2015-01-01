#ifndef __VARIANCE_H__
#define __VARIANCE_H__

typedef unsigned int (*LUMAFunction)(const unsigned char *pSrc, intptr_t nSrcPitch);

template<int nBlkWidth, int nBlkHeight>
unsigned int Luma_C(const unsigned char *pSrc, intptr_t nSrcPitch)
{
    const unsigned char *s = pSrc;
    int meanLuma = 0;
    for ( int j = 0; j < nBlkHeight; j++ )
    {
        for ( int i = 0; i < nBlkWidth; i++ )
            meanLuma += s[i];
        s += nSrcPitch;
    }
    return meanLuma;
}


extern "C" unsigned int mvtools_Luma32x32_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x32_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma32x16_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x16_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma8x8_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma4x4_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma8x4_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x8_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x2_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);

#endif
