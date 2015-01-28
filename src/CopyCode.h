#ifndef __COPYCODE_H__
#define __COPYCODE_H__

#include <cstring>


typedef void (*COPYFunction)(unsigned char *pDst, intptr_t nDstPitch,
        const unsigned char *pSrc, intptr_t nSrcPitch);

template<int nBlkWidth, int nBlkHeight, typename PixelType>
void Copy_C(uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch)
{
    for ( int j = 0; j < nBlkHeight; j++ )
    {
        memcpy(pDst, pSrc, nBlkWidth * sizeof(PixelType));
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}

#define MK_CFUNC(functionname) extern "C" void  functionname (uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch)
MK_CFUNC(mvtools_Copy2x1_sse2);
MK_CFUNC(mvtools_Copy2x2_sse2);
MK_CFUNC(mvtools_Copy2x4_sse2);
MK_CFUNC(mvtools_Copy4x2_sse2);
MK_CFUNC(mvtools_Copy4x4_sse2);
MK_CFUNC(mvtools_Copy4x8_sse2);
MK_CFUNC(mvtools_Copy8x1_sse2);
MK_CFUNC(mvtools_Copy8x2_sse2);
MK_CFUNC(mvtools_Copy8x4_sse2);
MK_CFUNC(mvtools_Copy8x8_sse2);
MK_CFUNC(mvtools_Copy8x16_sse2);
MK_CFUNC(mvtools_Copy16x1_sse2);
MK_CFUNC(mvtools_Copy16x2_sse2);
MK_CFUNC(mvtools_Copy16x4_sse2);
MK_CFUNC(mvtools_Copy16x8_sse2);
MK_CFUNC(mvtools_Copy16x16_sse2);
MK_CFUNC(mvtools_Copy16x32_sse2);
MK_CFUNC(mvtools_Copy32x8_sse2);
MK_CFUNC(mvtools_Copy32x16_sse2);
MK_CFUNC(mvtools_Copy32x32_sse2);
#undef MK_CFUNC

#endif
