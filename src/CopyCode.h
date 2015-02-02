#ifndef __COPYCODE_H__
#define __COPYCODE_H__

#include <cstring>


typedef void (*COPYFunction)(uint8_t *pDst, intptr_t nDstPitch,
        const uint8_t *pSrc, intptr_t nSrcPitch);

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


#endif
