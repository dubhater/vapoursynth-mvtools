#ifndef __MV_DEGRAINS__
#define __MV_DEGRAINS__

#include <VSHelper.h>

#include "MVInterface.h"


extern "C" void mvtools_LimitChanges_sse2(unsigned char *pDst, intptr_t nDstPitch, const unsigned char *pSrc, intptr_t nSrcPitch, intptr_t nWidth, intptr_t nHeight, intptr_t nLimit);


inline void LimitChanges_c(unsigned char *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, int nLimit) {
    for (int h = 0; h < nHeight; h++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = VSMIN( VSMAX(pDst[i], (pSrc[i] - nLimit)), (pSrc[i] + nLimit));
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}


inline int DegrainWeight(int thSAD, int blockSAD) {
    int a = ( (thSAD - blockSAD) > 0 ? (thSAD - blockSAD) : 0 ) * (thSAD + blockSAD);
    int b = a < (1 << 23) ? (a << 8) / (thSAD * thSAD + blockSAD * blockSAD)  // small a
                        : a / ((thSAD * thSAD + blockSAD * blockSAD) >> 8); // very large a, prevent overflow
    return b;
}


inline void useBlock(const uint8_t * &p, int &np, int &WRef, bool isUsable, const MVClipBalls &mvclip, int i, MVPlane **pPlane, const uint8_t **pSrcCur, int xx, const int *nSrcPitch, int nLogPel, int plane, int xSubUV, int ySubUV, const int *thSAD) {
    if (isUsable) {
        const FakeBlockData &block = mvclip.GetBlock(0, i);
        int blx = (block.GetX() << nLogPel) + block.GetMV().x;
        int bly = (block.GetY() << nLogPel) + block.GetMV().y;
        p = pPlane[plane]->GetPointer(plane ? blx >> xSubUV : blx, plane ? bly >> ySubUV : bly);
        np = pPlane[plane]->GetPitch();
        int blockSAD = block.GetSAD();
        WRef = DegrainWeight(thSAD[plane], blockSAD);
    } else {
        p = pSrcCur[plane] + xx;
        np = nSrcPitch[plane];
        WRef = 0;
    }
}

#endif
