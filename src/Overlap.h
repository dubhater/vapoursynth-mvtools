#ifndef __OVERLAP__
#define __OVERLAP__

#include <math.h>
#include <stdint.h>
#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b)            (((a) < (b)) ? (b) : (a))
#endif

// top, middle, botom and left, middle, right windows
#define OW_TL 0
#define OW_TM 1
#define OW_TR 2
#define OW_ML 3
#define OW_MM 4
#define OW_MR 5
#define OW_BL 6
#define OW_BM 7
#define OW_BR 8

class OverlapWindows
{
    int nx; // window sizes
    int ny;
    int ox; // overap sizes
    int oy;
    int size; // full window size= nx*ny

    short * Overlap9Windows;

    float *fWin1UVx;
    float *fWin1UVxfirst;
    float *fWin1UVxlast;
    float *fWin1UVy;
    float *fWin1UVyfirst;
    float *fWin1UVylast;
    public :

    OverlapWindows(int _nx, int _ny, int _ox, int _oy);
    ~OverlapWindows();

    inline int Getnx() const { return nx; }
    inline int Getny() const { return ny; }
    inline int GetSize() const { return size; }
    inline short *GetWindow(int i) const { return Overlap9Windows + size*i; }
};

typedef void (*OverlapsFunction)(uint8_t *pDst, intptr_t nDstPitch,
        const uint8_t *pSrc, intptr_t nSrcPitch,
        short *pWin, intptr_t nWinPitch);


template <int blockWidth, int blockHeight, typename PixelType2, typename PixelType>
void Overlaps_C(uint8_t *pDst8, intptr_t nDstPitch, const uint8_t *pSrc8, intptr_t nSrcPitch, short *pWin, intptr_t nWinPitch)
{
    // pWin from 0 to 2048
    for (int j=0; j<blockHeight; j++)
    {
        for (int i=0; i<blockWidth; i++)
        {
            PixelType2 *pDst = (PixelType2 *)pDst8;
            const PixelType *pSrc = (const PixelType *)pSrc8;

            pDst[i] += ((pSrc[i] * pWin[i]) >> 6);
        }
        pDst8 += nDstPitch;
        pSrc8 += nSrcPitch;
        pWin += nWinPitch;
    }
}

#define MK_CFUNC(functionname) extern "C" void functionname (uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch, short *pWin, intptr_t nWinPitch)

MK_CFUNC(mvtools_Overlaps2x2_sse2);
MK_CFUNC(mvtools_Overlaps2x4_sse2);
MK_CFUNC(mvtools_Overlaps4x2_sse2);
MK_CFUNC(mvtools_Overlaps4x4_sse2);
MK_CFUNC(mvtools_Overlaps4x8_sse2);
MK_CFUNC(mvtools_Overlaps8x1_sse2);
MK_CFUNC(mvtools_Overlaps8x2_sse2);
MK_CFUNC(mvtools_Overlaps8x4_sse2);
MK_CFUNC(mvtools_Overlaps8x8_sse2);
MK_CFUNC(mvtools_Overlaps8x16_sse2);
MK_CFUNC(mvtools_Overlaps16x1_sse2);
MK_CFUNC(mvtools_Overlaps16x2_sse2);
MK_CFUNC(mvtools_Overlaps16x4_sse2);
MK_CFUNC(mvtools_Overlaps16x8_sse2);
MK_CFUNC(mvtools_Overlaps16x16_sse2);
MK_CFUNC(mvtools_Overlaps16x32_sse2);
MK_CFUNC(mvtools_Overlaps32x8_sse2);
MK_CFUNC(mvtools_Overlaps32x16_sse2);
MK_CFUNC(mvtools_Overlaps32x32_sse2);

#undef MK_CFUNC


typedef void (*ToPixelsFunction)(uint8_t *pDst, int nDstPitch,
        const uint8_t *pSrc, int nSrcPitch,
        int width, int height, int bitsPerSample);


template <typename PixelType2, typename PixelType>
void ToPixels(uint8_t *pDst8, int nDstPitch, const uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int bitsPerSample)
{
    int pixelMax = (1 << bitsPerSample) - 1;

    for (int h=0; h<nHeight; h++)
    {
        for (int i=0; i<nWidth; i++)
        {
            const PixelType2 *pSrc = (const PixelType2 *)pSrc8;
            PixelType *pDst = (PixelType *)pDst8;

            int a = (pSrc[i] + 16)>>5;
            if (sizeof(PixelType) == 1)
                pDst[i] = a | ((255-a) >> (sizeof(int)*8-1));
            else
                pDst[i] = min(pixelMax, a);
        }
        pDst8 += nDstPitch;
        pSrc8 += nSrcPitch;
    }
}


#endif
