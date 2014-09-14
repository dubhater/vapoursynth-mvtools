#ifndef __OVERLAP__
#define __OVERLAP__

#include "math.h"
//#define M_PI       3.14159265358979323846f

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

typedef void (OverlapsFunction)(unsigned short *pDst, int nDstPitch,
                            const unsigned char *pSrc, int nSrcPitch,
							short *pWin, int nWinPitch);

//=============================================================
// short
template <int blockWidth, int blockHeight>
void Overlaps_C(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch)
{
	// pWin from 0 to 2048
	for (int j=0; j<blockHeight; j++)
	{
	    for (int i=0; i<blockWidth; i++)
	    {
            pDst[i] = ( pDst[i] + ((pSrc[i]*pWin[i]+256)>>6));
	    }
		pDst += nDstPitch;
		pSrc += nSrcPitch;
		pWin += nWinPitch;
	}
}

extern "C" void Overlaps32x32_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps16x32_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps32x16_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps16x16_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps8x16_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps8x8_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps4x8_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps4x4_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps2x4_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps2x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps8x4_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps4x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps16x8_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps16x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps8x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);
extern "C" void Overlaps8x1_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch);

void Short2Bytes(unsigned char *pDst, int nDstPitch, unsigned short *pDstShort, int dstShortPitch, int nWidth, int nHeight);

void LimitChanges_c(unsigned char *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, int nLimit);
extern "C" void LimitChanges_mmx(unsigned char *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, int nLimit);

inline int DegrainWeight(int thSAD, int blockSAD) // not reaaly related to overlap, but common to MDegrainX functions
{
    	int a = ( (thSAD - blockSAD) > 0 ? (thSAD - blockSAD) : 0 ) * (thSAD + blockSAD);
    	int b = a < (1<<23) ? (a<<8)/(thSAD*thSAD + blockSAD*blockSAD)  // small a
    	                    : a/((thSAD*thSAD + blockSAD*blockSAD)>>8); // very large a, prevent overflow
    	return b;
}

#endif
