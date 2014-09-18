#ifndef __COPYCODE_H__
#define __COPYCODE_H__


#include <VSHelper.h>

inline void BitBlt(unsigned char* dstp, int dst_pitch, const unsigned char* srcp, int src_pitch, int row_size, int height, bool isse) {
    vs_bitblt(dstp, dst_pitch, srcp, src_pitch, row_size, height);
}

#if 0
void asm_BitBlt_ISSE(unsigned char* dstp, int dst_pitch, const unsigned char* srcp, int src_pitch, int row_size, int height);
void memcpy_amd(void *dest, const void *src, size_t n);
#endif
void MemZoneSet(unsigned char *ptr, unsigned char value, int width,
				int height, int offsetX, int offsetY, int pitch);

typedef void (COPYFunction)(unsigned char *pDst, int nDstPitch,
                            const unsigned char *pSrc, int nSrcPitch);

template<int nBlkWidth, int nBlkHeight>
void Copy_C(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch)
{
   for ( int j = 0; j < nBlkHeight; j++ )
   {
//      for ( int i = 0; i < nBlkWidth; i++ )  //  waste cycles removed by Fizick in v1.2
         memcpy(pDst, pSrc, nBlkWidth);
      pDst += nDstPitch;
      pSrc += nSrcPitch;
   }
}

template<int nBlkSize>
void Copy_C(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch)
{
   Copy_C<nBlkSize, nBlkSize>(pDst, nDstPitch, pSrc, nSrcPitch);
}

//extern "C" void  Copy16x16_mmx(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch);
#define MK_CFUNC(functionname) extern "C" void  functionname (uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch)
//default functions
MK_CFUNC(mvtools_Copy32x32_sse2);
MK_CFUNC(mvtools_Copy16x32_sse2);
MK_CFUNC(mvtools_Copy32x16_sse2);
MK_CFUNC(mvtools_Copy16x16_sse2);
MK_CFUNC(mvtools_Copy16x8_sse2);
MK_CFUNC(mvtools_Copy16x2_sse2);
MK_CFUNC(mvtools_Copy8x16_sse2);
MK_CFUNC(mvtools_Copy8x8_sse2);
MK_CFUNC(mvtools_Copy8x4_sse2);
MK_CFUNC(mvtools_Copy8x2_sse2);
MK_CFUNC(mvtools_Copy8x1_sse2);
MK_CFUNC(mvtools_Copy4x8_sse2);
MK_CFUNC(mvtools_Copy4x4_sse2);
MK_CFUNC(mvtools_Copy4x2_sse2);
MK_CFUNC(mvtools_Copy2x4_sse2);
MK_CFUNC(mvtools_Copy2x2_sse2);
MK_CFUNC(mvtools_Copy2x1_sse2);
#undef MK_CFUNC

#endif
