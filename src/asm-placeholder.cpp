#include <stdint.h>
#include <stdlib.h>


// XXX From Interpolation.h
//
extern "C" void  VerticalBilin_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight) { abort(); }
extern "C" void  HorizontalBilin_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                                  int nSrcPitch, int nWidth, int nHeight){ abort(); }
extern "C" void  DiagonalBilin_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight){ abort(); }

extern "C" void  RB2F_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                       int nSrcPitch, int nWidth, int nHeight){ abort(); }

extern "C" void  RB2CubicHorizontalInplaceLine_SSE(unsigned char *pSrc, int nWidthMMX){ abort(); }
extern "C" void  RB2CubicVerticalLine_SSE(unsigned char *pDst, const unsigned char *pSrc, int nSrcPitch, int nWidthMMX){ abort(); }
extern "C" void  RB2QuadraticHorizontalInplaceLine_SSE(unsigned char *pSrc, int nWidthMMX){ abort(); }
extern "C" void  RB2QuadraticVerticalLine_SSE(unsigned char *pDst, const unsigned char *pSrc, int nSrcPitch, int nWidthMMX){ abort(); }
extern "C" void  RB2FilteredVerticalLine_SSE(unsigned char *pDst, const unsigned char *pSrc, int nSrcPitch, int nWidthMMX){ abort(); }
extern "C" void  RB2FilteredHorizontalInplaceLine_SSE(unsigned char *pSrc, int nWidthMMX){ abort(); }
extern "C" void  RB2BilinearFilteredVerticalLine_SSE(unsigned char *pDst, const unsigned char *pSrc, int nSrcPitch, int nWidthMMX){ abort(); }
extern "C" void  RB2BilinearFilteredHorizontalInplaceLine_SSE(unsigned char *pSrc, int nWidthMMX){ abort(); }


extern "C" void  VerticalWiener_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight){ abort(); }
extern "C" void  HorizontalWiener_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight){ abort(); }


extern "C" void  VerticalBicubic_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight){ abort(); }
extern "C" void  HorizontalBicubic_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight){ abort(); }

extern "C" void Average2_iSSE(unsigned char *pDst, const unsigned char *pSrc1, const unsigned char *pSrc2, int nPitch, int nWidth, int nHeight){ abort(); }


// XXX From CopyCode.h
//
#define MK_CFUNC(functionname) extern "C" void  functionname (uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch) { abort(); }
//default functions
MK_CFUNC(Copy32x32_mmx);
MK_CFUNC(Copy16x32_mmx);
MK_CFUNC(Copy32x16_mmx);
MK_CFUNC(Copy16x16_mmx);
MK_CFUNC(Copy16x8_mmx);
MK_CFUNC(Copy16x2_mmx);
MK_CFUNC(Copy8x16_mmx);
MK_CFUNC(Copy8x8_mmx);
MK_CFUNC(Copy8x4_mmx);
MK_CFUNC(Copy8x2_mmx);
MK_CFUNC(Copy8x1_mmx);
MK_CFUNC(Copy4x8_mmx);
MK_CFUNC(Copy4x4_mmx);
MK_CFUNC(Copy4x2_mmx);
MK_CFUNC(Copy2x4_mmx);
MK_CFUNC(Copy2x2_mmx);
MK_CFUNC(Copy2x1_mmx);
#undef MK_CFUNC


// XXX From Variance.h
//
extern "C" unsigned int  Var32x32_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var16x32_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var32x16_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var16x16_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var8x8_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var4x4_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var8x4_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var16x8_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Var16x2_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma) { abort(); }
extern "C" unsigned int  Luma32x32_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma16x32_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma32x16_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma16x16_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma8x8_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma4x4_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma8x4_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma16x8_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }
extern "C" unsigned int  Luma16x2_iSSE(const unsigned char *pSrc, int nSrcPitch) { abort(); }


// XXX From SADFunctions.h
//
#define MK_CFUNC(functionname) extern "C" unsigned int  functionname (const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch) { abort(); }

#define SAD_ISSE(blsizex, blsizey) extern "C" unsigned int  Sad##blsizex##x##blsizey##_iSSE(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch) { abort(); }
//Sad16x16_iSSE( x,y can be: 32 16 8 4 2
SAD_ISSE(32,32);
SAD_ISSE(16,32);
SAD_ISSE(32,16);
SAD_ISSE(16,16);
SAD_ISSE(16,8);
SAD_ISSE(16,2);
SAD_ISSE(16,1);
SAD_ISSE(8,16);
SAD_ISSE(8,8);
SAD_ISSE(8,4);
SAD_ISSE(8,2);
SAD_ISSE(8,1);
SAD_ISSE(4,8);
SAD_ISSE(4,4);
SAD_ISSE(4,2);
SAD_ISSE(2,4);
SAD_ISSE(2,2);

MK_CFUNC(Sad2x2_iSSE_T); //only work with packed, aligned source block copy
MK_CFUNC(Sad2x4_iSSE_T);

#undef SAD_ISSE


#define SAD_x264(blsizex, blsizey) extern "C" unsigned int  x264_pixel_sad_##blsizex##x##blsizey##_mmxext(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch) { abort(); }
//x264_pixel_sad_16x16_mmxext(   x,y can be: 16 8 4
SAD_x264(16,16);
SAD_x264(16,8);
SAD_x264(8,16);
SAD_x264(8,8);
SAD_x264(8,4);
SAD_x264(4,8);
SAD_x264(4,4);
#undef SAD_x264
//parameter is function name
MK_CFUNC(x264_pixel_sad_16x16_sse2); //non optimized cache access, for AMD?
MK_CFUNC(x264_pixel_sad_16x8_sse2);	 //non optimized cache access, for AMD?
MK_CFUNC(x264_pixel_sad_16x16_sse3); //LDDQU Pentium4E (Core1?), not for Core2!
MK_CFUNC(x264_pixel_sad_16x8_sse3);  //LDDQU Pentium4E (Core1?), not for Core2!
MK_CFUNC(x264_pixel_sad_16x16_cache64_sse2);//core2 optimized
MK_CFUNC(x264_pixel_sad_16x8_cache64_sse2);//core2 optimized
MK_CFUNC(x264_pixel_sad_16x16_cache64_ssse3);//core2 optimized
MK_CFUNC(x264_pixel_sad_16x8_cache64_ssse3); //core2 optimized

MK_CFUNC(x264_pixel_sad_16x16_cache32_mmxext);
MK_CFUNC(x264_pixel_sad_16x8_cache32_mmxext);
MK_CFUNC(x264_pixel_sad_16x16_cache64_mmxext);
MK_CFUNC(x264_pixel_sad_16x8_cache64_mmxext);
MK_CFUNC(x264_pixel_sad_8x16_cache32_mmxext);
MK_CFUNC(x264_pixel_sad_8x8_cache32_mmxext);
MK_CFUNC(x264_pixel_sad_8x4_cache32_mmxext);
MK_CFUNC(x264_pixel_sad_8x16_cache64_mmxext);
MK_CFUNC(x264_pixel_sad_8x8_cache64_mmxext);
MK_CFUNC(x264_pixel_sad_8x4_cache64_mmxext);

//1.9.5.3: added ssd & SATD (TSchniede)
/* alternative to SAD - SSD: squared sum of differences, VERY sensitive to noise */
MK_CFUNC(x264_pixel_ssd_16x16_mmx);
MK_CFUNC(x264_pixel_ssd_16x8_mmx);
MK_CFUNC(x264_pixel_ssd_8x16_mmx);
MK_CFUNC(x264_pixel_ssd_8x8_mmx);
MK_CFUNC(x264_pixel_ssd_8x4_mmx);
MK_CFUNC(x264_pixel_ssd_4x8_mmx);
MK_CFUNC(x264_pixel_ssd_4x4_mmx);

/* SATD: Sum of Absolute Transformed Differences, more sensitive to noise, frequency domain based - replacement to dct/SAD */
MK_CFUNC(x264_pixel_satd_16x16_mmxext);
MK_CFUNC(x264_pixel_satd_16x8_mmxext);
MK_CFUNC(x264_pixel_satd_8x16_mmxext);
MK_CFUNC(x264_pixel_satd_8x8_mmxext);
MK_CFUNC(x264_pixel_satd_8x4_mmxext);
MK_CFUNC(x264_pixel_satd_4x8_mmxext);
MK_CFUNC(x264_pixel_satd_4x4_mmxext);

#define SATD_SSE2(blsizex, blsizey) extern "C" unsigned int  x264_pixel_satd_##blsizex##x##blsizey##_sse2(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch) { abort(); }
#define SATD_SSSE3(blsizex, blsizey) extern "C" unsigned int  x264_pixel_satd_##blsizex##x##blsizey##_ssse3(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch) { abort(); }
#define SATD_SSSE3_PHADD(blsizex, blsizey) extern "C" unsigned int  x264_pixel_satd_##blsizex##x##blsizey##_ssse3_phadd(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch) { abort(); }

//x264_pixel_satd_16x16_%1
SATD_SSE2(16, 16);
SATD_SSE2(16,  8);
SATD_SSE2( 8, 16);
SATD_SSE2( 8,  8);
SATD_SSE2( 8,  4);
SATD_SSSE3(16, 16);
SATD_SSSE3(16,  8);
SATD_SSSE3( 8, 16);
SATD_SSSE3( 8,  8);
SATD_SSSE3( 8,  4);
SATD_SSSE3_PHADD(16, 16); //identical to ssse3, for Penryn useful only?
SATD_SSSE3_PHADD(16,  8); //identical to ssse3
SATD_SSSE3_PHADD( 8, 16);
SATD_SSSE3_PHADD( 8,  8);
SATD_SSSE3_PHADD( 8,  4);
#undef SATD_SSE2
#undef SATD_SSSE3
#undef SATD_SSSE3_PHADD

//dummy for testing and deactivate SAD
MK_CFUNC(SadDummy);
#undef MK_CFUNC


// XXX From overlap.h
extern "C" void Overlaps32x32_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps16x32_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps32x16_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps16x16_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps8x16_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps8x8_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps4x8_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps4x4_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps2x4_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps2x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps8x4_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps4x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps16x8_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps16x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps8x2_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
extern "C" void Overlaps8x1_mmx(unsigned short *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, short *pWin, int nWinPitch) { abort(); }
