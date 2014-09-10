#include <stdlib.h>

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

