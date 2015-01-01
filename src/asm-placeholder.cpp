#include <stdint.h>
#include <stdlib.h>


// XXX From Interpolation.h
//
// TODO: Not yet ported
//extern "C" void  RB2F_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
//                       int nSrcPitch, int nWidth, int nHeight){ abort(); }
//extern "C" void  RB2FilteredVerticalLine_SSE(unsigned char *pDst, const unsigned char *pSrc, int nSrcPitch, int nWidthMMX){ abort(); }
//extern "C" void  RB2FilteredHorizontalInplaceLine_SSE(unsigned char *pSrc, int nWidthMMX){ abort(); }
//extern "C" void  VerticalBicubic_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
//                                int nSrcPitch, int nWidth, int nHeight){ abort(); }
//extern "C" void  HorizontalBicubic_iSSE(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
//                                int nSrcPitch, int nWidth, int nHeight){ abort(); }


// XXX From SADFunctions.h
//
#define MK_CFUNC(functionname) extern "C" unsigned int  functionname (const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch) { abort(); }
//dummy for testing and deactivate SAD
MK_CFUNC(SadDummy);
#undef MK_CFUNC
