#ifndef __VARIANCE_H__
#define __VARIANCE_H__

inline unsigned int VARABS(int x) { return x < 0 ? -x : x; }

typedef unsigned int (VARFunction)(const unsigned char *pSrc, int nSrcPitch, int *pLuma);

template<int nBlkWidth, int nBlkHeight>
unsigned int Var_C(const unsigned char *pSrc, int nSrcPitch, int *pLuma)
{
   const unsigned char *s = pSrc;
   int meanLuma = 0;
   int meanVariance = 0;
   for ( int j = 0; j < nBlkHeight; j++ )
   {
      for ( int i = 0; i < nBlkWidth; i++ )
         meanLuma += s[i];
      s += nSrcPitch;
   }
   *pLuma = meanLuma;
   meanLuma = (meanLuma + ((nBlkWidth * nBlkHeight) >> 1)) / (nBlkWidth * nBlkHeight);
   s = pSrc;
   for ( int j = 0; j < nBlkHeight; j++ )
   {
      for ( int i = 0; i < nBlkWidth; i++ )
         meanVariance += VARABS(s[i] - meanLuma);
      s += nSrcPitch;
   }
   return meanVariance;
}

template<int nBlkSize>
unsigned int Var_C(const unsigned char *pSrc, int nSrcPitch, int *pLuma)
{
   return Var_C<nBlkSize, nBlkSize>(pSrc, nSrcPitch, pLuma);
}

extern "C" unsigned int  Var32x32_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var16x32_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var32x16_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var16x16_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var8x8_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var4x4_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var8x4_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var16x8_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);
extern "C" unsigned int  Var16x2_iSSE(const unsigned char *pSrc, int nSrcPitch, int *pLuma);

typedef unsigned int (LUMAFunction)(const unsigned char *pSrc, int nSrcPitch);

template<int nBlkWidth, int nBlkHeight>
unsigned int Luma_C(const unsigned char *pSrc, int nSrcPitch)
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

template<int nBlkSize>
unsigned int Luma_C(const unsigned char *pSrc, int nSrcPitch)
{
   return Luma_C<nBlkSize, nBlkSize>(pSrc, nSrcPitch);
}

extern "C" unsigned int  Luma32x32_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma16x32_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma32x16_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma16x16_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma8x8_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma4x4_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma8x4_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma16x8_iSSE(const unsigned char *pSrc, int nSrcPitch);
extern "C" unsigned int  Luma16x2_iSSE(const unsigned char *pSrc, int nSrcPitch);

#endif
