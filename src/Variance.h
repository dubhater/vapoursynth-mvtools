#ifndef __VARIANCE_H__
#define __VARIANCE_H__

inline unsigned int VARABS(int x) { return x < 0 ? -x : x; }

typedef unsigned int (VARFunction)(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);

template<int nBlkWidth, int nBlkHeight>
unsigned int Var_C(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma)
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


extern "C" unsigned int mvtools_Var32x32_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var16x32_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var32x16_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var16x16_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var8x8_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var4x4_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var8x4_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var16x8_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);
extern "C" unsigned int mvtools_Var16x2_sse2(const unsigned char *pSrc, intptr_t nSrcPitch, int *pLuma);

typedef unsigned int (*LUMAFunction)(const unsigned char *pSrc, intptr_t nSrcPitch);

template<int nBlkWidth, int nBlkHeight>
unsigned int Luma_C(const unsigned char *pSrc, intptr_t nSrcPitch)
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


extern "C" unsigned int mvtools_Luma32x32_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x32_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma32x16_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x16_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma8x8_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma4x4_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma8x4_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x8_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);
extern "C" unsigned int mvtools_Luma16x2_sse2(const unsigned char *pSrc, intptr_t nSrcPitch);

#endif
