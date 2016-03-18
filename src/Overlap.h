#ifndef __OVERLAP__
#define __OVERLAP__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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

typedef struct OverlapWindows {
    int nx; // window sizes
    int ny;
    int ox; // overap sizes
    int oy;
    int size; // full window size= nx*ny

    int16_t *Overlap9Windows;

    float *fWin1UVx;
    float *fWin1UVxfirst;
    float *fWin1UVxlast;
    float *fWin1UVy;
    float *fWin1UVyfirst;
    float *fWin1UVylast;
} OverlapWindows;

void overInit(OverlapWindows *over, int nx, int ny, int ox, int oy);

void overDeinit(OverlapWindows *over);

int16_t *overGetWindow(const OverlapWindows *over, int i);


typedef void (*OverlapsFunction)(uint8_t *pDst, intptr_t nDstPitch,
                                 const uint8_t *pSrc, intptr_t nSrcPitch,
                                 int16_t *pWin, intptr_t nWinPitch);


#define MK_CFUNC(functionname) void functionname(uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch, int16_t *pWin, intptr_t nWinPitch)

MK_CFUNC(mvtools_overlaps_2x2_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_2x4_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_4x2_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_4x4_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_4x8_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_8x1_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_8x2_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_8x4_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_8x8_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_8x16_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_16x1_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_16x2_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_16x4_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_16x8_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_16x16_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_16x32_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_32x8_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_32x16_uint16_t_uint8_t_c);
MK_CFUNC(mvtools_overlaps_32x32_uint16_t_uint8_t_c);

MK_CFUNC(mvtools_overlaps_2x2_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_2x4_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_4x2_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_4x4_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_4x8_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_8x1_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_8x2_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_8x4_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_8x8_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_8x16_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_16x1_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_16x2_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_16x4_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_16x8_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_16x16_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_16x32_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_32x8_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_32x16_uint32_t_uint16_t_c);
MK_CFUNC(mvtools_overlaps_32x32_uint32_t_uint16_t_c);

#if defined(MVTOOLS_X86)
MK_CFUNC(mvtools_overlaps_2x2_sse2);
MK_CFUNC(mvtools_overlaps_2x4_sse2);
MK_CFUNC(mvtools_overlaps_4x2_sse2);
MK_CFUNC(mvtools_overlaps_4x4_sse2);
MK_CFUNC(mvtools_overlaps_4x8_sse2);
MK_CFUNC(mvtools_overlaps_8x1_sse2);
MK_CFUNC(mvtools_overlaps_8x2_sse2);
MK_CFUNC(mvtools_overlaps_8x4_sse2);
MK_CFUNC(mvtools_overlaps_8x8_sse2);
MK_CFUNC(mvtools_overlaps_8x16_sse2);
MK_CFUNC(mvtools_overlaps_16x1_sse2);
MK_CFUNC(mvtools_overlaps_16x2_sse2);
MK_CFUNC(mvtools_overlaps_16x4_sse2);
MK_CFUNC(mvtools_overlaps_16x8_sse2);
MK_CFUNC(mvtools_overlaps_16x16_sse2);
MK_CFUNC(mvtools_overlaps_16x32_sse2);
MK_CFUNC(mvtools_overlaps_32x8_sse2);
MK_CFUNC(mvtools_overlaps_32x16_sse2);
MK_CFUNC(mvtools_overlaps_32x32_sse2);
#endif

#undef MK_CFUNC


typedef void (*ToPixelsFunction)(uint8_t *pDst, int nDstPitch,
                                 const uint8_t *pSrc, int nSrcPitch,
                                 int width, int height, int bitsPerSample);

void ToPixels_uint16_t_uint8_t(uint8_t *pDst8, int nDstPitch, const uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int bitsPerSample);
void ToPixels_uint32_t_uint16_t(uint8_t *pDst8, int nDstPitch, const uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int bitsPerSample);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
