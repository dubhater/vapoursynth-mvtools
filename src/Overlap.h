#ifndef OVERLAP_H
#define OVERLAP_H

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


typedef void (*ToPixelsFunction)(uint8_t *pDst, int nDstPitch,
                                 const uint8_t *pSrc, int nSrcPitch,
                                 int width, int height, int bitsPerSample);

void ToPixels_uint16_t_uint8_t(uint8_t *pDst8, int nDstPitch, const uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int bitsPerSample);
void ToPixels_uint32_t_uint16_t(uint8_t *pDst8, int nDstPitch, const uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int bitsPerSample);

OverlapsFunction selectOverlapsFunction(unsigned width, unsigned height, unsigned bits, int opt);

#if defined(MVTOOLS_X86)
OverlapsFunction selectOverlapsFunctionAVX2(unsigned width, unsigned height, unsigned bits);
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OVERLAP_H
