// Functions that interpolates a frame

// See legal notice in Copying.txt for more information

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .


#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include <stdint.h>


/* TODO: port these
   extern "C" void  VerticalBicubic_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nWidth, intptr_t nHeight);
   extern "C" void  HorizontalBicubic_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nWidth, intptr_t nHeight);
   extern "C" void  RB2F_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nSrcPitch, intptr_t nWidth, intptr_t nHeight);
   extern "C" void  RB2FilteredVerticalLine_SSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
   extern "C" void  RB2FilteredHorizontalInplaceLine_SSE(uint8_t *pSrc, intptr_t nWidthMMX);
   */

void mvtools_Average2_sse2(uint8_t *pDst, const uint8_t *pSrc1, const uint8_t *pSrc2, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight);

void mvtools_VerticalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                   intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);
void mvtools_HorizontalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                     intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);
void mvtools_DiagonalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                   intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);


void mvtools_RB2CubicHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX);
void mvtools_RB2CubicVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
void mvtools_RB2QuadraticHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX);
void mvtools_RB2QuadraticVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
void mvtools_RB2BilinearFilteredVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
void mvtools_RB2BilinearFilteredHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX);
void mvtools_VerticalWiener_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                 intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);
void mvtools_HorizontalWiener_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                   intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);


#define VerticalBilinear(PixelType) \
static void VerticalBilinear_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, \
                      intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) { \
    PixelType *pDst = (PixelType *)pDst8;                       \
    PixelType *pSrc = (PixelType *)pSrc8;                       \
                                                                \
    nPitch /= sizeof(PixelType);                                \
                                                                \
    for (int j = 0; j < nHeight - 1; j++) {                     \
        for (int i = 0; i < nWidth; i++)                        \
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;    \
        pDst += nPitch;                                         \
        pSrc += nPitch;                                         \
    }                                                           \
    /* last row */                                              \
    for (int i = 0; i < nWidth; i++)                            \
        pDst[i] = pSrc[i];                                      \
}

VerticalBilinear(uint8_t)
VerticalBilinear(uint16_t)


#define HorizontalBilinear(PixelType) \
static void HorizontalBilinear_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, \
                        intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nPitch /= sizeof(PixelType); \
 \
    for (int j = 0; j < nHeight; j++) { \
        for (int i = 0; i < nWidth - 1; i++) \
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1; \
 \
        pDst[nWidth - 1] = pSrc[nWidth - 1]; \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
}

HorizontalBilinear(uint8_t)
HorizontalBilinear(uint16_t)


#define DiagonalBilinear(PixelType) \
static void DiagonalBilinear_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, \
                      intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nPitch /= sizeof(PixelType); \
 \
    for (int j = 0; j < nHeight - 1; j++) { \
        for (int i = 0; i < nWidth - 1; i++) \
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nPitch] + pSrc[i + nPitch + 1] + 2) >> 2; \
 \
        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nPitch - 1] + 1) >> 1; \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
    for (int i = 0; i < nWidth - 1; i++) \
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1; \
    pDst[nWidth - 1] = pSrc[nWidth - 1]; \
}

DiagonalBilinear(uint8_t)
DiagonalBilinear(uint16_t)


#define RB2F_C(PixelType) \
static void RB2F_C_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch, \
            int nSrcPitch, int nWidth, int nHeight, int isse) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nDstPitch /= sizeof(PixelType); \
    nSrcPitch /= sizeof(PixelType); \
 \
    for (int y = 0; y < nHeight; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] \
                    + pSrc[x * 2 + nSrcPitch + 1] + pSrc[x * 2 + nSrcPitch] + 2) / 4; \
 \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
}

RB2F_C(uint8_t)
RB2F_C(uint16_t)


//  Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
#define RB2FilteredVertical(PixelType) \
static void RB2FilteredVertical_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch, \
                         int nSrcPitch, int nWidth, int nHeight, int isse) { \
    /* int nWidthMMX = (nWidth/4)*4; */ \
 \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nDstPitch /= sizeof(PixelType); \
    nSrcPitch /= sizeof(PixelType); \
 \
    for (int y = 0; y < 1; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2; \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
 \
    /* TODO: port the asm
       if (isse && nWidthMMX>=4)
       {
       for ( int y = 1; y < nHeight; y++ )
       {
       RB2FilteredVerticalLine_SSE((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);

       for ( int x = nWidthMMX; x < nWidth; x++ )
       pDst[x] = (pSrc[x-nSrcPitch] + pSrc[x]*2 + pSrc[x+nSrcPitch] + 2) /4;

       pDst += nDstPitch;
       pSrc += nSrcPitch * 2;
       }
       }
       else
       */ \
    { \
        for (int y = 1; y < nHeight; y++) { \
            for (int x = 0; x < nWidth; x++) \
                pDst[x] = (pSrc[x - nSrcPitch] + pSrc[x] * 2 + pSrc[x + nSrcPitch] + 2) / 4; \
 \
            pDst += nDstPitch; \
            pSrc += nSrcPitch * 2; \
        } \
    } \
}

RB2FilteredVertical(uint8_t)
RB2FilteredVertical(uint16_t)


// Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
#define RB2FilteredHorizontalInplace(PixelType) \
static void RB2FilteredHorizontalInplace_##PixelType(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int isse) { \
    /* int nWidthMMX = 1 + ((nWidth-2)/4)*4; */ \
 \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nSrcPitch /= sizeof(PixelType); \
 \
    for (int y = 0; y < nHeight; y++) { \
        int x = 0; \
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; \
 \
        /* TODO: port the asm
           if (isse)
           {
           RB2FilteredHorizontalInplaceLine_SSE((uint8_t *)pSrc, nWidthMMX); // very first is skipped
           for ( x = nWidthMMX; x < nWidth; x++ )
           pSrc[x] = (pSrc[x*2-1] + pSrc[x*2]*2 + pSrc[x*2+1] + 2) /4;
           }
           else
           */ \
        { \
            for (x = 1; x < nWidth; x++) \
                pSrc[x] = (pSrc[x * 2 - 1] + pSrc[x * 2] * 2 + pSrc[x * 2 + 1] + 2) / 4; \
        } \
        pSrc[0] = pSrc0; \
 \
        pSrc += nSrcPitch; \
    } \
}

RB2FilteredHorizontalInplace(uint8_t)
RB2FilteredHorizontalInplace(uint16_t)


// separable Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
#define RB2Filtered(PixelType) \
static void RB2Filtered_##PixelType(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch, \
                 int nSrcPitch, int nWidth, int nHeight, int isse) { \
    RB2FilteredVertical_##PixelType(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); /* intermediate half height */ \
    RB2FilteredHorizontalInplace_##PixelType(pDst, nDstPitch, nWidth, nHeight, isse);             /* inpace width reduction */ \
}

RB2Filtered(uint8_t)
RB2Filtered(uint16_t)


//  BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
#define RB2BilinearFilteredVertical(PixelType) \
static void RB2BilinearFilteredVertical_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch, \
                                 int nSrcPitch, int nWidth, int nHeight, int isse) { \
 \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nDstPitch /= sizeof(PixelType); \
    nSrcPitch /= sizeof(PixelType); \
 \
    int nWidthMMX = (nWidth / 8) * 8; \
 \
    for (int y = 0; y < 1 && y < nHeight; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2; \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
 \
    if (sizeof(PixelType) == 1 && isse && nWidthMMX >= 8) { \
        for (int y = 1; y < nHeight - 1; y++) { \
            mvtools_RB2BilinearFilteredVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX); \
 \
            for (int x = nWidthMMX; x < nWidth; x++) \
                pDst[x] = (pSrc[x - nSrcPitch] + pSrc[x] * 3 + pSrc[x + nSrcPitch] * 3 + pSrc[x + nSrcPitch * 2] + 4) / 8; \
 \
            pDst += nDstPitch; \
            pSrc += nSrcPitch * 2; \
        } \
    } else { \
        for (int y = 1; y < nHeight - 1; y++) { \
            for (int x = 0; x < nWidth; x++) \
                pDst[x] = (pSrc[x - nSrcPitch] + pSrc[x] * 3 + pSrc[x + nSrcPitch] * 3 + pSrc[x + nSrcPitch * 2] + 4) / 8; \
 \
            pDst += nDstPitch; \
            pSrc += nSrcPitch * 2; \
        } \
    } \
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2; \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
}

RB2BilinearFilteredVertical(uint8_t)
RB2BilinearFilteredVertical(uint16_t)


// BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
#define RB2BilinearFilteredHorizontalInplace(PixelType) \
static void RB2BilinearFilteredHorizontalInplace_##PixelType(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int isse) { \
 \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nSrcPitch /= sizeof(PixelType); \
 \
    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8; \
 \
    for (int y = 0; y < nHeight; y++) { \
        int x = 0; \
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; \
 \
        if (sizeof(PixelType) == 1 && isse) { \
            mvtools_RB2BilinearFilteredHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX); /* very first is skipped */ \
            for (x = nWidthMMX; x < nWidth - 1; x++) \
                pSrc[x] = (pSrc[x * 2 - 1] + pSrc[x * 2] * 3 + pSrc[x * 2 + 1] * 3 + pSrc[x * 2 + 2] + 4) / 8; \
        } else { \
            for (x = 1; x < nWidth - 1; x++) \
                pSrc[x] = (pSrc[x * 2 - 1] + pSrc[x * 2] * 3 + pSrc[x * 2 + 1] * 3 + pSrc[x * 2 + 2] + 4) / 8; \
        } \
        pSrc[0] = pSrc0; \
 \
        for (x = max(nWidth - 1, 1); x < nWidth; x++) \
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; \
 \
        pSrc += nSrcPitch; \
    } \
}

RB2BilinearFilteredHorizontalInplace(uint8_t)
RB2BilinearFilteredHorizontalInplace(uint16_t)


// separable BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
#define RB2BilinearFiltered(PixelType) \
static void RB2BilinearFiltered_##PixelType(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch, \
                         int nSrcPitch, int nWidth, int nHeight, int isse) { \
    RB2BilinearFilteredVertical_##PixelType(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); /* intermediate half height */ \
    RB2BilinearFilteredHorizontalInplace_##PixelType(pDst, nDstPitch, nWidth, nHeight, isse);             /* inpace width reduction */ \
}

RB2BilinearFiltered(uint8_t)
RB2BilinearFiltered(uint16_t)


// filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
#define RB2QuadraticVertical(PixelType) \
static void RB2QuadraticVertical_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch, \
                          int nSrcPitch, int nWidth, int nHeight, int isse) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nDstPitch /= sizeof(PixelType); \
    nSrcPitch /= sizeof(PixelType); \
 \
    int nWidthMMX = (nWidth / 8) * 8; \
 \
    for (int y = 0; y < 1 && y < nHeight; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2; \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
 \
    if (sizeof(PixelType) == 1 && isse && nWidthMMX >= 8) { \
        for (int y = 1; y < nHeight - 1; y++) { \
            mvtools_RB2QuadraticVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX); \
 \
            for (int x = nWidthMMX; x < nWidth; x++) \
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 9 + pSrc[x] * 22 + \
                           pSrc[x + nSrcPitch] * 22 + pSrc[x + nSrcPitch * 2] * 9 + pSrc[x + nSrcPitch * 3] + 32) / 64; \
 \
            pDst += nDstPitch; \
            pSrc += nSrcPitch * 2; \
        } \
    } else { \
 \
        for (int y = 1; y < nHeight - 1; y++) { \
            for (int x = 0; x < nWidth; x++) \
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 9 + pSrc[x] * 22 + \
                           pSrc[x + nSrcPitch] * 22 + pSrc[x + nSrcPitch * 2] * 9 + pSrc[x + nSrcPitch * 3] + 32) / 64; \
 \
            pDst += nDstPitch; \
            pSrc += nSrcPitch * 2; \
        } \
    } \
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2; \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
}

RB2QuadraticVertical(uint8_t)
RB2QuadraticVertical(uint16_t)


// filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
#define RB2QuadraticHorizontalInplace(PixelType) \
static void RB2QuadraticHorizontalInplace_##PixelType(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int isse) { \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nSrcPitch /= sizeof(PixelType); \
 \
    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8; \
 \
    for (int y = 0; y < nHeight; y++) { \
        int x = 0; \
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; /* store temporary */ \
 \
        if (sizeof(PixelType) == 1 && isse) { \
            mvtools_RB2QuadraticHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX); \
            for (x = nWidthMMX; x < nWidth - 1; x++) \
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 9 + pSrc[x * 2] * 22 + pSrc[x * 2 + 1] * 22 + pSrc[x * 2 + 2] * 9 + pSrc[x * 2 + 3] + 32) / 64; \
        } else { \
            for (x = 1; x < nWidth - 1; x++) \
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 9 + pSrc[x * 2] * 22 + pSrc[x * 2 + 1] * 22 + pSrc[x * 2 + 2] * 9 + pSrc[x * 2 + 3] + 32) / 64; \
        } \
        pSrc[0] = pSrc0; \
 \
        for (x = max(nWidth - 1, 1); x < nWidth; x++) \
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; \
 \
        pSrc += nSrcPitch; \
    } \
}

RB2QuadraticHorizontalInplace(uint8_t)
RB2QuadraticHorizontalInplace(uint16_t)


// separable filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
#define RB2Quadratic(PixelType) \
static void RB2Quadratic_##PixelType(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch, \
                  int nSrcPitch, int nWidth, int nHeight, int isse) { \
    RB2QuadraticVertical_##PixelType(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); /* intermediate half height */ \
    RB2QuadraticHorizontalInplace_##PixelType(pDst, nDstPitch, nWidth, nHeight, isse);             /* inpace width reduction */ \
}

RB2Quadratic(uint8_t)
RB2Quadratic(uint16_t)


// filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
#define RB2CubicVertical(PixelType) \
static void RB2CubicVertical_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch, \
                      int nSrcPitch, int nWidth, int nHeight, int isse) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nDstPitch /= sizeof(PixelType); \
    nSrcPitch /= sizeof(PixelType); \
 \
    int nWidthMMX = (nWidth / 8) * 8; \
    for (int y = 0; y < 1 && y < nHeight; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2; \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
 \
    if (sizeof(PixelType) == 1 && isse && nWidthMMX >= 8) { \
        for (int y = 1; y < nHeight - 1; y++) { \
            mvtools_RB2CubicVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX); \
 \
            for (int x = nWidthMMX; x < nWidth; x++) \
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 5 + pSrc[x] * 10 + \
                           pSrc[x + nSrcPitch] * 10 + pSrc[x + nSrcPitch * 2] * 5 + pSrc[x + nSrcPitch * 3] + 16) / 32; \
 \
            pDst += nDstPitch; \
            pSrc += nSrcPitch * 2; \
        } \
    } else { \
        for (int y = 1; y < nHeight - 1; y++) { \
            for (int x = 0; x < nWidth; x++) \
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 5 + pSrc[x] * 10 + \
                           pSrc[x + nSrcPitch] * 10 + pSrc[x + nSrcPitch * 2] * 5 + pSrc[x + nSrcPitch * 3] + 16) / 32; \
 \
            pDst += nDstPitch; \
            pSrc += nSrcPitch * 2; \
        } \
    } \
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) { \
        for (int x = 0; x < nWidth; x++) \
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2; \
        pDst += nDstPitch; \
        pSrc += nSrcPitch * 2; \
    } \
}

RB2CubicVertical(uint8_t)
RB2CubicVertical(uint16_t)


// filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
#define RB2CubicHorizontalInplace(PixelType) \
static void RB2CubicHorizontalInplace_##PixelType(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int isse) { \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nSrcPitch /= sizeof(PixelType); \
 \
    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8; \
    for (int y = 0; y < nHeight; y++) { \
        int x = 0; \
        int pSrcw0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; /* store temporary */ \
        if (sizeof(PixelType) == 1 && isse) { \
            mvtools_RB2CubicHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX); \
            for (x = nWidthMMX; x < nWidth - 1; x++) \
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 5 + pSrc[x * 2] * 10 + pSrc[x * 2 + 1] * 10 + pSrc[x * 2 + 2] * 5 + pSrc[x * 2 + 3] + 16) / 32; \
        } else { \
            for (x = 1; x < nWidth - 1; x++) \
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 5 + pSrc[x * 2] * 10 + pSrc[x * 2 + 1] * 10 + pSrc[x * 2 + 2] * 5 + pSrc[x * 2 + 3] + 16) / 32; \
        } \
        pSrc[0] = pSrcw0; \
 \
        for (x = max(nWidth - 1, 1); x < nWidth; x++) \
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; \
 \
        pSrc += nSrcPitch; \
    } \
}

RB2CubicHorizontalInplace(uint8_t)
RB2CubicHorizontalInplace(uint16_t)


// separable filtered cubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
#define RB2Cubic(PixelType) \
static void RB2Cubic_##PixelType(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch, \
              int nSrcPitch, int nWidth, int nHeight, int isse) { \
    RB2CubicVertical_##PixelType(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); /* intermediate half height */ \
    RB2CubicHorizontalInplace_##PixelType(pDst, nDstPitch, nWidth, nHeight, isse);             /* inpace width reduction */ \
}

RB2Cubic(uint8_t)
RB2Cubic(uint16_t)


// so called Wiener interpolation. (sharp, similar to Lanczos ?)
// invarint simplified, 6 taps. Weights: (1, -5, 20, 20, -5, 1)/32 - added by Fizick
#define VerticalWiener(PixelType) \
static void VerticalWiener_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, \
                    intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nPitch /= sizeof(PixelType); \
 \
    int pixelMax = (1 << bitsPerSample) - 1; \
 \
    for (int j = 0; j < 2; j++) { \
        for (int i = 0; i < nWidth; i++) \
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1; \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
    for (int j = 2; j < nHeight - 4; j++) { \
        for (int i = 0; i < nWidth; i++) { \
            pDst[i] = min(pixelMax, max(0, \
                                        ((pSrc[i - nPitch * 2]) + (-(pSrc[i - nPitch]) + (pSrc[i] << 2) + (pSrc[i + nPitch] << 2) - (pSrc[i + nPitch * 2])) * 5 + (pSrc[i + nPitch * 3]) + 16) >> 5)); \
        } \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
    for (int j = nHeight - 4; j < nHeight - 1; j++) { \
        for (int i = 0; i < nWidth; i++) { \
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1; \
        } \
 \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
    /* last row */ \
    for (int i = 0; i < nWidth; i++) \
        pDst[i] = pSrc[i]; \
}

VerticalWiener(uint8_t)
VerticalWiener(uint16_t)


#define HorizontalWiener(PixelType) \
static void HorizontalWiener_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, \
                      intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nPitch /= sizeof(PixelType); \
 \
    int pixelMax = (1 << bitsPerSample) - 1; \
 \
    for (int j = 0; j < nHeight; j++) { \
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1; \
        pDst[1] = (pSrc[1] + pSrc[2] + 1) >> 1; \
        for (int i = 2; i < nWidth - 4; i++) { \
            pDst[i] = min(pixelMax, max(0, ((pSrc[i - 2]) + (-(pSrc[i - 1]) + (pSrc[i] << 2) + (pSrc[i + 1] << 2) - (pSrc[i + 2])) * 5 + (pSrc[i + 3]) + 16) >> 5)); \
        } \
        for (int i = nWidth - 4; i < nWidth - 1; i++) \
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1; \
 \
        pDst[nWidth - 1] = pSrc[nWidth - 1]; \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
}

HorizontalWiener(uint8_t)
HorizontalWiener(uint16_t)


// bicubic (Catmull-Rom 4 taps interpolation)
#define VerticalBicubic(PixelType) \
static void VerticalBicubic_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, \
                     intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nPitch /= sizeof(PixelType); \
 \
    int pixelMax = (1 << bitsPerSample) - 1; \
 \
    for (int j = 0; j < 1; j++) { \
        for (int i = 0; i < nWidth; i++) \
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1; \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
    for (int j = 1; j < nHeight - 3; j++) { \
        for (int i = 0; i < nWidth; i++) { \
            pDst[i] = min(pixelMax, max(0, \
                                        (-pSrc[i - nPitch] - pSrc[i + nPitch * 2] + (pSrc[i] + pSrc[i + nPitch]) * 9 + 8) >> 4)); \
        } \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
    for (int j = nHeight - 3; j < nHeight - 1; j++) { \
        for (int i = 0; i < nWidth; i++) { \
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1; \
        } \
 \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
    /* last row */ \
    for (int i = 0; i < nWidth; i++) \
        pDst[i] = pSrc[i]; \
}

VerticalBicubic(uint8_t)
VerticalBicubic(uint16_t)


#define HorizontalBicubic(PixelType) \
static void HorizontalBicubic_##PixelType(uint8_t *pDst8, const uint8_t *pSrc8, \
                       intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc = (PixelType *)pSrc8; \
 \
    nPitch /= sizeof(PixelType); \
 \
    int pixelMax = (1 << bitsPerSample) - 1; \
 \
    for (int j = 0; j < nHeight; j++) { \
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1; \
        for (int i = 1; i < nWidth - 3; i++) { \
            pDst[i] = min(pixelMax, max(0, \
                                        (-(pSrc[i - 1] + pSrc[i + 2]) + (pSrc[i] + pSrc[i + 1]) * 9 + 8) >> 4)); \
        } \
        for (int i = nWidth - 3; i < nWidth - 1; i++) \
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1; \
 \
        pDst[nWidth - 1] = pSrc[nWidth - 1]; \
        pDst += nPitch; \
        pSrc += nPitch; \
    } \
}

HorizontalBicubic(uint8_t)
HorizontalBicubic(uint16_t)


// assume all pitches equal
#define Average2(PixelType) \
static void Average2_##PixelType(uint8_t *pDst8, const uint8_t *pSrc18, const uint8_t *pSrc28, \
              intptr_t nPitch, intptr_t nWidth, intptr_t nHeight) { \
    PixelType *pDst = (PixelType *)pDst8; \
    PixelType *pSrc1 = (PixelType *)pSrc18; \
    PixelType *pSrc2 = (PixelType *)pSrc28; \
 \
    nPitch /= sizeof(PixelType); \
 \
    for (int j = 0; j < nHeight; j++) { \
        for (int i = 0; i < nWidth; i++) \
            pDst[i] = (pSrc1[i] + pSrc2[i] + 1) >> 1; \
 \
        pDst += nPitch; \
        pSrc1 += nPitch; \
        pSrc2 += nPitch; \
    } \
}

Average2(uint8_t)
Average2(uint16_t)


#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif
