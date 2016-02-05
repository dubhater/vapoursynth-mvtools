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

/*! \file Interpolation.h
 *  \brief Interpolating functions.
 *
 *    This set of functions is used to create a picture 4 times bigger, by
 *    interpolating the value of the half pixels. There are three different
 *  interpolations : horizontal computes the pixels ( x + 0.5, y ), vertical
 *    computes the pixels ( x, y + 0.5 ), and finally, diagonal computes the
 *  pixels ( x + 0.5, y + 0.5 ).
 *    For each type of interpolations, there are two functions, one in classical C,
 *  and one optimized for iSSE processors.
 */

#ifndef __INTERPOL__
#define __INTERPOL__

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

extern "C" void mvtools_Average2_sse2(uint8_t *pDst, const uint8_t *pSrc1, const uint8_t *pSrc2, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight);

extern "C" void mvtools_VerticalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                              intptr_t nWidth, intptr_t nHeight);
extern "C" void mvtools_HorizontalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                                intptr_t nWidth, intptr_t nHeight);
extern "C" void mvtools_DiagonalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                              intptr_t nWidth, intptr_t nHeight);


extern "C" void mvtools_RB2CubicHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX);
extern "C" void mvtools_RB2CubicVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
extern "C" void mvtools_RB2QuadraticHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX);
extern "C" void mvtools_RB2QuadraticVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
extern "C" void mvtools_RB2BilinearFilteredVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
extern "C" void mvtools_RB2BilinearFilteredHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX);
extern "C" void mvtools_VerticalWiener_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                            intptr_t nWidth, intptr_t nHeight);
extern "C" void mvtools_HorizontalWiener_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                              intptr_t nWidth, intptr_t nHeight);


template <typename PixelType>
void VerticalBilinear(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                      int nSrcPitch, int nWidth, int nHeight) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    // last row
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
void HorizontalBilinear(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                        int nSrcPitch, int nWidth, int nHeight) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}


template <typename PixelType>
void DiagonalBilinear(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                      int nSrcPitch, int nWidth, int nHeight) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int i = 0; i < nWidth - 1; i++)
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}


template <typename PixelType>
void RB2F_C(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + pSrc[x * 2 + nSrcPitch + 1] + pSrc[x * 2 + nSrcPitch] + 2) / 4;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}


template <typename PixelType>
void RB2FilteredVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                         int nSrcPitch, int nWidth, int nHeight, bool isse) { //  Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
    // nHeight is dst height which is reduced by 2 source height
    //int nWidthMMX = (nWidth/4)*4;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int y = 0; y < 1; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

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
       */
    {
        for (int y = 1; y < nHeight; y++) {
            for (int x = 0; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch] + pSrc[x] * 2 + pSrc[x + nSrcPitch] + 2) / 4;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
}

template <typename PixelType>
void RB2FilteredHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, bool isse) { // Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
    // nWidth is dst height which is reduced by 2 source width
    //int nWidthMMX = 1 + ((nWidth-2)/4)*4;

    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        /* TODO: port the asm
           if (isse)
           {
           RB2FilteredHorizontalInplaceLine_SSE((uint8_t *)pSrc, nWidthMMX); // very first is skipped
           for ( x = nWidthMMX; x < nWidth; x++ )
           pSrc[x] = (pSrc[x*2-1] + pSrc[x*2]*2 + pSrc[x*2+1] + 2) /4;
           }
           else
           */
        {
            for (x = 1; x < nWidth; x++)
                pSrc[x] = (pSrc[x * 2 - 1] + pSrc[x * 2] * 2 + pSrc[x * 2 + 1] + 2) / 4;
        }
        pSrc[0] = pSrc0;

        pSrc += nSrcPitch;
    }
}


template <typename PixelType>
void RB2Filtered(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
                 int nSrcPitch, int nWidth, int nHeight, bool isse) { // separable Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick v.2.5.2
    // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2FilteredVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); // intermediate half height
    RB2FilteredHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, isse);             // inpace width reduction
}


template <typename PixelType>
void RB2BilinearFilteredVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                                 int nSrcPitch, int nWidth, int nHeight, bool isse) { //  BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
    // nHeight is dst height which is reduced by 2 source height

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = (nWidth / 8) * 8;

    for (int y = 0; y < 1 && y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

    if (sizeof(PixelType) == 1 && isse && nWidthMMX >= 8) {
        for (int y = 1; y < nHeight - 1; y++) {
            mvtools_RB2BilinearFilteredVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);

            for (int x = nWidthMMX; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch] + pSrc[x] * 3 + pSrc[x + nSrcPitch] * 3 + pSrc[x + nSrcPitch * 2] + 4) / 8;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    } else {
        for (int y = 1; y < nHeight - 1; y++) {
            for (int x = 0; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch] + pSrc[x] * 3 + pSrc[x + nSrcPitch] * 3 + pSrc[x + nSrcPitch * 2] + 4) / 8;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}


template <typename PixelType>
void RB2BilinearFilteredHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, bool isse) { // BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
    // nWidth is dst height which is reduced by 2 source width

    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8;

    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        if (sizeof(PixelType) == 1 && isse) {
            mvtools_RB2BilinearFilteredHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX); // very first is skipped
            for (x = nWidthMMX; x < nWidth - 1; x++)
                pSrc[x] = (pSrc[x * 2 - 1] + pSrc[x * 2] * 3 + pSrc[x * 2 + 1] * 3 + pSrc[x * 2 + 2] + 4) / 8;
        } else {
            for (x = 1; x < nWidth - 1; x++)
                pSrc[x] = (pSrc[x * 2 - 1] + pSrc[x * 2] * 3 + pSrc[x * 2 + 1] * 3 + pSrc[x * 2 + 2] + 4) / 8;
        }
        pSrc[0] = pSrc0;

        for (x = max(nWidth - 1, 1); x < nWidth; x++)
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        pSrc += nSrcPitch;
    }
}


template <typename PixelType>
void RB2BilinearFiltered(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
                         int nSrcPitch, int nWidth, int nHeight, bool isse) { // separable BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick v.2.5.2
    // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2BilinearFilteredVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); // intermediate half height
    RB2BilinearFilteredHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, isse);             // inpace width reduction
}


template <typename PixelType>
void RB2QuadraticVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                          int nSrcPitch, int nWidth, int nHeight, bool isse) { // filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
    // nHeight is dst height which is reduced by 2 source height
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = (nWidth / 8) * 8;

    for (int y = 0; y < 1 && y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

    if (sizeof(PixelType) == 1 && isse && nWidthMMX >= 8) {
        for (int y = 1; y < nHeight - 1; y++) {
            mvtools_RB2QuadraticVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);

            for (int x = nWidthMMX; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 9 + pSrc[x] * 22 +
                           pSrc[x + nSrcPitch] * 22 + pSrc[x + nSrcPitch * 2] * 9 + pSrc[x + nSrcPitch * 3] + 32) / 64;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    } else {

        for (int y = 1; y < nHeight - 1; y++) {
            for (int x = 0; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 9 + pSrc[x] * 22 +
                           pSrc[x + nSrcPitch] * 22 + pSrc[x + nSrcPitch * 2] * 9 + pSrc[x + nSrcPitch * 3] + 32) / 64;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}


template <typename PixelType>
void RB2QuadraticHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int isse) { // filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
    // nWidth is dst height which is reduced by 2 source width
    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8;

    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; // store temporary

        if (sizeof(PixelType) == 1 && isse) {
            mvtools_RB2QuadraticHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX);
            for (x = nWidthMMX; x < nWidth - 1; x++)
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 9 + pSrc[x * 2] * 22 + pSrc[x * 2 + 1] * 22 + pSrc[x * 2 + 2] * 9 + pSrc[x * 2 + 3] + 32) / 64;
        } else {
            for (x = 1; x < nWidth - 1; x++)
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 9 + pSrc[x * 2] * 22 + pSrc[x * 2 + 1] * 22 + pSrc[x * 2 + 2] * 9 + pSrc[x * 2 + 3] + 32) / 64;
        }
        pSrc[0] = pSrc0;

        for (x = max(nWidth - 1, 1); x < nWidth; x++)
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        pSrc += nSrcPitch;
    }
}


template <typename PixelType>
void RB2Quadratic(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
                  int nSrcPitch, int nWidth, int nHeight, bool isse) { // separable filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick v.2.5.2
    // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2QuadraticVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); // intermediate half height
    RB2QuadraticHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, isse);             // inpace width reduction
}


template <typename PixelType>
void RB2CubicVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                      int nSrcPitch, int nWidth, int nHeight, bool isse) { // filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
    // nHeight is dst height which is reduced by 2 source height
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = (nWidth / 8) * 8;
    for (int y = 0; y < 1 && y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

    if (sizeof(PixelType) == 1 && isse && nWidthMMX >= 8) {
        for (int y = 1; y < nHeight - 1; y++) {
            mvtools_RB2CubicVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);

            for (int x = nWidthMMX; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 5 + pSrc[x] * 10 +
                           pSrc[x + nSrcPitch] * 10 + pSrc[x + nSrcPitch * 2] * 5 + pSrc[x + nSrcPitch * 3] + 16) / 32;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    } else {
        for (int y = 1; y < nHeight - 1; y++) {
            for (int x = 0; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch * 2] + pSrc[x - nSrcPitch] * 5 + pSrc[x] * 10 +
                           pSrc[x + nSrcPitch] * 10 + pSrc[x + nSrcPitch * 2] * 5 + pSrc[x + nSrcPitch * 3] + 16) / 32;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}


template <typename PixelType>
void RB2CubicHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, bool isse) { // filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
    // nWidth is dst height which is reduced by 2 source width
    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8;
    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrcw0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; // store temporary
        if (sizeof(PixelType) == 1 && isse) {
            mvtools_RB2CubicHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX);
            for (x = nWidthMMX; x < nWidth - 1; x++)
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 5 + pSrc[x * 2] * 10 + pSrc[x * 2 + 1] * 10 + pSrc[x * 2 + 2] * 5 + pSrc[x * 2 + 3] + 16) / 32;
        } else {
            for (x = 1; x < nWidth - 1; x++)
                pSrc[x] = (pSrc[x * 2 - 2] + pSrc[x * 2 - 1] * 5 + pSrc[x * 2] * 10 + pSrc[x * 2 + 1] * 10 + pSrc[x * 2 + 2] * 5 + pSrc[x * 2 + 3] + 16) / 32;
        }
        pSrc[0] = pSrcw0;

        for (x = max(nWidth - 1, 1); x < nWidth; x++)
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        pSrc += nSrcPitch;
    }
}


template <typename PixelType>
void RB2Cubic(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
              int nSrcPitch, int nWidth, int nHeight, bool isse) { // separable filtered cubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick v.2.5.2
    // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2CubicVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, isse); // intermediate half height
    RB2CubicHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, isse);             // inpace width reduction
}

// so called Wiener interpolation. (sharp, similar to Lanczos ?)
// invarint simplified, 6 taps. Weights: (1, -5, 20, 20, -5, 1)/32 - added by Fizick
template <typename PixelType>
void VerticalWiener(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                    int nSrcPitch, int nWidth, int nHeight, int bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = 2; j < nHeight - 4; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = min(pixelMax, max(0,
                                        ((pSrc[i - nSrcPitch * 2]) + (-(pSrc[i - nSrcPitch]) + (pSrc[i] << 2) + (pSrc[i + nSrcPitch] << 2) - (pSrc[i + nSrcPitch * 2])) * 5 + (pSrc[i + nSrcPitch * 3]) + 16) >> 5));
        }
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = nHeight - 4; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        }

        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    // last row
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
void HorizontalWiener(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                      int nSrcPitch, int nWidth, int nHeight, int bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < nHeight; j++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        pDst[1] = (pSrc[1] + pSrc[2] + 1) >> 1;
        for (int i = 2; i < nWidth - 4; i++) {
            pDst[i] = min(pixelMax, max(0, ((pSrc[i - 2]) + (-(pSrc[i - 1]) + (pSrc[i] << 2) + (pSrc[i + 1] << 2) - (pSrc[i + 2])) * 5 + (pSrc[i + 3]) + 16) >> 5));
        }
        for (int i = nWidth - 4; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}


template <typename PixelType>
void DiagonalWiener(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                    int nSrcPitch, int nWidth, int nHeight, int bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = 2; j < nHeight - 4; j++) {
        for (int i = 0; i < 2; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;
        for (int i = 2; i < nWidth - 4; i++) {
            pDst[i] = min(pixelMax, max(0,
                                        ((pSrc[i - 2 - nSrcPitch * 2]) + (-(pSrc[i - 1 - nSrcPitch]) + (pSrc[i] << 2) + (pSrc[i + 1 + nSrcPitch] << 2) - (pSrc[i + 2 + nSrcPitch * 2] << 2)) * 5 + (pSrc[i + 3 + nSrcPitch * 3]) + (pSrc[i + 3 - nSrcPitch * 2]) + (-(pSrc[i + 2 - nSrcPitch]) + (pSrc[i + 1] << 2) + (pSrc[i + nSrcPitch] << 2) - (pSrc[i - 1 + nSrcPitch * 2])) * 5 + (pSrc[i - 2 + nSrcPitch * 3]) + 32) >> 6));
        }
        for (int i = nWidth - 4; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = nHeight - 4; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int i = 0; i < nWidth - 1; i++)
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}


// bicubic (Catmull-Rom 4 taps interpolation)
template <typename PixelType>
void VerticalBicubic(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                     int nSrcPitch, int nWidth, int nHeight, int bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 1; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = 1; j < nHeight - 3; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = min(pixelMax, max(0,
                                        (-pSrc[i - nSrcPitch] - pSrc[i + nSrcPitch * 2] + (pSrc[i] + pSrc[i + nSrcPitch]) * 9 + 8) >> 4));
        }
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = nHeight - 3; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        }

        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    // last row
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}

template <typename PixelType>
void HorizontalBicubic(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                       int nSrcPitch, int nWidth, int nHeight, int bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < nHeight; j++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        for (int i = 1; i < nWidth - 3; i++) {
            pDst[i] = min(pixelMax, max(0,
                                        (-(pSrc[i - 1] + pSrc[i + 2]) + (pSrc[i] + pSrc[i + 1]) * 9 + 8) >> 4));
        }
        for (int i = nWidth - 3; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}

template <typename PixelType>
void DiagonalBicubic(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                     int nSrcPitch, int nWidth, int nHeight, int bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 1; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = 1; j < nHeight - 3; j++) {
        for (int i = 0; i < 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;
        for (int i = 1; i < nWidth - 3; i++) {
            pDst[i] = min(pixelMax, max(0,
                                        (-pSrc[i - 1 - nSrcPitch] - pSrc[i + 2 + nSrcPitch * 2] + (pSrc[i] + pSrc[i + 1 + nSrcPitch]) * 9 - pSrc[i - 1 + nSrcPitch * 2] - pSrc[i + 2 - nSrcPitch] + (pSrc[i + 1] + pSrc[i + nSrcPitch]) * 9 + 16) >> 5));
        }
        for (int i = nWidth - 3; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int j = nHeight - 3; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for (int i = 0; i < nWidth - 1; i++)
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}


template <typename PixelType>
void Average2(uint8_t *pDst8, const uint8_t *pSrc18, const uint8_t *pSrc28,
              int nPitch, int nWidth, int nHeight) { // assume all pitches equal
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc1 = (PixelType *)pSrc18;
    PixelType *pSrc2 = (PixelType *)pSrc28;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc1[i] + pSrc2[i] + 1) >> 1;

        pDst += nPitch;
        pSrc1 += nPitch;
        pSrc2 += nPitch;
    }
}

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#endif
