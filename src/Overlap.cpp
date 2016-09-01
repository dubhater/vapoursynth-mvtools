// Overlap copy (really addition)
// Copyright(c)2006 A.G.Balakhnin aka Fizick

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

#include <cmath>
#include <cstdlib>
#include <unordered_map>

#include "Overlap.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) (((a) < (b)) ? (b) : (a))
#endif


void overInit(OverlapWindows *over, int nx, int ny, int ox, int oy) {
    over->nx = nx;
    over->ny = ny;
    over->ox = ox;
    over->oy = oy;
    over->size = nx * ny;

    //  windows
    over->fWin1UVx = (float *)malloc(nx * sizeof(float));
    over->fWin1UVxfirst = (float *)malloc(nx * sizeof(float));
    over->fWin1UVxlast = (float *)malloc(nx * sizeof(float));
    for (int i = 0; i < ox; i++) {
        over->fWin1UVx[i] = cosf(M_PI * (i - ox + 0.5f) / (ox * 2));
        over->fWin1UVx[i] = over->fWin1UVx[i] * over->fWin1UVx[i];  // left window (rised cosine)
        over->fWin1UVxfirst[i] = 1;                                 // very first window
        over->fWin1UVxlast[i] = over->fWin1UVx[i];                  // very last
    }
    for (int i = ox; i < nx - ox; i++) {
        over->fWin1UVx[i] = 1;
        over->fWin1UVxfirst[i] = 1; // very first window
        over->fWin1UVxlast[i] = 1;  // very last
    }
    for (int i = nx - ox; i < nx; i++) {
        over->fWin1UVx[i] = cosf(M_PI * (i - nx + ox + 0.5f) / (ox * 2));
        over->fWin1UVx[i] = over->fWin1UVx[i] * over->fWin1UVx[i];  // right window (falled cosine)
        over->fWin1UVxfirst[i] = over->fWin1UVx[i];                 // very first window
        over->fWin1UVxlast[i] = 1;                                  // very last
    }

    over->fWin1UVy = (float *)malloc(ny * sizeof(float));
    over->fWin1UVyfirst = (float *)malloc(ny * sizeof(float));
    over->fWin1UVylast = (float *)malloc(ny * sizeof(float));
    for (int i = 0; i < oy; i++) {
        over->fWin1UVy[i] = cosf(M_PI * (i - oy + 0.5f) / (oy * 2));
        over->fWin1UVy[i] = over->fWin1UVy[i] * over->fWin1UVy[i];  // left window (rised cosine)
        over->fWin1UVyfirst[i] = 1;                                 // very first window
        over->fWin1UVylast[i] = over->fWin1UVy[i];                  // very last
    }
    for (int i = oy; i < ny - oy; i++) {
        over->fWin1UVy[i] = 1;
        over->fWin1UVyfirst[i] = 1; // very first window
        over->fWin1UVylast[i] = 1;  // very last
    }
    for (int i = ny - oy; i < ny; i++) {
        over->fWin1UVy[i] = cosf(M_PI * (i - ny + oy + 0.5f) / (oy * 2));
        over->fWin1UVy[i] = over->fWin1UVy[i] * over->fWin1UVy[i];  // right window (falled cosine)
        over->fWin1UVyfirst[i] = over->fWin1UVy[i];                 // very first window
        over->fWin1UVylast[i] = 1;                                  // very last
    }


    over->Overlap9Windows = (int16_t *)malloc(over->size * 9 * sizeof(int16_t));

    int16_t *winOverUVTL = over->Overlap9Windows;
    int16_t *winOverUVTM = over->Overlap9Windows + over->size;
    int16_t *winOverUVTR = over->Overlap9Windows + over->size * 2;
    int16_t *winOverUVML = over->Overlap9Windows + over->size * 3;
    int16_t *winOverUVMM = over->Overlap9Windows + over->size * 4;
    int16_t *winOverUVMR = over->Overlap9Windows + over->size * 5;
    int16_t *winOverUVBL = over->Overlap9Windows + over->size * 6;
    int16_t *winOverUVBM = over->Overlap9Windows + over->size * 7;
    int16_t *winOverUVBR = over->Overlap9Windows + over->size * 8;

    for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
            winOverUVTL[i] = (int)(over->fWin1UVyfirst[j] * over->fWin1UVxfirst[i] * 2048 + 0.5f);
            winOverUVTM[i] = (int)(over->fWin1UVyfirst[j] * over->fWin1UVx[i] * 2048 + 0.5f);
            winOverUVTR[i] = (int)(over->fWin1UVyfirst[j] * over->fWin1UVxlast[i] * 2048 + 0.5f);
            winOverUVML[i] = (int)(over->fWin1UVy[j] * over->fWin1UVxfirst[i] * 2048 + 0.5f);
            winOverUVMM[i] = (int)(over->fWin1UVy[j] * over->fWin1UVx[i] * 2048 + 0.5f);
            winOverUVMR[i] = (int)(over->fWin1UVy[j] * over->fWin1UVxlast[i] * 2048 + 0.5f);
            winOverUVBL[i] = (int)(over->fWin1UVylast[j] * over->fWin1UVxfirst[i] * 2048 + 0.5f);
            winOverUVBM[i] = (int)(over->fWin1UVylast[j] * over->fWin1UVx[i] * 2048 + 0.5f);
            winOverUVBR[i] = (int)(over->fWin1UVylast[j] * over->fWin1UVxlast[i] * 2048 + 0.5f);
        }
        winOverUVTL += nx;
        winOverUVTM += nx;
        winOverUVTR += nx;
        winOverUVML += nx;
        winOverUVMM += nx;
        winOverUVMR += nx;
        winOverUVBL += nx;
        winOverUVBM += nx;
        winOverUVBR += nx;
    }
}


void overDeinit(OverlapWindows *over) {
    free(over->Overlap9Windows);
    free(over->fWin1UVx);
    free(over->fWin1UVxfirst);
    free(over->fWin1UVxlast);
    free(over->fWin1UVy);
    free(over->fWin1UVyfirst);
    free(over->fWin1UVylast);
}


int16_t *overGetWindow(const OverlapWindows *over, int i) {
    return over->Overlap9Windows + over->size * i;
}


template <unsigned blockWidth, unsigned blockHeight, typename PixelType2, typename PixelType>
void overlaps_c(uint8_t *pDst8, intptr_t nDstPitch, const uint8_t *pSrc8, intptr_t nSrcPitch, int16_t *pWin, intptr_t nWinPitch) {
    /* pWin from 0 to 2048 */
    for (unsigned j = 0; j < blockHeight; j++) {
        for (unsigned i = 0; i < blockWidth; i++) {
            PixelType2 *pDst = (PixelType2 *)pDst8;
            const PixelType *pSrc = (const PixelType *)pSrc8;

            pDst[i] += ((pSrc[i] * pWin[i]) >> 6);
        }
        pDst8 += nDstPitch;
        pSrc8 += nSrcPitch;
        pWin += nWinPitch;
    }
}


#if defined(MVTOOLS_X86)

#define DECLARE_OVERS(width, height) extern "C" void mvtools_overlaps_##width##x##height##_sse2(uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch, int16_t *pWin, intptr_t nWinPitch);

DECLARE_OVERS(2, 2)
DECLARE_OVERS(2, 4)
DECLARE_OVERS(4, 2)
DECLARE_OVERS(4, 4)
DECLARE_OVERS(4, 8)
DECLARE_OVERS(8, 1)
DECLARE_OVERS(8, 2)
DECLARE_OVERS(8, 4)
DECLARE_OVERS(8, 8)
DECLARE_OVERS(8, 16)
DECLARE_OVERS(16, 1)
DECLARE_OVERS(16, 2)
DECLARE_OVERS(16, 4)
DECLARE_OVERS(16, 8)
DECLARE_OVERS(16, 16)
DECLARE_OVERS(16, 32)
DECLARE_OVERS(32, 8)
DECLARE_OVERS(32, 16)
DECLARE_OVERS(32, 32)

#undef DECLARE_OVERS

#endif


enum InstructionSets {
    Scalar,
    SSE2,
};


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86)
#define OVERS_SSE2(width, height) \
    { KEY(width, height, 8, SSE2), mvtools_overlaps_##width##x##height##_sse2 },
#else
#define OVERS_SSE2(width, height)
#endif

#define OVERS(width, height) \
    { KEY(width, height, 8, Scalar), overlaps_c<width, height, uint16_t, uint8_t> }, \
    { KEY(width, height, 16, Scalar), overlaps_c<width, height, uint32_t, uint16_t> }, \
    OVERS_SSE2(width, height)

static const std::unordered_map<uint32_t, OverlapsFunction> overlaps_functions = {
    OVERS(2, 2)
    OVERS(2, 4)
    OVERS(4, 2)
    OVERS(4, 4)
    OVERS(4, 8)
    OVERS(8, 1)
    OVERS(8, 2)
    OVERS(8, 4)
    OVERS(8, 8)
    OVERS(8, 16)
    OVERS(16, 1)
    OVERS(16, 2)
    OVERS(16, 4)
    OVERS(16, 8)
    OVERS(16, 16)
    OVERS(16, 32)
    OVERS(32, 8)
    OVERS(32, 16)
    OVERS(32, 32)
};

OverlapsFunction selectOverlapsFunction(unsigned width, unsigned height, unsigned bits, int opt) {
    OverlapsFunction overs = overlaps_functions.at(KEY(width, height, bits, Scalar));

#if defined(MVTOOLS_X86)
    if (opt) {
        try {
            overs = overlaps_functions.at(KEY(width, height, bits, SSE2));
        } catch (std::out_of_range &) { }
    }
#endif

    return overs;
}

#undef OVERS
#undef OVERS_SSE2
#undef KEY


#define ToPixels(PixelType2, PixelType) \
void ToPixels_##PixelType2##_##PixelType(uint8_t *pDst8, int nDstPitch, const uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int bitsPerSample) { \
    int pixelMax = (1 << bitsPerSample) - 1; \
 \
    for (int h = 0; h < nHeight; h++) { \
        for (int i = 0; i < nWidth; i++) { \
            const PixelType2 *pSrc = (const PixelType2 *)pSrc8; \
            PixelType *pDst = (PixelType *)pDst8; \
 \
            int a = (pSrc[i] + 16) >> 5; \
            if (sizeof(PixelType) == 1) \
                pDst[i] = a | ((255 - a) >> (sizeof(int) * 8 - 1)); \
            else \
                pDst[i] = min(pixelMax, a); \
        } \
        pDst8 += nDstPitch; \
        pSrc8 += nSrcPitch; \
    } \
}

ToPixels(uint16_t, uint8_t)
ToPixels(uint32_t, uint16_t)
