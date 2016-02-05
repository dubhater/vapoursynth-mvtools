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

#include <math.h>
#include <stdlib.h>

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
        over->fWin1UVx[i] = cos(M_PI * (i - ox + 0.5f) / (ox * 2));
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
        over->fWin1UVx[i] = cos(M_PI * (i - nx + ox + 0.5f) / (ox * 2));
        over->fWin1UVx[i] = over->fWin1UVx[i] * over->fWin1UVx[i];  // right window (falled cosine)
        over->fWin1UVxfirst[i] = over->fWin1UVx[i];                 // very first window
        over->fWin1UVxlast[i] = 1;                                  // very last
    }

    over->fWin1UVy = (float *)malloc(ny * sizeof(float));
    over->fWin1UVyfirst = (float *)malloc(ny * sizeof(float));
    over->fWin1UVylast = (float *)malloc(ny * sizeof(float));
    for (int i = 0; i < oy; i++) {
        over->fWin1UVy[i] = cos(M_PI * (i - oy + 0.5f) / (oy * 2));
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
        over->fWin1UVy[i] = cos(M_PI * (i - ny + oy + 0.5f) / (oy * 2));
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


#define Overlaps_C(blockWidth, blockHeight, PixelType2, PixelType) \
void mvtools_overlaps_##blockWidth##x##blockHeight##_##PixelType2##_##PixelType##_c(uint8_t *pDst8, intptr_t nDstPitch, const uint8_t *pSrc8, intptr_t nSrcPitch, int16_t *pWin, intptr_t nWinPitch) { \
    /* pWin from 0 to 2048 */ \
    for (int j = 0; j < blockHeight; j++) { \
        for (int i = 0; i < blockWidth; i++) { \
            PixelType2 *pDst = (PixelType2 *)pDst8; \
            const PixelType *pSrc = (const PixelType *)pSrc8; \
 \
            pDst[i] += ((pSrc[i] * pWin[i]) >> 6); \
        } \
        pDst8 += nDstPitch; \
        pSrc8 += nSrcPitch; \
        pWin += nWinPitch; \
    } \
}

Overlaps_C(2, 2, uint16_t, uint8_t)
Overlaps_C(2, 4, uint16_t, uint8_t)
Overlaps_C(4, 2, uint16_t, uint8_t)
Overlaps_C(4, 4, uint16_t, uint8_t)
Overlaps_C(4, 8, uint16_t, uint8_t)
Overlaps_C(8, 1, uint16_t, uint8_t)
Overlaps_C(8, 2, uint16_t, uint8_t)
Overlaps_C(8, 4, uint16_t, uint8_t)
Overlaps_C(8, 8, uint16_t, uint8_t)
Overlaps_C(8, 16, uint16_t, uint8_t)
Overlaps_C(16, 1, uint16_t, uint8_t)
Overlaps_C(16, 2, uint16_t, uint8_t)
Overlaps_C(16, 4, uint16_t, uint8_t)
Overlaps_C(16, 8, uint16_t, uint8_t)
Overlaps_C(16, 16, uint16_t, uint8_t)
Overlaps_C(16, 32, uint16_t, uint8_t)
Overlaps_C(32, 8, uint16_t, uint8_t)
Overlaps_C(32, 16, uint16_t, uint8_t)
Overlaps_C(32, 32, uint16_t, uint8_t)

Overlaps_C(2, 2, uint32_t, uint16_t)
Overlaps_C(2, 4, uint32_t, uint16_t)
Overlaps_C(4, 2, uint32_t, uint16_t)
Overlaps_C(4, 4, uint32_t, uint16_t)
Overlaps_C(4, 8, uint32_t, uint16_t)
Overlaps_C(8, 1, uint32_t, uint16_t)
Overlaps_C(8, 2, uint32_t, uint16_t)
Overlaps_C(8, 4, uint32_t, uint16_t)
Overlaps_C(8, 8, uint32_t, uint16_t)
Overlaps_C(8, 16, uint32_t, uint16_t)
Overlaps_C(16, 1, uint32_t, uint16_t)
Overlaps_C(16, 2, uint32_t, uint16_t)
Overlaps_C(16, 4, uint32_t, uint16_t)
Overlaps_C(16, 8, uint32_t, uint16_t)
Overlaps_C(16, 16, uint32_t, uint16_t)
Overlaps_C(16, 32, uint32_t, uint16_t)
Overlaps_C(32, 8, uint32_t, uint16_t)
Overlaps_C(32, 16, uint32_t, uint16_t)
Overlaps_C(32, 32, uint32_t, uint16_t)


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
