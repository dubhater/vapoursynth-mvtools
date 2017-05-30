// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - overlap, global MV, divide
// See legal notice in Copying.txt for more information
//
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

#include "GroupOfPlanes.h"


void gopInit(GroupOfPlanes *gop, int nBlkSizeX, int nBlkSizeY, int nLevelCount, int nPel, int nMotionFlags, int nCPUFlags, int nOverlapX, int nOverlapY, int nBlkX, int nBlkY, int xRatioUV, int yRatioUV, int divideExtra, int bitsPerSample) {
    gop->nBlkSizeX = nBlkSizeX;
    gop->nBlkSizeY = nBlkSizeY;
    gop->nLevelCount = nLevelCount;
    gop->nOverlapX = nOverlapX;
    gop->nOverlapY = nOverlapY;
    gop->xRatioUV = xRatioUV;
    gop->yRatioUV = yRatioUV;
    gop->divideExtra = divideExtra;

    gop->planes = (PlaneOfBlocks **)malloc(gop->nLevelCount * sizeof(PlaneOfBlocks *));

    int nBlkXCurrent = nBlkX;
    int nBlkYCurrent = nBlkY;

    int nPelCurrent = nPel;
    int nMotionFlagsCurrent = nMotionFlags;

    int nWidth_B = (gop->nBlkSizeX - gop->nOverlapX) * nBlkX + gop->nOverlapX;
    int nHeight_B = (gop->nBlkSizeY - gop->nOverlapY) * nBlkY + gop->nOverlapY;

    for (int i = 0; i < gop->nLevelCount; i++) {
        if (i == gop->nLevelCount - 1)
            nMotionFlagsCurrent |= MOTION_SMALLEST_PLANE;
        nBlkXCurrent = ((nWidth_B >> i) - gop->nOverlapX) / (gop->nBlkSizeX - gop->nOverlapX);
        nBlkYCurrent = ((nHeight_B >> i) - gop->nOverlapY) / (gop->nBlkSizeY - gop->nOverlapY);

        gop->planes[i] = (PlaneOfBlocks *)malloc(sizeof(PlaneOfBlocks));
        pobInit(gop->planes[i], nBlkXCurrent, nBlkYCurrent, gop->nBlkSizeX, gop->nBlkSizeY, nPelCurrent, i, nMotionFlagsCurrent, nCPUFlags, gop->nOverlapX, gop->nOverlapY, gop->xRatioUV, gop->yRatioUV, bitsPerSample);
        nPelCurrent = 1;
    }
}


void gopDeinit(GroupOfPlanes *gop) {
    for (int i = 0; i < gop->nLevelCount; i++) {
        pobDeinit(gop->planes[i]);
        free(gop->planes[i]);
    }

    free(gop->planes);
}


void gopSearchMVs(GroupOfPlanes *gop, MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF,
                  SearchType searchType, int nSearchParam, int nPelSearch, int nLambda,
                  int lsad, int pnew, int plevel, int global,
                  int *out, int fieldShift, DCTFFTW *DCT, int dctmode,
                  int pzero, int pglobal, int64_t badSAD, int badrange, int meander, int tryMany,
                  SearchType coarseSearchType) {
    int i;

    // write group's size
    out[0] = gopGetArraySize(gop);

    // write validity : 1 in that case
    out[1] = 1;

    out += 2;

    int fieldShiftCur = (gop->nLevelCount - 1 == 0) ? fieldShift : 0; // may be non zero for finest level only

    VECTOR globalMV = zeroMV; // create and init global motion vector as zero

    if (!global)
        pglobal = pzero;

    int meanLumaChange = 0;

    // Search the motion vectors, for the low details interpolations first
    SearchType searchTypeSmallest = (gop->nLevelCount == 1 || searchType == SearchHorizontal || searchType == SearchVertical) ? searchType : coarseSearchType; // full search for smallest coarse plane
    int nSearchParamSmallest = (gop->nLevelCount == 1) ? nPelSearch : nSearchParam;
    int tryManyLevel = tryMany && gop->nLevelCount > 1;
    pobSearchMVs(gop->planes[gop->nLevelCount - 1],
                 pSrcGOF->frames[gop->nLevelCount - 1],
                 pRefGOF->frames[gop->nLevelCount - 1],
                 searchTypeSmallest, nSearchParamSmallest, nLambda, lsad, pnew, plevel,
                 out, &globalMV, fieldShiftCur, DCT, dctmode, &meanLumaChange,
                 pzero, pglobal, badSAD, badrange, meander, tryManyLevel);
    // Refining the search until we reach the highest detail interpolation.

    out += pobGetArraySize(gop->planes[gop->nLevelCount - 1], gop->divideExtra);

    for (i = gop->nLevelCount - 2; i >= 0; i--) {
        SearchType searchTypeLevel = (i == 0 || searchType == SearchHorizontal || searchType == SearchVertical) ? searchType : coarseSearchType; // full search for coarse planes
        int nSearchParamLevel = (i == 0) ? nPelSearch : nSearchParam;                                                            // special case for finest level
        if (global) {
            pobEstimateGlobalMVDoubled(gop->planes[i + 1], &globalMV); // get updated global MV (doubled)
        }
        pobInterpolatePrediction(gop->planes[i], gop->planes[i + 1]);
        fieldShiftCur = (i == 0) ? fieldShift : 0; // may be non zero for finest level only
        tryManyLevel = tryMany && i > 0;           // not for finest level to not decrease speed
        pobSearchMVs(gop->planes[i], pSrcGOF->frames[i], pRefGOF->frames[i],
                     searchTypeLevel, nSearchParamLevel, nLambda, lsad, pnew, plevel,
                     out, &globalMV, fieldShiftCur, DCT, dctmode, &meanLumaChange,
                     pzero, pglobal, badSAD, badrange, meander, tryManyLevel);
        out += pobGetArraySize(gop->planes[i], gop->divideExtra);
    }
}


void gopRecalculateMVs(GroupOfPlanes *gop, FakeGroupOfPlanes *fgop, MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF,
                       SearchType searchType, int nSearchParam, int nLambda,
                       int pnew,
                       int *out, int fieldShift, int thSAD, DCTFFTW *DCT, int dctmode, int smooth, int meander) {
    // write group's size
    out[0] = gopGetArraySize(gop);

    // write validity : 1 in that case
    out[1] = 1;

    out += 2;

    // Search the motion vectors, for the low details interpolations first
    // Refining the search until we reach the highest detail interpolation.
    pobRecalculateMVs(gop->planes[0], fgop, pSrcGOF->frames[0], pRefGOF->frames[0],
                      searchType, nSearchParam, nLambda, pnew,
                      out, fieldShift, thSAD, DCT, dctmode, smooth, meander);
}


void gopWriteDefaultToArray(GroupOfPlanes *gop, int *array) {
    // write group's size
    array[0] = gopGetArraySize(gop);

    // write validity : unvalid in that case
    array[1] = 0;

    array += 2;

    // write planes
    for (int i = gop->nLevelCount - 1; i >= 0; i--) {
        array += pobWriteDefaultToArray(gop->planes[i], array, gop->divideExtra);
    }
}


int gopGetArraySize(GroupOfPlanes *gop) {
    int size = 2; // size, validity
    for (int i = gop->nLevelCount - 1; i >= 0; i--)
        size += pobGetArraySize(gop->planes[i], gop->divideExtra);


    return size;
}


// FIND MEDIAN OF 3 ELEMENTS
//
static inline int Median3(int a, int b, int c) {
    // b a c || c a b
    if (((b <= a) && (a <= c)) || ((c <= a) && (a <= b)))
        return a;

    // a b c || c b a
    else if (((a <= b) && (b <= c)) || ((c <= b) && (b <= a)))
        return b;

    // b c a || a c b
    else
        return c;
}


static void GetMedian(int *vx, int *vy, int vx1, int vy1, int vx2, int vy2, int vx3, int vy3) { // existant median vector (not mixed)
    *vx = Median3(vx1, vx2, vx3);
    *vy = Median3(vy1, vy2, vy3);
    if ((*vx == vx1 && *vy == vy1) || (*vx == vx2 && *vy == vy2) || (*vx == vx3 && *vy == vy3))
        return;
    else {
        *vx = vx1;
        *vy = vy1;
    }
}


void gopExtraDivide(GroupOfPlanes *gop, int *out) {
    out += 2;                                  // skip full size and validity
    for (int i = gop->nLevelCount - 1; i >= 1; i--) // skip all levels up to finest estimated
        out += pobGetArraySize(gop->planes[i], 0);

    int *inp = out + 1; // finest estimated plane
    out += out[0] + 1;  // position for divided sublocks data

    int nBlkY = gop->planes[0]->nBlkY;
    int nBlkXN = N_PER_BLOCK * gop->planes[0]->nBlkX;

    int by = 0; // top blocks
    for (int bx = 0; bx < nBlkXN; bx += N_PER_BLOCK) {
        for (int i = 0; i < 2; i++) // vx, vy
        {
            out[bx * 2 + i] = inp[bx + i];                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i];              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i];               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i]; // bottom right subblock
        }
        for (int i = 2; i < N_PER_BLOCK; i++) // sad
        {
            out[bx * 2 + i] = inp[bx + i] >> 2;                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i] >> 2;              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i] >> 2;               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i] >> 2; // bottom right subblock
        }
    }
    out += nBlkXN * 4;
    inp += nBlkXN;
    for (by = 1; by < nBlkY - 1; by++) // middle blocks
    {
        int bx = 0;
        for (int i = 0; i < 2; i++) {
            out[bx * 2 + i] = inp[bx + i];                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i];              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i];               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i]; // bottom right subblock
        }
        for (int i = 2; i < N_PER_BLOCK; i++) {
            out[bx * 2 + i] = inp[bx + i] >> 2;                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i] >> 2;              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i] >> 2;               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i] >> 2; // bottom right subblock
        }
        for (bx = N_PER_BLOCK; bx < nBlkXN - N_PER_BLOCK; bx += N_PER_BLOCK) {
            if (gop->divideExtra == 1) {
                out[bx * 2] = inp[bx];                                    // top left subblock
                out[bx * 2 + N_PER_BLOCK] = inp[bx];                      // top right subblock
                out[bx * 2 + nBlkXN * 2] = inp[bx];                       // bottom left subblock
                out[bx * 2 + N_PER_BLOCK + nBlkXN * 2] = inp[bx];         // bottom right subblock
                out[bx * 2 + 1] = inp[bx + 1];                            // top left subblock
                out[bx * 2 + N_PER_BLOCK + 1] = inp[bx + 1];              // top right subblock
                out[bx * 2 + nBlkXN * 2 + 1] = inp[bx + 1];               // bottom left subblock
                out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + 1] = inp[bx + 1]; // bottom right subblock
            } else {                                                      // divideExtra=2
                int vx;
                int vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx + 1], inp[bx - N_PER_BLOCK], inp[bx + 1 - N_PER_BLOCK], inp[bx - nBlkXN], inp[bx + 1 - nBlkXN]); // top left subblock
                out[bx * 2] = vx;
                out[bx * 2 + 1] = vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx + 1], inp[bx + N_PER_BLOCK], inp[bx + 1 + N_PER_BLOCK], inp[bx - nBlkXN], inp[bx + 1 - nBlkXN]); // top right subblock
                out[bx * 2 + N_PER_BLOCK] = vx;
                out[bx * 2 + N_PER_BLOCK + 1] = vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx + 1], inp[bx - N_PER_BLOCK], inp[bx + 1 - N_PER_BLOCK], inp[bx + nBlkXN], inp[bx + 1 + nBlkXN]); // bottom left subblock
                out[bx * 2 + nBlkXN * 2] = vx;
                out[bx * 2 + nBlkXN * 2 + 1] = vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx + 1], inp[bx + N_PER_BLOCK], inp[bx + 1 + N_PER_BLOCK], inp[bx + nBlkXN], inp[bx + 1 + nBlkXN]); // bottom right subblock
                out[bx * 2 + N_PER_BLOCK + nBlkXN * 2] = vx;
                out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + 1] = vy;
            }
            for (int i = 2; i < N_PER_BLOCK; i++) {
                out[bx * 2 + i] = inp[bx + i] >> 2;                            // top left subblock
                out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i] >> 2;              // top right subblock
                out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i] >> 2;               // bottom left subblock
                out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i] >> 2; // bottom right subblock
            }
        }
        bx = nBlkXN - N_PER_BLOCK;
        for (int i = 0; i < 2; i++) {
            out[bx * 2 + i] = inp[bx + i];                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i];              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i];               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i]; // bottom right subblock
        }
        for (int i = 2; i < N_PER_BLOCK; i++) {
            out[bx * 2 + i] = inp[bx + i] >> 2;                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i] >> 2;              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i] >> 2;               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i] >> 2; // bottom right subblock
        }
        out += nBlkXN * 4;
        inp += nBlkXN;
    }
    by = nBlkY - 1; // bottom blocks
    for (int bx = 0; bx < nBlkXN; bx += N_PER_BLOCK) {
        for (int i = 0; i < 2; i++) {
            out[bx * 2 + i] = inp[bx + i];                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i];              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i];               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i]; // bottom right subblock
        }
        for (int i = 2; i < N_PER_BLOCK; i++) {
            out[bx * 2 + i] = inp[bx + i] >> 2;                            // top left subblock
            out[bx * 2 + N_PER_BLOCK + i] = inp[bx + i] >> 2;              // top right subblock
            out[bx * 2 + nBlkXN * 2 + i] = inp[bx + i] >> 2;               // bottom left subblock
            out[bx * 2 + N_PER_BLOCK + nBlkXN * 2 + i] = inp[bx + i] >> 2; // bottom right subblock
        }
    }
}
