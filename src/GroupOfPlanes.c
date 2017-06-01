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

#include <string.h>

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
                  uint8_t *out, int fieldShift, DCTFFTW *DCT, int dctmode,
                  int pzero, int pglobal, int64_t badSAD, int badrange, int meander, int tryMany,
                  SearchType coarseSearchType) {
    int i;

    // write group's size
    MVArraySizeType size = gopGetArraySize(gop);
    memcpy(out, &size, sizeof(size));

    // write validity : 1 in that case
    MVArraySizeType validity = 1;
    memcpy(out + sizeof(size), &validity, sizeof(validity));

    out += sizeof(size) + sizeof(validity);

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
                       uint8_t *out, int fieldShift, int64_t thSAD, DCTFFTW *DCT, int dctmode, int smooth, int meander) {
    // write group's size
    MVArraySizeType size = gopGetArraySize(gop);
    memcpy(out, &size, sizeof(size));

    // write validity : 1 in that case
    MVArraySizeType validity = 1;
    memcpy(out + sizeof(size), &validity, sizeof(validity));

    out += sizeof(size) + sizeof(validity);

    // Search the motion vectors, for the low details interpolations first
    // Refining the search until we reach the highest detail interpolation.
    pobRecalculateMVs(gop->planes[0], fgop, pSrcGOF->frames[0], pRefGOF->frames[0],
                      searchType, nSearchParam, nLambda, pnew,
                      out, fieldShift, thSAD, DCT, dctmode, smooth, meander);
}


void gopWriteDefaultToArray(GroupOfPlanes *gop, uint8_t *array) {
    // write group's size
    MVArraySizeType size = gopGetArraySize(gop);
    memcpy(array, &size, sizeof(size));

    // write validity : unvalid in that case
    MVArraySizeType validity = 0;
    memcpy(array + sizeof(size), &validity, sizeof(validity));

    array += sizeof(size) + sizeof(validity);

    // write planes
    for (int i = gop->nLevelCount - 1; i >= 0; i--)
        array += pobWriteDefaultToArray(gop->planes[i], array, gop->divideExtra);
}


MVArraySizeType gopGetArraySize(GroupOfPlanes *gop) {
    MVArraySizeType size = 2 * sizeof(MVArraySizeType); // size, validity
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


void gopExtraDivide(GroupOfPlanes *gop, uint8_t *out) {
    out += 2 * sizeof(MVArraySizeType);             // skip full size and validity
    for (int i = gop->nLevelCount - 1; i >= 1; i--) // skip all levels up to finest estimated
        out += pobGetArraySize(gop->planes[i], 0);

    MVArraySizeType size;
    memcpy(&size, out, sizeof(size));

    const VECTOR *blocks_in = (const VECTOR *)(out + sizeof(size)); // finest estimated plane
    VECTOR *blocks_out = (VECTOR *)(out + size + sizeof(MVArraySizeType)); // position for divided subblocks data

    int nBlkY = gop->planes[0]->nBlkY;
    int nBlkX = gop->planes[0]->nBlkX;

    // top blocks
    for (int bx = 0; bx < nBlkX; bx++) {
        VECTOR block = blocks_in[bx];
        block.sad >>= 2;

        blocks_out[bx * 2] = block;                 // top left subblock
        blocks_out[bx * 2 + 1] = block;             // top right subblock
        blocks_out[bx * 2 + nBlkX * 2] = block;     // bottom left subblock
        blocks_out[bx * 2 + nBlkX * 2 + 1] = block; // bottom right subblock
    }

    blocks_out += nBlkX * 4;
    blocks_in += nBlkX;

    // middle blocks
    for (int by = 1; by < nBlkY - 1; by++) {
        int bx = 0;

        VECTOR block = blocks_in[bx];
        block.sad >>= 2;

        blocks_out[bx * 2] = block;                 // top left subblock
        blocks_out[bx * 2 + 1] = block;             // top right subblock
        blocks_out[bx * 2 + nBlkX * 2] = block;     // bottom left subblock
        blocks_out[bx * 2 + nBlkX * 2 + 1] = block; // bottom right subblock

        for (bx = 1; bx < nBlkX - 1; bx++) {
            block = blocks_in[bx];
            block.sad >>= 2;

            blocks_out[bx * 2] = block;                 // top left subblock
            blocks_out[bx * 2 + 1] = block;             // top right subblock
            blocks_out[bx * 2 + nBlkX * 2] = block;     // bottom left subblock
            blocks_out[bx * 2 + nBlkX * 2 + 1] = block; // bottom right subblock

            if (gop->divideExtra > 1) {
                GetMedian(&blocks_out[bx * 2].x,    &blocks_out[bx * 2].y,
                          blocks_in[bx].x,          blocks_in[bx].y,
                          blocks_in[bx - 1].x,      blocks_in[bx - 1].y,
                          blocks_in[bx - nBlkX].x,  blocks_in[bx - nBlkX].y);

                GetMedian(&blocks_out[bx * 2 + 1].x,    &blocks_out[bx * 2 + 1].y,
                          blocks_in[bx].x,              blocks_in[bx].y,
                          blocks_in[bx + 1].x,          blocks_in[bx + 1].y,
                          blocks_in[bx - nBlkX].x,      blocks_in[bx - nBlkX].y);

                GetMedian(&blocks_out[bx * 2 + nBlkX * 2].x,    &blocks_out[bx * 2 + nBlkX * 2].y,
                          blocks_in[bx].x,                      blocks_in[bx].y,
                          blocks_in[bx - 1].x,                  blocks_in[bx - 1].y,
                          blocks_in[bx + nBlkX].x,              blocks_in[bx + nBlkX].y);

                GetMedian(&blocks_out[bx * 2 + nBlkX * 2 + 1].x,    &blocks_out[bx * 2 + nBlkX * 2 + 1].y,
                          blocks_in[bx].x,                          blocks_in[bx].y,
                          blocks_in[bx + 1].x,                      blocks_in[bx + 1].y,
                          blocks_in[bx + nBlkX].x,                  blocks_in[bx + nBlkX].y);
            }
        }

        bx = nBlkX - 1;

        block = blocks_in[bx];
        block.sad >>= 2;

        blocks_out[bx * 2] = block;                 // top left subblock
        blocks_out[bx * 2 + 1] = block;             // top right subblock
        blocks_out[bx * 2 + nBlkX * 2] = block;     // bottom left subblock
        blocks_out[bx * 2 + nBlkX * 2 + 1] = block; // bottom right subblock

        blocks_out += nBlkX * 4;
        blocks_in += nBlkX;
    }

    // bottom blocks
    for (int bx = 0; bx < nBlkX; bx++) {
        VECTOR block = blocks_in[bx];
        block.sad >>= 2;

        blocks_out[bx * 2] = block;                 // top left subblock
        blocks_out[bx * 2 + 1] = block;             // top right subblock
        blocks_out[bx * 2 + nBlkX * 2] = block;     // bottom left subblock
        blocks_out[bx * 2 + nBlkX * 2 + 1] = block; // bottom right subblock
    }
}
