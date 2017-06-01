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

#ifndef GROUPOFPLANES_H
#define GROUPOFPLANES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "DCTFFTW.h"
#include "Fakery.h"
#include "MVFrame.h"
#include "PlaneOfBlocks.h"


typedef struct GroupOfPlanes {
    int nBlkSizeX;
    int nBlkSizeY;
    int nLevelCount;
    int nOverlapX;
    int nOverlapY;
    int xRatioUV;
    int yRatioUV;
    int divideExtra;

    PlaneOfBlocks **planes;
} GroupOfPlanes;


void gopInit(GroupOfPlanes *gop, int nBlkSizeX, int nBlkSizeY, int nLevelCount, int nPel, int nMotionFlags, int nCPUFlags, int nOverlapX, int nOverlapY, int nBlkX, int nBlkY, int xRatioUV, int yRatioUV, int divideExtra, int bitsPerSample);

void gopDeinit(GroupOfPlanes *gop);

void gopSearchMVs(GroupOfPlanes *gop, MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF, SearchType searchType, int nSearchParam, int nPelSearch, int nLambda, int lsad, int pnew, int plevel, int global, uint8_t *out, int fieldShift, DCTFFTW *DCT, int dctmode, int pzero, int pglobal, int64_t badSAD, int badrange, int meander, int tryMany, SearchType coarseSearchType);

void gopRecalculateMVs(GroupOfPlanes *gop, FakeGroupOfPlanes *fgop, MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF, SearchType searchType, int nSearchParam, int nLambda, int pnew, uint8_t *out, int fieldShift, int thSAD, DCTFFTW *DCT, int dctmode, int smooth, int meander);

void gopWriteDefaultToArray(GroupOfPlanes *gop, uint8_t *array);

MVArraySizeType gopGetArraySize(GroupOfPlanes *gop);

void gopExtraDivide(GroupOfPlanes *gop, uint8_t *out);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
