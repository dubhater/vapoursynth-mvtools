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

GroupOfPlanes::GroupOfPlanes(int _nBlkSizeX, int _nBlkSizeY, int _nLevelCount, int _nPel, int _nMotionFlags, int _nCPUFlags, int _nOverlapX, int _nOverlapY, int _nBlkX, int _nBlkY, int _xRatioUV, int _yRatioUV, int _divideExtra, int _bitsPerSample)
{
    nBlkSizeX = _nBlkSizeX;
    nBlkSizeY = _nBlkSizeY;
    nLevelCount = _nLevelCount;
    nPel = _nPel;
    nMotionFlags = _nMotionFlags;
    nCPUFlags = _nCPUFlags;
    nOverlapX = _nOverlapX;
    nOverlapY = _nOverlapY;
    xRatioUV = _xRatioUV;
    yRatioUV = _yRatioUV;
    divideExtra = _divideExtra;
    bitsPerSample = _bitsPerSample;

    planes = new PlaneOfBlocks*[nLevelCount];

    int nBlkX = _nBlkX;
    int nBlkY = _nBlkY;

    int nPelCurrent = nPel;
    int nMotionFlagsCurrent = nMotionFlags;

    int nWidth_B = (nBlkSizeX - nOverlapX)*nBlkX + nOverlapX;
    int nHeight_B = (nBlkSizeY - nOverlapY)*nBlkY + nOverlapY;

    for ( int i = 0; i < nLevelCount; i++ )
    {
        if (i == nLevelCount-1)
            nMotionFlagsCurrent |= MOTION_SMALLEST_PLANE;
        nBlkX = ((nWidth_B>>i) - nOverlapX)/(nBlkSizeX-nOverlapX);
        nBlkY = ((nHeight_B>>i) - nOverlapY)/(nBlkSizeY-nOverlapY);
        planes[i] = new PlaneOfBlocks(nBlkX, nBlkY, nBlkSizeX, nBlkSizeY, nPelCurrent, i, nMotionFlagsCurrent, nCPUFlags, nOverlapX, nOverlapY, xRatioUV, yRatioUV, bitsPerSample);
        nPelCurrent = 1;
    }
}

GroupOfPlanes::~GroupOfPlanes()
{
    for ( int i = 0; i < nLevelCount; i++ )
        delete planes[i];
    delete[] planes;
}

void GroupOfPlanes::SearchMVs(MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF,
        SearchType searchType, int nSearchParam, int nPelSearch, int nLambda,
        int lsad, int pnew, int plevel, bool global,
        int *out, short *outfilebuf, int fieldShift, DCTClass * _DCT,
        int pzero, int pglobal, int badSAD, int badrange, bool meander, int *vecPrev, bool tryMany,
        SearchType coarseSearchType)
{
    int i;

    // write group's size
    out[0] = GetArraySize();

    // write validity : 1 in that case
    out[1] = 1;

    out += 2;
    if (vecPrev) vecPrev += 2;

    int fieldShiftCur = (nLevelCount - 1 == 0) ? fieldShift : 0; // may be non zero for finest level only

    VECTOR globalMV; // create and init global motion vector as zero
    globalMV.x = zeroMV.x;
    globalMV.y = zeroMV.y;
    globalMV.sad = zeroMV.sad;

    if (!global)
        pglobal = pzero;

    int meanLumaChange = 0;

    // Search the motion vectors, for the low details interpolations first
    SearchType searchTypeSmallest = (nLevelCount == 1 || searchType == HSEARCH || searchType == VSEARCH) ? searchType : coarseSearchType; // full search for smallest coarse plane
    int nSearchParamSmallest = (nLevelCount == 1) ? nPelSearch : nSearchParam;
    bool tryManyLevel = tryMany && nLevelCount>1;
    planes[nLevelCount - 1]->SearchMVs(pSrcGOF->GetFrame(nLevelCount-1),
            pRefGOF->GetFrame(nLevelCount-1),
            searchTypeSmallest, nSearchParamSmallest, nLambda, lsad, pnew, plevel,
            out, &globalMV, outfilebuf, fieldShiftCur, _DCT, &meanLumaChange, divideExtra,
            pzero, pglobal, badSAD, badrange, meander, vecPrev, tryManyLevel);
    // Refining the search until we reach the highest detail interpolation.

    out += planes[nLevelCount - 1]->GetArraySize(divideExtra);
    if (vecPrev) vecPrev += planes[nLevelCount - 1]->GetArraySize(divideExtra);

    for ( i = nLevelCount - 2; i >= 0; i-- )
    {
        SearchType searchTypeLevel = (i==0 || searchType == HSEARCH || searchType == VSEARCH) ? searchType : coarseSearchType; // full search for coarse planes
        int nSearchParamLevel = (i==0) ? nPelSearch : nSearchParam; // special case for finest level
        if (global)
        {
            planes[i+1]->EstimateGlobalMVDoubled(&globalMV); // get updated global MV (doubled)
        }
        planes[i]->InterpolatePrediction(*(planes[i+1]));
        fieldShiftCur = (i == 0) ? fieldShift : 0; // may be non zero for finest level only
        tryManyLevel = tryMany && i>0; // not for finest level to not decrease speed
        planes[i]->SearchMVs(pSrcGOF->GetFrame(i), pRefGOF->GetFrame(i),
                searchTypeLevel, nSearchParamLevel, nLambda, lsad, pnew, plevel,
                out, &globalMV, outfilebuf, fieldShiftCur, _DCT, &meanLumaChange, divideExtra,
                pzero, pglobal, badSAD, badrange, meander, vecPrev, tryManyLevel);
        out += planes[i]->GetArraySize(divideExtra);
        if (vecPrev) vecPrev += planes[i]->GetArraySize(divideExtra);
    }
}

void GroupOfPlanes::RecalculateMVs(MVClipBalls &mvClip, MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF,
        SearchType searchType, int nSearchParam, int nLambda,
        int pnew,
        int *out, short *outfilebuf, int fieldShift, int thSAD, DCTClass * _DCT, int smooth, bool meander)
{
    // write group's size
    out[0] = GetArraySize();

    // write validity : 1 in that case
    out[1] = 1;

    out += 2;

    // Search the motion vectors, for the low details interpolations first
    // Refining the search until we reach the highest detail interpolation.
    planes[0]->RecalculateMVs(mvClip, pSrcGOF->GetFrame(0),
            pRefGOF->GetFrame(0),
            searchType, nSearchParam, nLambda, pnew,
            out, outfilebuf, fieldShift, thSAD, _DCT, divideExtra, smooth, meander);

    out += planes[0]->GetArraySize(divideExtra);

}

void GroupOfPlanes::WriteDefaultToArray(int *array)
{
    // write group's size
    array[0] = GetArraySize();

    // write validity : unvalid in that case
    array[1] = 0;

    array += 2;

    // write planes
    for (int i = nLevelCount - 1; i >= 0; i-- )
    {
        array += planes[i]->WriteDefaultToArray(array, divideExtra);
    }
}

int GroupOfPlanes::GetArraySize()
{
    int size = 2; // size, validity
    for (int i = nLevelCount - 1; i >= 0; i-- )
        size += planes[i]->GetArraySize(divideExtra);


    return size;
}
// FIND MEDIAN OF 3 ELEMENTS
//
inline int Median3 (int a, int b, int c)
{
    // b a c || c a b
    if (((b <= a) && (a <= c)) || ((c <= a) && (a <= b))) return a;

    // a b c || c b a
    else if (((a <= b) && (b <= c)) || ((c <= b) && (b <= a))) return b;

    // b c a || a c b
    else return c;

}

void GetMedian(int *vx, int *vy, int vx1, int vy1, int vx2, int vy2, int vx3, int vy3)
{ // existant median vector (not mixed)
    *vx = Median3(vx1, vx2, vx3);
    *vy = Median3(vy1, vy2, vy3);
    if ( (*vx == vx1 && *vy == vy1) || (*vx == vx2 && *vy == vy2) ||(*vx == vx3 && *vy == vy3))
        return;
    else
    {
        *vx = vx1;
        *vy = vy1;
    }
}

void GroupOfPlanes::ExtraDivide(int *out)
{
    out += 2; // skip full size and validity
    for (int i = nLevelCount - 1; i >= 1; i-- ) // skip all levels up to finest estimated
        out += planes[i]->GetArraySize(0);

    int * inp = out + 1; // finest estimated plane
    out += out[0] + 1; // position for divided sublocks data

    int nBlkY = planes[0]->GetnBlkY();
    int nBlkXN = N_PER_BLOCK*planes[0]->GetnBlkX();
    // 6 stored variables

    int by = 0; // top blocks
    for (int bx = 0; bx<nBlkXN; bx+=N_PER_BLOCK)
    {
        for (int i=0; i<2; i++) // vx, vy
        {
            out[bx*2+i] = inp[bx+i]; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]; // bottom right subblock
        }
        for (int i=2; i<N_PER_BLOCK; i++) // sad, var, luma, ref
        {
            out[bx*2+i] = inp[bx+i]>>2; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]>>2; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]>>2; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]>>2; // bottom right subblock
        }
    }
    out += nBlkXN*4;
    inp += nBlkXN;
    for (by = 1; by<nBlkY-1; by++) // middle blocks
    {
        int bx = 0;
        for (int i=0; i<2; i++)
        {
            out[bx*2+i] = inp[bx+i]; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]; // bottom right subblock
        }
        for (int i=2; i<N_PER_BLOCK; i++)
        {
            out[bx*2+i] = inp[bx+i]>>2; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]>>2; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]>>2; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]>>2; // bottom right subblock
        }
        for (bx = N_PER_BLOCK; bx<nBlkXN-N_PER_BLOCK; bx+=N_PER_BLOCK)
        {
            if (divideExtra==1)
            {
                out[bx*2] = inp[bx]; // top left subblock
                out[bx*2+N_PER_BLOCK] = inp[bx]; // top right subblock
                out[bx*2 + nBlkXN*2] = inp[bx]; // bottom left subblock
                out[bx*2+N_PER_BLOCK + nBlkXN*2] = inp[bx]; // bottom right subblock
                out[bx*2+1] = inp[bx+1]; // top left subblock
                out[bx*2+N_PER_BLOCK+1] = inp[bx+1]; // top right subblock
                out[bx*2 + nBlkXN*2+1] = inp[bx+1]; // bottom left subblock
                out[bx*2+N_PER_BLOCK + nBlkXN*2+1] = inp[bx+1]; // bottom right subblock
            }
            else // divideExtra=2
            {
                int vx;
                int vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx+1], inp[bx-N_PER_BLOCK], inp[bx+1-N_PER_BLOCK], inp[bx-nBlkXN], inp[bx+1-nBlkXN]);// top left subblock
                out[bx*2] = vx;
                out[bx*2+1] = vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx+1], inp[bx+N_PER_BLOCK], inp[bx+1+N_PER_BLOCK], inp[bx-nBlkXN], inp[bx+1-nBlkXN]);// top right subblock
                out[bx*2+N_PER_BLOCK] = vx;
                out[bx*2+N_PER_BLOCK+1] = vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx+1], inp[bx-N_PER_BLOCK], inp[bx+1-N_PER_BLOCK], inp[bx+nBlkXN], inp[bx+1+nBlkXN]);// bottom left subblock
                out[bx*2 + nBlkXN*2] = vx;
                out[bx*2 + nBlkXN*2+1] = vy;
                GetMedian(&vx, &vy, inp[bx], inp[bx+1], inp[bx+N_PER_BLOCK], inp[bx+1+N_PER_BLOCK], inp[bx+nBlkXN], inp[bx+1+nBlkXN]);// bottom right subblock
                out[bx*2+N_PER_BLOCK + nBlkXN*2] = vx;
                out[bx*2+N_PER_BLOCK + nBlkXN*2+1] = vy;
            }
            for (int i=2; i<N_PER_BLOCK; i++)
            {
                out[bx*2+i] = inp[bx+i]>>2; // top left subblock
                out[bx*2+N_PER_BLOCK+i] = inp[bx+i]>>2; // top right subblock
                out[bx*2 + nBlkXN*2+i] = inp[bx+i]>>2; // bottom left subblock
                out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]>>2; // bottom right subblock
            }
        }
        bx = nBlkXN - N_PER_BLOCK;
        for (int i=0; i<2; i++)
        {
            out[bx*2+i] = inp[bx+i]; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]; // bottom right subblock
        }
        for (int i=2; i<N_PER_BLOCK; i++)
        {
            out[bx*2+i] = inp[bx+i]>>2; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]>>2; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]>>2; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]>>2; // bottom right subblock
        }
        out += nBlkXN*4;
        inp += nBlkXN;
    }
    by = nBlkY - 1; // bottom blocks
    for (int bx = 0; bx<nBlkXN; bx+=N_PER_BLOCK)
    {
        for (int i=0; i<2; i++)
        {
            out[bx*2+i] = inp[bx+i]; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]; // bottom right subblock
        }
        for (int i=2; i<N_PER_BLOCK; i++)
        {
            out[bx*2+i] = inp[bx+i]>>2; // top left subblock
            out[bx*2+N_PER_BLOCK+i] = inp[bx+i]>>2; // top right subblock
            out[bx*2 + nBlkXN*2+i] = inp[bx+i]>>2; // bottom left subblock
            out[bx*2+N_PER_BLOCK + nBlkXN*2+i] = inp[bx+i]>>2; // bottom right subblock
        }
    }
}
