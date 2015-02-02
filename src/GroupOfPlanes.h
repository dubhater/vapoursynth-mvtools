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

#ifndef __GOPLANES__
#define __GOPLANES__

#include "PlaneOfBlocks.h"

class GroupOfPlanes {
    int nBlkSizeX;
    int nBlkSizeY;
    int nLevelCount;
    int nPel;
    int nMotionFlags;
    int nCPUFlags;
    int nOverlapX;
    int nOverlapY;
    int xRatioUV;
    int yRatioUV;
    int divideExtra;
    int bitsPerSample;

    PlaneOfBlocks **planes;

    public :
    GroupOfPlanes(int _nBlkSizeX, int _nBlkSizeY, int _nLevelCount, int _nPel,
            int _nMotionFlags, int _nCPUFlags, int _nOverlapX, int _nOverlapY, int _nBlkX, int _nBlkY, int _xRatioUV, int _yRatioUV, int _divideExtra, int _bitsPerSample);
    ~GroupOfPlanes();
    void SearchMVs(MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF,
            SearchType searchType, int nSearchParam, int _PelSearch, int _nLambda, int _lsad, int _pnew, int _plevel, bool _global, int *out, short * outfilebuf, int fieldShift, DCTClass * DCT, int _pzero, int _pglobal, int badSAD, int badrange, bool meander, int *vecPrev, bool tryMany, SearchType coarseSearchType);
    void WriteDefaultToArray(int *array);
    int GetArraySize();
    void ExtraDivide(int *out);
    void RecalculateMVs(MVClipBalls &mvClip, MVGroupOfFrames *pSrcGOF, MVGroupOfFrames *pRefGOF,
            SearchType _searchType, int _nSearchParam, int _nLambda, int _pnew, int *out, short * outfilebuf, int fieldShift, int thSAD, DCTClass * DCT, int smooth, bool meander);
};

#endif
