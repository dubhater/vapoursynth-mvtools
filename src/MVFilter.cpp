// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - overlap, yRatioUV
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

#include "MVInterface.h"

MVFilter::MVFilter(VSNodeRef *vector, const char *filterName, const VSAPI *vsapi)
{
    if (vector == NULL)
        throw MVException(std::string(filterName).append(": vector clip must be specified")); //v1.8

    MVClipDicks mvClip(vector, 0, 0, vsapi);

    nWidth = mvClip.GetWidth();
    nHeight = mvClip.GetHeight();
    nHPadding = mvClip.GetHPadding();
    nVPadding = mvClip.GetVPadding();
    nBlkCount = mvClip.GetBlkCount();
    nBlkSizeX = mvClip.GetBlkSizeX();
    nBlkSizeY = mvClip.GetBlkSizeY();
    //   nIdx = mvClip.GetFramesIdx();
    nBlkX = mvClip.GetBlkX();
    nBlkY = mvClip.GetBlkY();
    nPel = mvClip.GetPel();
    //   mvCore = mvClip.GetMVCore();
    nOverlapX = mvClip.GetOverlapX();
    nOverlapY = mvClip.GetOverlapY();
    pixelType = mvClip.GetPixelType();
    yRatioUV = mvClip.GetYRatioUV();

    name = filterName;
}

void MVFilter::CheckSimilarity(const MVClipDicks *vector, const char *vectorName)
{
    if ( nWidth != vector->GetWidth() )
        throw MVException(std::string(name).append(": ").append(vectorName).append("'s width is incorrect."));

    if ( nHeight != vector->GetHeight() )
        throw MVException(std::string(name).append(": ").append(vectorName).append("'s height is incorrect."));

    if ( nBlkSizeX != vector->GetBlkSizeX() || nBlkSizeY != vector->GetBlkSizeY())
        throw MVException(std::string(name).append(": ").append(vectorName).append("'s block size is incorrect."));

    if ( nPel != vector->GetPel() )
        throw MVException(std::string(name).append(": ").append(vectorName).append("'s pel precisionis incorrect."));

    if ( nOverlapX != vector->GetOverlapX() ||  nOverlapY != vector->GetOverlapY())
        throw MVException(std::string(name).append(": ").append(vectorName).append("'s overlap size is incorrect."));

    if ( yRatioUV != vector->GetYRatioUV() )
        throw MVException(std::string(name).append(": ").append(vectorName).append("'s subsampling is incorrect."));
}
