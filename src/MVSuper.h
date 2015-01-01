// MVTools v2

// Copyright(c)2008 A.G.Balakhnin aka Fizick
// Prepare super clip (hierachical levels data) for MVAnalyse

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

#ifndef __MV_SUPER__
#define __MV_SUPER__


inline int PlaneHeightLuma(int src_height, int level, int yRatioUV, int vpad)
{
    int height = src_height;

    for (int i=1; i<=level; i++)
    {
        height = vpad >= yRatioUV ? ((height/yRatioUV + 1) / 2) * yRatioUV : ((height/yRatioUV) / 2) * yRatioUV;
    }
    return height;
}

inline int PlaneWidthLuma(int src_width, int level, int xRatioUV, int hpad)
{
    int width = src_width;

    for (int i=1; i<=level; i++)
    {
        width = hpad >= xRatioUV ? ((width/xRatioUV + 1) / 2) * xRatioUV : ((width/xRatioUV) / 2) * xRatioUV;
    }
    return width;
}

inline unsigned int PlaneSuperOffset(bool chroma, int src_height, int level, int pel, int vpad, int plane_pitch, int yRatioUV)
{
    // storing subplanes in superframes may be implemented by various ways
    int height = src_height; // luma or chroma

    unsigned int offset;

    if (level==0)
        offset = 0;
    else
    {
        offset = pel*pel*plane_pitch*(src_height + vpad*2);

        for (int i=1; i<level; i++)
        {
            height = chroma ? PlaneHeightLuma(src_height*yRatioUV, i, yRatioUV, vpad*yRatioUV)/yRatioUV : PlaneHeightLuma(src_height, i, yRatioUV, vpad);

            offset += plane_pitch*(height + vpad*2);
        }
    }
    return offset;
}


#endif
