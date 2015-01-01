// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - YUY2

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


void PadCorner(unsigned char *p, unsigned char v, int hPad, int vPad, int refPitch)
{
    for ( int i = 0; i < vPad; i++ )
    {
        memset(p, v, hPad); // faster than loop
        //        for ( int j = 0; j < hPad; j++ )
        //        {
        //            p[j] = v;
        //        }
        p += refPitch;
    }
}


void PadReferenceFrame(unsigned char *refFrame, int refPitch, int hPad, int vPad, int width, int height)
{
    unsigned char value;
    unsigned char *pfoff = refFrame + vPad * refPitch + hPad;
    unsigned char *p;

    // Up-Left
    PadCorner(refFrame, pfoff[0], hPad, vPad, refPitch);
    // Up-Right
    PadCorner(refFrame + hPad + width, pfoff[width - 1], hPad, vPad, refPitch);
    // Down-Left
    PadCorner(refFrame + (vPad + height) * refPitch,
            pfoff[(height - 1) * refPitch], hPad, vPad, refPitch);
    // Down-Right
    PadCorner(refFrame + hPad + width + (vPad + height) * refPitch,
            pfoff[(height - 1) * refPitch + width - 1], hPad, vPad, refPitch);

    // Up
    for ( int i = 0; i < width; i++ )
    {
        value = pfoff[i];
        p = refFrame + hPad + i;
        for ( int j = 0; j < vPad; j++ )
        {
            p[0] = value;
            p += refPitch;
        }
    }

    // Left
    for ( int i = 0; i < height; i++ )
    {
        value = pfoff[i*refPitch];
        p = refFrame + (vPad + i) * refPitch;
        for ( int j = 0; j < hPad; j++ )
            p[j] = value;
    }

    // Right
    for ( int i = 0; i < height; i++ )
    {
        value = pfoff[i * refPitch + width - 1];
        p = refFrame + (vPad + i) * refPitch + width + hPad;
        for ( int j = 0; j < hPad; j++ )
            p[j] = value;
    }

    // Down
    for ( int i = 0; i < width; i++ )
    {
        value = pfoff[i + (height - 1) * refPitch];
        p = refFrame + hPad + i + (height + vPad) * refPitch;
        for ( int j = 0; j < vPad; j++ )
        {
            p[0] = value;
            p += refPitch;
        }
    }
}
