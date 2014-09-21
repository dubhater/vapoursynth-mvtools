// Author: Manao
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

#include "CopyCode.h"

// I use here the copy code from avisynth. I duplicated it, because I have to use it
// in a class which doesn't have to know what "env" is. Anyway, such static functions
// should not have been put into that class in the first place ( imho )

#if 0
void BitBlt(unsigned char* dstp, int dst_pitch, const unsigned char* srcp, int src_pitch, int row_size, int height, bool isse) {
  if ( (!height)|| (!row_size)) return;
  if (isse) {
    if (height == 1 || (src_pitch == dst_pitch && dst_pitch == row_size)) {
      memcpy_amd(dstp, srcp, row_size*height);
    } else {
      asm_BitBlt_ISSE(dstp,dst_pitch,srcp,src_pitch,row_size,height);
    }
    return;
  }
  if (height == 1 || (dst_pitch == src_pitch && src_pitch == row_size)) {
    memcpy(dstp, srcp, row_size*height); // Fizick: fixed bug
  } else {
    for (int y=height; y>0; --y) {
      memcpy(dstp, srcp, row_size);
      dstp += dst_pitch;
      srcp += src_pitch;
    }
  }
}
#endif

void MemZoneSet(unsigned char *ptr, unsigned char value, int width,
				int height, int offsetX, int offsetY, int pitch)
{
	ptr += offsetX + offsetY*pitch;
	for (int y=offsetY; y<height+offsetY; y++)
	{
		memset(ptr, value, width);
		ptr += pitch;
	}
}
