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

// I (Fizick) borrow code from Tom Barry's SimpleResize here

#ifndef __SIMPLERESIZE__
#define __SIMPLERESIZE__


#include <stdint.h>


class SimpleResize {
    int dst_width;
    int dst_height;
    int src_width;
    int src_height;

    int *vertical_offsets;
    int *vertical_weights;

    int *horizontal_offsets;
    int *horizontal_weights;

    void InitTables(int *offsets, int *weights, int out, int in);

    public:
    SimpleResize(int _dst_width, int _dst_height, int _src_width, int _src_height);
    ~SimpleResize();

    void Resize(uint8_t *dstp, int dst_stride, const uint8_t* srcp, int src_stride);
};


#endif
