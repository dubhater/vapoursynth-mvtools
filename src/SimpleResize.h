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

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


typedef struct SimpleResize {
    int dst_width;
    int dst_height;
    int src_width;
    int src_height;

    int *vertical_offsets;
    int *vertical_weights;

    int *horizontal_offsets;
    int *horizontal_weights;
} SimpleResize;


void simpleInit(SimpleResize *simple, int dst_width, int dst_height, int src_width, int src_height);
void simpleDeinit(SimpleResize *simple);
void simpleResize(SimpleResize *simple, uint8_t *dstp, int dst_stride, const uint8_t *srcp, int src_stride);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // __SIMPLERESIZE__
