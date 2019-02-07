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

#ifndef SIMPLERESIZE_H
#define SIMPLERESIZE_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


enum {
    simple_resize_weight_shift = 14,
    simple_resize_weight_max = 1 << simple_resize_weight_shift,
    simple_resize_weight_half = simple_resize_weight_max / 2,
};


typedef struct SimpleResize SimpleResize;


typedef void (*ResizeFunction8)(const SimpleResize *simple,
                                uint8_t *dstp, int dst_stride,
                                const uint8_t *srcp, int src_stride,
                                int horizontal_vectors);
typedef void (*ResizeFunction16)(const SimpleResize *simple,
                                 int16_t *dstp, int dst_stride,
                                 const int16_t *srcp, int src_stride,
                                 int horizontal_vectors);


typedef struct SimpleResize {
    int dst_width;
    int dst_height;
    int src_width;
    int src_height;

    // Used only to limit the vectors in the 16 bit resizer.
    // dst_width and dst_height are usually the padded dimensions.
    // The two below are the unpadded dimensions, i.e. the actual frame size.
    int limit_width;
    int limit_height;
    int pel;

    int *vertical_offsets;
    int *vertical_weights;

    int *horizontal_offsets;
    int *horizontal_weights;

    ResizeFunction8 simpleResize_uint8_t;
    ResizeFunction16 simpleResize_int16_t;
} SimpleResize;


void simpleInit(SimpleResize *simple, int dst_width, int dst_height, int src_width, int src_height, int limit_width, int limit_height, int pel, int opt);
void simpleDeinit(SimpleResize *simple);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // SIMPLERESIZE_H
