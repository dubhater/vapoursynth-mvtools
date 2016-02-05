// Functions that computes distances between blocks

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

#ifndef SADFUNCTIONS_H
#define SADFUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef unsigned int (*SADFunction)(const uint8_t *pSrc, intptr_t nSrcPitch,
                                    const uint8_t *pRef, intptr_t nRefPitch);


#define MK_CFUNC(functionname) unsigned int functionname(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch)

// From SADFunctions.cpp
MK_CFUNC(mvtools_sad_2x2_u8_c);
MK_CFUNC(mvtools_sad_2x4_u8_c);
MK_CFUNC(mvtools_sad_4x2_u8_c);
MK_CFUNC(mvtools_sad_4x4_u8_c);
MK_CFUNC(mvtools_sad_4x8_u8_c);
MK_CFUNC(mvtools_sad_8x1_u8_c);
MK_CFUNC(mvtools_sad_8x2_u8_c);
MK_CFUNC(mvtools_sad_8x4_u8_c);
MK_CFUNC(mvtools_sad_8x8_u8_c);
MK_CFUNC(mvtools_sad_8x16_u8_c);
MK_CFUNC(mvtools_sad_16x1_u8_c);
MK_CFUNC(mvtools_sad_16x2_u8_c);
MK_CFUNC(mvtools_sad_16x4_u8_c);
MK_CFUNC(mvtools_sad_16x8_u8_c);
MK_CFUNC(mvtools_sad_16x16_u8_c);
MK_CFUNC(mvtools_sad_16x32_u8_c);
MK_CFUNC(mvtools_sad_32x8_u8_c);
MK_CFUNC(mvtools_sad_32x16_u8_c);
MK_CFUNC(mvtools_sad_32x32_u8_c);

MK_CFUNC(mvtools_sad_2x2_u16_c);
MK_CFUNC(mvtools_sad_2x4_u16_c);
MK_CFUNC(mvtools_sad_4x2_u16_c);
MK_CFUNC(mvtools_sad_4x4_u16_c);
MK_CFUNC(mvtools_sad_4x8_u16_c);
MK_CFUNC(mvtools_sad_8x1_u16_c);
MK_CFUNC(mvtools_sad_8x2_u16_c);
MK_CFUNC(mvtools_sad_8x4_u16_c);
MK_CFUNC(mvtools_sad_8x8_u16_c);
MK_CFUNC(mvtools_sad_8x16_u16_c);
MK_CFUNC(mvtools_sad_16x1_u16_c);
MK_CFUNC(mvtools_sad_16x2_u16_c);
MK_CFUNC(mvtools_sad_16x4_u16_c);
MK_CFUNC(mvtools_sad_16x8_u16_c);
MK_CFUNC(mvtools_sad_16x16_u16_c);
MK_CFUNC(mvtools_sad_16x32_u16_c);
MK_CFUNC(mvtools_sad_32x8_u16_c);
MK_CFUNC(mvtools_sad_32x16_u16_c);
MK_CFUNC(mvtools_sad_32x32_u16_c);

// From SAD.asm
MK_CFUNC(mvtools_sad_4x2_sse2);
MK_CFUNC(mvtools_sad_8x1_sse2);
MK_CFUNC(mvtools_sad_8x2_sse2);
MK_CFUNC(mvtools_sad_16x1_sse2);
MK_CFUNC(mvtools_sad_16x2_sse2);
MK_CFUNC(mvtools_sad_16x4_sse2);
MK_CFUNC(mvtools_sad_16x32_sse2);
MK_CFUNC(mvtools_sad_32x8_sse2);
MK_CFUNC(mvtools_sad_32x16_sse2);
MK_CFUNC(mvtools_sad_32x32_sse2);

MK_CFUNC(mvtools_sad_2x2_u16_sse2);
MK_CFUNC(mvtools_sad_2x4_u16_sse2);
MK_CFUNC(mvtools_sad_4x2_u16_sse2);
MK_CFUNC(mvtools_sad_4x4_u16_sse2);
MK_CFUNC(mvtools_sad_4x8_u16_sse2);
MK_CFUNC(mvtools_sad_8x1_u16_sse2);
MK_CFUNC(mvtools_sad_8x2_u16_sse2);
MK_CFUNC(mvtools_sad_8x4_u16_sse2);
MK_CFUNC(mvtools_sad_8x8_u16_sse2);
MK_CFUNC(mvtools_sad_8x16_u16_sse2);
MK_CFUNC(mvtools_sad_16x1_u16_sse2);
MK_CFUNC(mvtools_sad_16x2_u16_sse2);
MK_CFUNC(mvtools_sad_16x4_u16_sse2);
MK_CFUNC(mvtools_sad_16x8_u16_sse2);
MK_CFUNC(mvtools_sad_16x16_u16_sse2);
MK_CFUNC(mvtools_sad_16x32_u16_sse2);
MK_CFUNC(mvtools_sad_32x8_u16_sse2);
MK_CFUNC(mvtools_sad_32x16_u16_sse2);
MK_CFUNC(mvtools_sad_32x32_u16_sse2);

// From sad-a.asm - stolen from x264
MK_CFUNC(mvtools_pixel_sad_4x4_mmx2);
MK_CFUNC(mvtools_pixel_sad_4x8_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x4_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x8_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x16_mmx2);
MK_CFUNC(mvtools_pixel_sad_16x8_mmx2);
MK_CFUNC(mvtools_pixel_sad_16x16_mmx2);

MK_CFUNC(mvtools_pixel_sad_8x4_cache64_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x8_cache64_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x16_cache64_mmx2);

MK_CFUNC(mvtools_pixel_sad_8x16_sse2);
MK_CFUNC(mvtools_pixel_sad_16x8_sse2);  //non optimized cache access, for AMD?
MK_CFUNC(mvtools_pixel_sad_16x16_sse2); //non optimized cache access, for AMD?

MK_CFUNC(mvtools_pixel_sad_16x8_sse3);  //LDDQU Pentium4E (Core1?), not for Core2!
MK_CFUNC(mvtools_pixel_sad_16x16_sse3); //LDDQU Pentium4E (Core1?), not for Core2!

MK_CFUNC(mvtools_pixel_sad_16x8_cache64_ssse3);  //core2 optimized
MK_CFUNC(mvtools_pixel_sad_16x16_cache64_ssse3); //core2 optimized

// From pixel-a.asm - stolen from x264
/* alternative to SAD - SSD: squared sum of differences, VERY sensitive to noise */
MK_CFUNC(mvtools_pixel_ssd_4x4_mmx);
MK_CFUNC(mvtools_pixel_ssd_4x8_mmx);
MK_CFUNC(mvtools_pixel_ssd_8x4_mmx);
MK_CFUNC(mvtools_pixel_ssd_8x8_mmx);
MK_CFUNC(mvtools_pixel_ssd_8x16_mmx);
MK_CFUNC(mvtools_pixel_ssd_16x8_mmx);
MK_CFUNC(mvtools_pixel_ssd_16x16_mmx);

/* SATD: Sum of Absolute Transformed Differences, more sensitive to noise, frequency domain based - replacement to dct/SAD */

// From SADFunctions.cpp
MK_CFUNC(mvtools_satd_4x4_u8_c);
MK_CFUNC(mvtools_satd_8x4_u8_c);
MK_CFUNC(mvtools_satd_8x8_u8_c);
MK_CFUNC(mvtools_satd_16x8_u8_c);
MK_CFUNC(mvtools_satd_16x16_u8_c);

MK_CFUNC(mvtools_satd_4x4_u16_c);
MK_CFUNC(mvtools_satd_8x4_u16_c);
MK_CFUNC(mvtools_satd_8x8_u16_c);
MK_CFUNC(mvtools_satd_16x8_u16_c);
MK_CFUNC(mvtools_satd_16x16_u16_c);

// From sad-a.asm - stolen from x264
MK_CFUNC(mvtools_pixel_satd_4x4_mmx2);

MK_CFUNC(mvtools_pixel_satd_8x4_sse2);
MK_CFUNC(mvtools_pixel_satd_8x8_sse2);
MK_CFUNC(mvtools_pixel_satd_16x8_sse2);
MK_CFUNC(mvtools_pixel_satd_16x16_sse2);

MK_CFUNC(mvtools_pixel_satd_4x4_ssse3);
MK_CFUNC(mvtools_pixel_satd_8x4_ssse3);
MK_CFUNC(mvtools_pixel_satd_8x8_ssse3);
MK_CFUNC(mvtools_pixel_satd_16x8_ssse3);
MK_CFUNC(mvtools_pixel_satd_16x16_ssse3);

MK_CFUNC(mvtools_pixel_satd_4x4_sse4);
MK_CFUNC(mvtools_pixel_satd_8x4_sse4);
MK_CFUNC(mvtools_pixel_satd_8x8_sse4);
MK_CFUNC(mvtools_pixel_satd_16x8_sse4);
MK_CFUNC(mvtools_pixel_satd_16x16_sse4);

MK_CFUNC(mvtools_pixel_satd_4x4_avx);
MK_CFUNC(mvtools_pixel_satd_8x4_avx);
MK_CFUNC(mvtools_pixel_satd_8x8_avx);
MK_CFUNC(mvtools_pixel_satd_16x8_avx);
MK_CFUNC(mvtools_pixel_satd_16x16_avx);

MK_CFUNC(mvtools_pixel_satd_4x4_xop);
MK_CFUNC(mvtools_pixel_satd_8x4_xop);
MK_CFUNC(mvtools_pixel_satd_8x8_xop);
MK_CFUNC(mvtools_pixel_satd_16x8_xop);
MK_CFUNC(mvtools_pixel_satd_16x16_xop);

MK_CFUNC(mvtools_pixel_satd_8x8_avx2);
MK_CFUNC(mvtools_pixel_satd_16x8_avx2);
MK_CFUNC(mvtools_pixel_satd_16x16_avx2);

#undef MK_CFUNC

#ifdef __cplusplus
} // extern "C"
#endif

#endif
