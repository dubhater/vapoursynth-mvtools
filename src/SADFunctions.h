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

#ifndef __SAD_FUNC__
#define __SAD_FUNC__

#include "MVInterface.h"

typedef unsigned int (*SADFunction)(const uint8_t *pSrc, intptr_t nSrcPitch,
        const uint8_t *pRef, intptr_t nRefPitch);

inline unsigned int SADABS(int x) {    return ( x < 0 ) ? -x : x; }

template<int nBlkWidth, int nBlkHeight, typename PixelType>
unsigned int Sad_C(const uint8_t *pSrc8, intptr_t nSrcPitch,const uint8_t *pRef8,
        intptr_t nRefPitch)
{
    unsigned int sum = 0;
    for ( int y = 0; y < nBlkHeight; y++ )
    {
        for ( int x = 0; x < nBlkWidth; x++ ) {
            const PixelType *pSrc = (const PixelType *)pSrc8;
            const PixelType *pRef = (const PixelType *)pRef8;
            sum += SADABS(pSrc[x] - pRef[x]);
        }
        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }
    return sum;
}


#define HADAMARD4(d0, d1, d2, d3, s0, s1, s2, s3) {\
    SumType2 t0 = s0 + s1;\
    SumType2 t1 = s0 - s1;\
    SumType2 t2 = s2 + s3;\
    SumType2 t3 = s2 - s3;\
    d0 = t0 + t2;\
    d2 = t0 - t2;\
    d1 = t1 + t3;\
    d3 = t1 - t3;\
}


// in: a pseudo-simd number of the form x+(y<<16)
// return: abs(x)+(abs(y)<<16)
template <typename SumType, typename SumType2>
static inline SumType2 abs2( SumType2 a )
{
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 s = ((a>>(bitsPerSum-1))&(((SumType2)1<<bitsPerSum)+1))*((SumType)-1);
    return (a+s)^s;
}


template <typename PixelType, typename SumType, typename SumType2>
unsigned int Real_Satd_4x4_C(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 tmp[4][2];
    SumType2 a0, a1, a2, a3, b0, b1;
    SumType2 sum = 0;

    for (int i = 0; i < 4; i++) {
        const PixelType *pSrc = (const PixelType *)pSrc8;
        const PixelType *pRef = (const PixelType *)pRef8;

        a0 = pSrc[0] - pRef[0];
        a1 = pSrc[1] - pRef[1];
        b0 = (a0 + a1) + ((a0 - a1) << bitsPerSum);
        a2 = pSrc[2] - pRef[2];
        a3 = pSrc[3] - pRef[3];
        b1 = (a2 + a3) + ((a2 - a3) << bitsPerSum);
        tmp[i][0] = b0 + b1;
        tmp[i][1] = b0 - b1;

        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }

    for (int i = 0; i < 2; i++) {
        HADAMARD4( a0, a1, a2, a3, tmp[0][i], tmp[1][i], tmp[2][i], tmp[3][i] );
        a0 = abs2<SumType, SumType2>(a0) + abs2<SumType, SumType2>(a1) + abs2<SumType, SumType2>(a2) + abs2<SumType, SumType2>(a3);
        sum += ((SumType)a0) + (a0 >> bitsPerSum);
    }

    return sum >> 1;
}


template <typename PixelType>
unsigned int Satd_4x4_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (sizeof(PixelType) == 1)
        return Real_Satd_4x4_C<PixelType, uint16_t, uint32_t>(pSrc, nSrcPitch, pRef, nRefPitch);
    else
        return Real_Satd_4x4_C<PixelType, uint32_t, uint64_t>(pSrc, nSrcPitch, pRef, nRefPitch);
}


template <typename PixelType, typename SumType, typename SumType2>
unsigned int Real_Satd_8x4_C(const uint8_t *pSrc8, intptr_t nSrcPitch, const uint8_t *pRef8, intptr_t nRefPitch) {
    int bitsPerSum = 8 * sizeof(SumType);

    SumType2 tmp[4][4];
    SumType2 a0, a1, a2, a3;
    SumType2 sum = 0;

    for (int i = 0; i < 4; i++) {
        const PixelType *pSrc = (const PixelType *)pSrc8;
        const PixelType *pRef = (const PixelType *)pRef8;

        a0 = (pSrc[0] - pRef[0]) + ((SumType2)(pSrc[4] - pRef[4]) << bitsPerSum);
        a1 = (pSrc[1] - pRef[1]) + ((SumType2)(pSrc[5] - pRef[5]) << bitsPerSum);
        a2 = (pSrc[2] - pRef[2]) + ((SumType2)(pSrc[6] - pRef[6]) << bitsPerSum);
        a3 = (pSrc[3] - pRef[3]) + ((SumType2)(pSrc[7] - pRef[7]) << bitsPerSum);
        HADAMARD4( tmp[i][0], tmp[i][1], tmp[i][2], tmp[i][3], a0, a1, a2, a3 );

        pSrc8 += nSrcPitch;
        pRef8 += nRefPitch;
    }
    for (int i = 0; i < 4; i++) {
        HADAMARD4( a0, a1, a2, a3, tmp[0][i], tmp[1][i], tmp[2][i], tmp[3][i] );
        sum += abs2<SumType, SumType2>(a0) + abs2<SumType, SumType2>(a1) + abs2<SumType, SumType2>(a2) + abs2<SumType, SumType2>(a3);
    }

    return (((SumType)sum) + (sum >> bitsPerSum)) >> 1;
}


template <typename PixelType>
unsigned int Satd_8x4_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (sizeof(PixelType) == 1)
        return Real_Satd_8x4_C<PixelType, uint16_t, uint32_t>(pSrc, nSrcPitch, pRef, nRefPitch);
    else
        return Real_Satd_8x4_C<PixelType, uint32_t, uint64_t>(pSrc, nSrcPitch, pRef, nRefPitch);
}


// Only handles 4x4, 8x4, 8x8, 8x16, 16x8, and 16x16.
template <int nBlkWidth, int nBlkHeight, typename PixelType>
unsigned int Satd_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
    if (nBlkWidth == 4 && nBlkHeight == 4)
        return Satd_4x4_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch);
    else if (nBlkWidth == 8 && nBlkHeight == 4)
        return Satd_8x4_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch);
    else {
        int bytesPerSample = sizeof(PixelType);

        int sum = Satd_8x4_C<PixelType>(pSrc, nSrcPitch, pRef, nRefPitch)
                + Satd_8x4_C<PixelType>(pSrc + 4*nSrcPitch, nSrcPitch, pRef + 4*nRefPitch, nRefPitch);

        if (nBlkWidth == 16)
            sum += Satd_8x4_C<PixelType>(pSrc + 8*bytesPerSample, nSrcPitch, pRef + 8*bytesPerSample, nRefPitch)
                + Satd_8x4_C<PixelType>(pSrc + 8*bytesPerSample + 4*nSrcPitch, nSrcPitch, pRef + 8*bytesPerSample + 4*nSrcPitch, nRefPitch);

        if (nBlkHeight == 16)
            sum += Satd_8x4_C<PixelType>(pSrc + 8*nSrcPitch, nSrcPitch, pRef + 8*nRefPitch, nRefPitch)
                + Satd_8x4_C<PixelType>(pSrc + 12*nSrcPitch, nSrcPitch, pRef + 12*nRefPitch, nRefPitch);

        if (nBlkWidth == 16 && nBlkHeight == 16)
            sum += Satd_8x4_C<PixelType>(pSrc + 8*bytesPerSample + 8*nSrcPitch, nSrcPitch, pRef + 8*bytesPerSample + 8*nRefPitch, nRefPitch)
                + Satd_8x4_C<PixelType>(pSrc + 8*bytesPerSample + 12*nSrcPitch, nSrcPitch, pRef + 8*bytesPerSample + 12*nRefPitch, nRefPitch);

        return sum;
    }
}


#define MK_CFUNC(functionname) extern "C" unsigned int  functionname (const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch)

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

/* included from x264 */
#define SAD_x264(blsizex, blsizey) extern "C" unsigned int  mvtools_pixel_sad_##blsizex##x##blsizey##_mmx2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch)
//mvtools_pixel_sad_16x16_mmx2(   x,y can be: 16 8 4
SAD_x264(16,16);
SAD_x264(16,8);
SAD_x264(8,16);
SAD_x264(8,8);
SAD_x264(8,4);
SAD_x264(4,8);
SAD_x264(4,4);
#undef SAD_x264
//parameter is function name
MK_CFUNC(mvtools_pixel_sad_8x16_sse2);
MK_CFUNC(mvtools_pixel_sad_16x16_sse2); //non optimized cache access, for AMD?
MK_CFUNC(mvtools_pixel_sad_16x8_sse2);     //non optimized cache access, for AMD?
MK_CFUNC(mvtools_pixel_sad_16x16_sse3); //LDDQU Pentium4E (Core1?), not for Core2!
MK_CFUNC(mvtools_pixel_sad_16x8_sse3);  //LDDQU Pentium4E (Core1?), not for Core2!
MK_CFUNC(mvtools_pixel_sad_16x16_cache64_ssse3);//core2 optimized
MK_CFUNC(mvtools_pixel_sad_16x8_cache64_ssse3); //core2 optimized

MK_CFUNC(mvtools_pixel_sad_8x16_cache64_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x8_cache64_mmx2);
MK_CFUNC(mvtools_pixel_sad_8x4_cache64_mmx2);

//1.9.5.3: added ssd & SATD (TSchniede)
/* alternative to SAD - SSD: squared sum of differences, VERY sensitive to noise */
MK_CFUNC(mvtools_pixel_ssd_16x16_mmx);
MK_CFUNC(mvtools_pixel_ssd_16x8_mmx);
MK_CFUNC(mvtools_pixel_ssd_8x16_mmx);
MK_CFUNC(mvtools_pixel_ssd_8x8_mmx);
MK_CFUNC(mvtools_pixel_ssd_8x4_mmx);
MK_CFUNC(mvtools_pixel_ssd_4x8_mmx);
MK_CFUNC(mvtools_pixel_ssd_4x4_mmx);

/* SATD: Sum of Absolute Transformed Differences, more sensitive to noise, frequency domain based - replacement to dct/SAD */
MK_CFUNC(mvtools_pixel_satd_16x16_mmx2);
MK_CFUNC(mvtools_pixel_satd_16x8_mmx2);
MK_CFUNC(mvtools_pixel_satd_8x16_mmx2);
MK_CFUNC(mvtools_pixel_satd_8x8_mmx2);
MK_CFUNC(mvtools_pixel_satd_8x4_mmx2);
MK_CFUNC(mvtools_pixel_satd_4x8_mmx2);
MK_CFUNC(mvtools_pixel_satd_4x4_mmx2);

#define SATD_SSE2(blsizex, blsizey) extern "C" unsigned int  mvtools_pixel_satd_##blsizex##x##blsizey##_sse2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch)
#define SATD_SSSE3(blsizex, blsizey) extern "C" unsigned int  mvtools_pixel_satd_##blsizex##x##blsizey##_ssse3(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch)

//mvtools_pixel_satd_16x16_%1
SATD_SSE2(16, 16);
SATD_SSE2(16,  8);
SATD_SSE2( 8, 16);
SATD_SSE2( 8,  8);
SATD_SSE2( 8,  4);
SATD_SSSE3(16, 16);
SATD_SSSE3(16,  8);
SATD_SSSE3( 8, 16);
SATD_SSSE3( 8,  8);
SATD_SSSE3( 8,  4);
#undef SATD_SSE2
#undef SATD_SSSE3

//dummy for testing and deactivate SAD
MK_CFUNC(SadDummy);
#undef MK_CFUNC


#endif
