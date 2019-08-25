// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - bicubic, wiener
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

#include <stdio.h>

#include <VSHelper.h>

#include "CPU.h"
#include "MVFrame.h"

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#if defined(MVTOOLS_X86)

#include <emmintrin.h>

#define zeroes _mm_setzero_si128()

/* TODO: port these
   extern "C" void  VerticalBicubic_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nWidth, intptr_t nHeight);
   extern "C" void  HorizontalBicubic_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nWidth, intptr_t nHeight);
   extern "C" void  RB2F_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nSrcPitch, intptr_t nWidth, intptr_t nHeight);
   extern "C" void  RB2FilteredVerticalLine_SSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
   extern "C" void  RB2FilteredHorizontalInplaceLine_SSE(uint8_t *pSrc, intptr_t nWidthMMX);
   */

void Average2_avx2(uint8_t *pDst, const uint8_t *pSrc1, const uint8_t *pSrc2, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight);
void VerticalBilinear_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                           intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);
void HorizontalBilinear_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                             intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);
void DiagonalBilinear_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                           intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);
void VerticalWiener_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                         intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);
void HorizontalWiener_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                           intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) ;

static void Average2_sse2(uint8_t *pDst, const uint8_t *pSrc1, const uint8_t *pSrc2, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight) {
    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x += 16) {
            __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc1[x]);
            __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc2[x]);

            m0 = _mm_avg_epu8(m0, m1);
            _mm_storeu_si128((__m128i *)&pDst[x], m0);
        }

        pSrc1 += nPitch;
        pSrc2 += nPitch;
        pDst += nPitch;
    }
}


static void VerticalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                  intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight - 1; y++) {
        for (int x = 0; x < nWidth; x += 16) {
            __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc[x]);
            __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc[x + nPitch]);

            m0 = _mm_avg_epu8(m0, m1);
            _mm_storeu_si128((__m128i *)&pDst[x], m0);
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int x = 0; x < nWidth; x++)
        pDst[x] = pSrc[x];
}


static void HorizontalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                    intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x += 16) {
            __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc[x]);
            __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc[x + 1]);

            m0 = _mm_avg_epu8(m0, m1);
            _mm_storeu_si128((__m128i *)&pDst[x], m0);
        }

        pDst[nWidth - 1] = pSrc[nWidth - 1];

        pSrc += nPitch;
        pDst += nPitch;
    }
}


static void DiagonalBilinear_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                  intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight - 1; y++) {
        for (int x = 0; x < nWidth; x += 8) {
            __m128i m0 = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
            __m128i m1 = _mm_loadl_epi64((const __m128i *)&pSrc[x + 1]);
            __m128i m2 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nPitch]);
            __m128i m3 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nPitch + 1]);

            m0 = _mm_unpacklo_epi8(m0, zeroes);
            m1 = _mm_unpacklo_epi8(m1, zeroes);
            m2 = _mm_unpacklo_epi8(m2, zeroes);
            m3 = _mm_unpacklo_epi8(m3, zeroes);

            m0 = _mm_add_epi16(m0, m1);
            m2 = _mm_add_epi16(m2, m3);
            m0 = _mm_add_epi16(m0, _mm_set1_epi16(2));
            m0 = _mm_add_epi16(m0, m2);

            m0 = _mm_srli_epi16(m0, 2);

            m0 = _mm_packus_epi16(m0, m0);
            _mm_storel_epi64((__m128i *)&pDst[x], m0);
        }

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth - 1 + nPitch] + 1) >> 1;

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int x = 0; x < nWidth; x += 8) {
        __m128i m0 = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
        __m128i m1 = _mm_loadl_epi64((const __m128i *)&pSrc[x + 1]);

        m0 = _mm_avg_epu8(m0, m1);
        _mm_storel_epi64((__m128i *)&pDst[x], m0);
    }

    pDst[nWidth - 1] = pSrc[nWidth - 1];
}


static void RB2CubicHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX) {
    __m128i words_255 = _mm_set1_epi16(255);

    for (int x = 1; x < nWidthMMX; x += 8) {
        __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 - 2]);
        __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 - 1]);
        __m128i m2 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2]);
        __m128i m3 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 1]);
        __m128i m4 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 2]);
        __m128i m5 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 3]);

        m0 = _mm_and_si128(m0, words_255);
        m1 = _mm_and_si128(m1, words_255);
        m2 = _mm_and_si128(m2, words_255);
        m3 = _mm_and_si128(m3, words_255);
        m4 = _mm_and_si128(m4, words_255);
        m5 = _mm_and_si128(m5, words_255);

        m2 = _mm_add_epi16(m2, m3);
        m3 = _mm_slli_epi16(m2, 3);
        m2 = _mm_slli_epi16(m2, 1);
        m2 = _mm_add_epi16(m2, m3);

        m1 = _mm_add_epi16(m1, m4);
        m4 = _mm_slli_epi16(m1, 2);
        m1 = _mm_add_epi16(m1, m4);

        m2 = _mm_add_epi16(m2, m1);
        m2 = _mm_add_epi16(m2, m0);
        m2 = _mm_add_epi16(m2, m5);

        m2 = _mm_add_epi16(m2, _mm_set1_epi16(16));
        m2 = _mm_srli_epi16(m2, 5);
        m2 = _mm_packus_epi16(m2, m2);
        _mm_storel_epi64((__m128i *)&pSrc[x], m2);
    }
}


static void RB2CubicVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX) {
    for (int x = 0; x < nWidthMMX; x += 8) {
        __m128i m0 = _mm_loadl_epi64((const __m128i *)&pSrc[x - nSrcPitch * 2]);
        __m128i m1 = _mm_loadl_epi64((const __m128i *)&pSrc[x - nSrcPitch]);
        __m128i m2 = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
        __m128i m3 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch]);
        __m128i m4 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch * 2]);
        __m128i m5 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch * 3]);

        m0 = _mm_unpacklo_epi8(m0, zeroes);
        m1 = _mm_unpacklo_epi8(m1, zeroes);
        m2 = _mm_unpacklo_epi8(m2, zeroes);
        m3 = _mm_unpacklo_epi8(m3, zeroes);
        m4 = _mm_unpacklo_epi8(m4, zeroes);
        m5 = _mm_unpacklo_epi8(m5, zeroes);

        m2 = _mm_add_epi16(m2, m3);
        m3 = _mm_slli_epi16(m2, 3);
        m2 = _mm_slli_epi16(m2, 1);
        m2 = _mm_add_epi16(m2, m3);

        m1 = _mm_add_epi16(m1, m4);
        m4 = _mm_slli_epi16(m1, 2);
        m1 = _mm_add_epi16(m1, m4);

        m2 = _mm_add_epi16(m2, m1);
        m2 = _mm_add_epi16(m2, m0);
        m2 = _mm_add_epi16(m2, m5);

        m2 = _mm_add_epi16(m2, _mm_set1_epi16(16));
        m2 = _mm_srli_epi16(m2, 5);
        m2 = _mm_packus_epi16(m2, m2);
        _mm_storel_epi64((__m128i *)&pDst[x], m2);
    }
}


static void RB2QuadraticHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX) {
    __m128i words_255 = _mm_set1_epi16(255);

    for (int x = 1; x < nWidthMMX; x += 8) {
        __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 - 2]);
        __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 - 1]);
        __m128i m2 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2]);
        __m128i m3 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 1]);
        __m128i m4 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 2]);
        __m128i m5 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 3]);

        m0 = _mm_and_si128(m0, words_255);
        m1 = _mm_and_si128(m1, words_255);
        m2 = _mm_and_si128(m2, words_255);
        m3 = _mm_and_si128(m3, words_255);
        m4 = _mm_and_si128(m4, words_255);
        m5 = _mm_and_si128(m5, words_255);

        m2 = _mm_add_epi16(m2, m3);
        m2 = _mm_mullo_epi16(m2, _mm_set1_epi16(22));

        m1 = _mm_add_epi16(m1, m4);
        m4 = _mm_slli_epi16(m1, 3);
        m1 = _mm_add_epi16(m1, m4);

        m2 = _mm_add_epi16(m2, m1);
        m2 = _mm_add_epi16(m2, m0);
        m2 = _mm_add_epi16(m2, m5);

        m2 = _mm_add_epi16(m2, _mm_set1_epi16(32));
        m2 = _mm_srli_epi16(m2, 6);
        m2 = _mm_packus_epi16(m2, m2);
        _mm_storel_epi64((__m128i *)&pSrc[x], m2);
    }
}


static void RB2QuadraticVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX) {
    for (int x = 0; x < nWidthMMX; x += 8) {
        __m128i m0 = _mm_loadl_epi64((const __m128i *)&pSrc[x - nSrcPitch * 2]);
        __m128i m1 = _mm_loadl_epi64((const __m128i *)&pSrc[x - nSrcPitch]);
        __m128i m2 = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
        __m128i m3 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch]);
        __m128i m4 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch * 2]);
        __m128i m5 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch * 3]);

        m0 = _mm_unpacklo_epi8(m0, zeroes);
        m1 = _mm_unpacklo_epi8(m1, zeroes);
        m2 = _mm_unpacklo_epi8(m2, zeroes);
        m3 = _mm_unpacklo_epi8(m3, zeroes);
        m4 = _mm_unpacklo_epi8(m4, zeroes);
        m5 = _mm_unpacklo_epi8(m5, zeroes);

        m2 = _mm_add_epi16(m2, m3);
        m2 = _mm_mullo_epi16(m2, _mm_set1_epi16(22));

        m1 = _mm_add_epi16(m1, m4);
        m4 = _mm_slli_epi16(m1, 3);
        m1 = _mm_add_epi16(m1, m4);

        m2 = _mm_add_epi16(m2, m1);
        m2 = _mm_add_epi16(m2, m0);
        m2 = _mm_add_epi16(m2, m5);

        m2 = _mm_add_epi16(m2, _mm_set1_epi16(32));
        m2 = _mm_srli_epi16(m2, 6);
        m2 = _mm_packus_epi16(m2, m2);
        _mm_storel_epi64((__m128i *)&pDst[x], m2);
    }
}


static void RB2BilinearFilteredVerticalLine_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX) {
    for (int x = 0; x < nWidthMMX; x += 8) {
        __m128i m0 = _mm_loadl_epi64((const __m128i *)&pSrc[x - nSrcPitch]);
        __m128i m1 = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
        __m128i m2 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch]);
        __m128i m3 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nSrcPitch * 2]);

        m0 = _mm_unpacklo_epi8(m0, zeroes);
        m1 = _mm_unpacklo_epi8(m1, zeroes);
        m2 = _mm_unpacklo_epi8(m2, zeroes);
        m3 = _mm_unpacklo_epi8(m3, zeroes);

        m1 = _mm_add_epi16(m1, m2);
        m2 = _mm_slli_epi16(m1, 1);
        m1 = _mm_add_epi16(m1, m2);

        m0 = _mm_add_epi16(m0, m1);
        m0 = _mm_add_epi16(m0, m3);
        m0 = _mm_add_epi16(m0, _mm_set1_epi16(4));
        m0 = _mm_srli_epi16(m0, 3);

        m0 = _mm_packus_epi16(m0, m0);
        _mm_storel_epi64((__m128i *)&pDst[x], m0);
    }
}


static void RB2BilinearFilteredHorizontalInplaceLine_sse2(uint8_t *pSrc, intptr_t nWidthMMX) {
    for (int x = 1; x < nWidthMMX; x += 8) {
        __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 - 1]);
        __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2]);
        __m128i m2 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 1]);
        __m128i m3 = _mm_loadu_si128((const __m128i *)&pSrc[x * 2 + 2]);

        __m128i words_255 = _mm_set1_epi16(255);

        m0 = _mm_and_si128(m0, words_255);
        m1 = _mm_and_si128(m1, words_255);
        m2 = _mm_and_si128(m2, words_255);
        m3 = _mm_and_si128(m3, words_255);

        m1 = _mm_add_epi16(m1, m2);
        m2 = _mm_slli_epi16(m1, 1);
        m1 = _mm_add_epi16(m1, m2);

        m0 = _mm_add_epi16(m0, m1);
        m0 = _mm_add_epi16(m0, m3);
        m0 = _mm_add_epi16(m0, _mm_set1_epi16(4));
        m0 = _mm_srli_epi16(m0, 3);

        m0 = _mm_packus_epi16(m0, m0);
        _mm_storel_epi64((__m128i *)&pSrc[x], m0);
    }
}


static void VerticalWiener_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < 2; y++) {
        for (int x = 0; x < nWidth; x += 16) {
            __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc[x]);
            __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc[x + nPitch]);

            m0 = _mm_avg_epu8(m0, m1);
            _mm_storeu_si128((__m128i *)&pDst[x], m0);
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int y = 2; y < nHeight - 4; y++) {
        for (int x = 0; x < nWidth; x += 8) {
            __m128i m0 = _mm_loadl_epi64((const __m128i *)&pSrc[x - nPitch * 2]);
            __m128i m1 = _mm_loadl_epi64((const __m128i *)&pSrc[x - nPitch]);
            __m128i m2 = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
            __m128i m3 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nPitch]);
            __m128i m4 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nPitch * 2]);
            __m128i m5 = _mm_loadl_epi64((const __m128i *)&pSrc[x + nPitch * 3]);

            m0 = _mm_unpacklo_epi8(m0, zeroes);
            m1 = _mm_unpacklo_epi8(m1, zeroes);
            m2 = _mm_unpacklo_epi8(m2, zeroes);
            m3 = _mm_unpacklo_epi8(m3, zeroes);
            m4 = _mm_unpacklo_epi8(m4, zeroes);
            m5 = _mm_unpacklo_epi8(m5, zeroes);

            m2 = _mm_add_epi16(m2, m3);
            m2 = _mm_slli_epi16(m2, 2);

            m1 = _mm_add_epi16(m1, m4);

            m2 = _mm_sub_epi16(m2, m1);
            m3 = _mm_slli_epi16(m2, 2);
            m2 = _mm_add_epi16(m2, m3);

            m0 = _mm_add_epi16(m0, m5);
            m0 = _mm_add_epi16(m0, m2);
            m0 = _mm_add_epi16(m0, _mm_set1_epi16(16));

            m0 = _mm_srai_epi16(m0, 5);
            m0 = _mm_packus_epi16(m0, m0);
            _mm_storel_epi64((__m128i *)&pDst[x], m0);
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int y = nHeight - 4; y < nHeight - 1; y++) {
        for (int x = 0; x < nWidth; x += 16) {
            __m128i m0 = _mm_loadu_si128((const __m128i *)&pSrc[x]);
            __m128i m1 = _mm_loadu_si128((const __m128i *)&pSrc[x + nPitch]);

            m0 = _mm_avg_epu8(m0, m1);
            _mm_storeu_si128((__m128i *)&pDst[x], m0);
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int x = 0; x < nWidth; x++)
        pDst[x] = pSrc[x];
}


static void HorizontalWiener_sse2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                                  intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight; y++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        pDst[1] = (pSrc[1] + pSrc[2] + 1) >> 1;

        for (int x = 2; x < nWidth - 4; x += 8) {
            __m128i m0 = _mm_loadl_epi64((const __m128i *)&pSrc[x - 2]);
            __m128i m1 = _mm_loadl_epi64((const __m128i *)&pSrc[x - 1]);
            __m128i m2 = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
            __m128i m3 = _mm_loadl_epi64((const __m128i *)&pSrc[x + 1]);
            __m128i m4 = _mm_loadl_epi64((const __m128i *)&pSrc[x + 2]);
            __m128i m5 = _mm_loadl_epi64((const __m128i *)&pSrc[x + 3]);

            m0 = _mm_unpacklo_epi8(m0, zeroes);
            m1 = _mm_unpacklo_epi8(m1, zeroes);
            m2 = _mm_unpacklo_epi8(m2, zeroes);
            m3 = _mm_unpacklo_epi8(m3, zeroes);
            m4 = _mm_unpacklo_epi8(m4, zeroes);
            m5 = _mm_unpacklo_epi8(m5, zeroes);

            m2 = _mm_add_epi16(m2, m3);
            m2 = _mm_slli_epi16(m2, 2);

            m1 = _mm_add_epi16(m1, m4);

            m2 = _mm_sub_epi16(m2, m1);
            m3 = _mm_slli_epi16(m2, 2);
            m2 = _mm_add_epi16(m2, m3);

            m0 = _mm_add_epi16(m0, m5);
            m0 = _mm_add_epi16(m0, m2);
            m0 = _mm_add_epi16(m0, _mm_set1_epi16(16));

            m0 = _mm_srai_epi16(m0, 5);
            m0 = _mm_packus_epi16(m0, m0);
            _mm_storel_epi64((__m128i *)&pDst[x], m0);
        }

        for (int x = nWidth - 4; x < nWidth - 1; x++)
            pDst[x] = (pSrc[x] + pSrc[x + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];

        pDst += nPitch;
        pSrc += nPitch;
    }
}

#endif // MVTOOLS_X86


template <typename PixelType>
static void VerticalBilinear(uint8_t *pDst8, const uint8_t *pSrc8,
                             intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    /* last row */
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
static void HorizontalBilinear(uint8_t *pDst8, const uint8_t *pSrc8,
                               intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nPitch;
        pSrc += nPitch;
    }
}


template <typename PixelType>
static void DiagonalBilinear(uint8_t *pDst8, const uint8_t *pSrc8,
                             intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nPitch] + pSrc[i + nPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nPitch - 1] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int i = 0; i < nWidth - 1; i++)
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}


template <typename PixelType>
static void RB2F_C(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                   int nSrcPitch, int nWidth, int nHeight, int opt) {
    (void)opt;

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x * 2] + pSrc[x * 2 + 1]
                    + pSrc[x * 2 + nSrcPitch + 1] + pSrc[x * 2 + nSrcPitch] + 2) / 4;

        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}


//  Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
template <typename PixelType>
static void RB2FilteredVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight, int opt) {
    (void)opt;

    /* int nWidthMMX = (nWidth/4)*4; */

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    for (int y = 0; y < 1; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

    /* TODO: port the asm
       if (opt && nWidthMMX>=4)
       {
       for ( int y = 1; y < nHeight; y++ )
       {
       RB2FilteredVerticalLine_SSE((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);

       for ( int x = nWidthMMX; x < nWidth; x++ )
       pDst[x] = (pSrc[x-nSrcPitch] + pSrc[x]*2 + pSrc[x+nSrcPitch] + 2) /4;

       pDst += nDstPitch;
       pSrc += nSrcPitch * 2;
       }
       }
       else
       */
    {
        for (int y = 1; y < nHeight; y++) {
            for (int x = 0; x < nWidth; x++)
                pDst[x] = (pSrc[x - nSrcPitch] + pSrc[x] * 2 + pSrc[x + nSrcPitch] + 2) / 4;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
}


// Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
template <typename PixelType>
static void RB2FilteredHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int opt) {
    (void)opt;

    /* int nWidthMMX = 1 + ((nWidth-2)/4)*4; */

    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        /* TODO: port the asm
           if (opt)
           {
           RB2FilteredHorizontalInplaceLine_SSE((uint8_t *)pSrc, nWidthMMX); // very first is skipped
           for ( x = nWidthMMX; x < nWidth; x++ )
           pSrc[x] = (pSrc[x*2-1] + pSrc[x*2]*2 + pSrc[x*2+1] + 2) /4;
           }
           else
           */
        {
            for (x = 1; x < nWidth; x++)
                pSrc[x] = (pSrc[x * 2 - 1] + pSrc[x * 2] * 2 + pSrc[x * 2 + 1] + 2) / 4;
        }
        pSrc[0] = pSrc0;

        pSrc += nSrcPitch;
    }
}


// separable Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
template <typename PixelType>
static void RB2Filtered(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
                        int nSrcPitch, int nWidth, int nHeight, int opt) {
    RB2FilteredVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, opt); /* intermediate half height */
    RB2FilteredHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, opt);             /* inpace width reduction */
}


//  BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
template <typename PixelType>
static void RB2BilinearFilteredVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                                        int nSrcPitch, int nWidth, int nHeight, int opt) {

    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = (nWidth / 8) * 8;

    for (int y = 0; y < 1 && y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

    for (int y = 1; y < nHeight - 1; y++) {
        int xstart = 0;

        if (sizeof(PixelType) == 1 && opt && nWidthMMX >= 8) {
#if defined(MVTOOLS_X86)
            RB2BilinearFilteredVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);
            xstart = nWidthMMX;
#endif
        }
        for (int x = xstart; x < nWidth; x++)
            pDst[x] = (pSrc[x - nSrcPitch] + (pSrc[x] + pSrc[x + nSrcPitch]) * 3 + pSrc[x + nSrcPitch * 2] + 4) / 8;
\
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}


// BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
template <typename PixelType>
static void RB2BilinearFilteredHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int opt) {

    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8;

    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        int xstart = 1;

        if (sizeof(PixelType) == 1 && opt) {
#if defined(MVTOOLS_X86)
            RB2BilinearFilteredHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX); /* very first is skipped */
            xstart = nWidthMMX;
#endif
        }
        for (x = xstart; x < nWidth - 1; x++)
            pSrc[x] = (pSrc[x * 2 - 1] + (pSrc[x * 2] + pSrc[x * 2 + 1]) * 3 + pSrc[x * 2 + 2] + 4) / 8;

        pSrc[0] = pSrc0;

        for (x = max(nWidth - 1, 1); x < nWidth; x++)
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        pSrc += nSrcPitch;
    }
}


// separable BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
template <typename PixelType>
static void RB2BilinearFiltered(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
                                int nSrcPitch, int nWidth, int nHeight, int opt) {
    RB2BilinearFilteredVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, opt); /* intermediate half height */
    RB2BilinearFilteredHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, opt);             /* inpace width reduction */
}


// filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
template <typename PixelType>
static void RB2QuadraticVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                                 int nSrcPitch, int nWidth, int nHeight, int opt) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = (nWidth / 8) * 8;

    for (int y = 0; y < 1 && y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

    for (int y = 1; y < nHeight - 1; y++) {
        int xstart = 0;

        if (sizeof(PixelType) == 1 && opt && nWidthMMX >= 8) {
#if defined(MVTOOLS_X86)
            RB2QuadraticVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);
            xstart = nWidthMMX;
#endif
        }

        for (int x = xstart; x < nWidth; x++) {
            int m0 = pSrc[x - nSrcPitch * 2];
            int m1 = pSrc[x - nSrcPitch];
            int m2 = pSrc[x];
            int m3 = pSrc[x + nSrcPitch];
            int m4 = pSrc[x + nSrcPitch * 2];
            int m5 = pSrc[x + nSrcPitch * 3];

            m2 = (m2 + m3) * 22;
            m1 = (m1 + m4) * 9;
            m0 += m5 + m2 + m1 + 32;
            m0 >>= 6;

            pDst[x] = m0;
        }

        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}


// filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
template <typename PixelType>
static void RB2QuadraticHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int opt) {
    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8;

    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrc0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; /* store temporary */

        int xstart = 1;

        if (sizeof(PixelType) == 1 && opt) {
#if defined(MVTOOLS_X86)
            RB2QuadraticHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX);
            xstart = nWidthMMX;
#endif
        }

        for (x = xstart; x < nWidth - 1; x++) {
            int m0 = pSrc[x * 2 - 2];
            int m1 = pSrc[x * 2 - 1];
            int m2 = pSrc[x * 2];
            int m3 = pSrc[x * 2 + 1];
            int m4 = pSrc[x * 2 + 2];
            int m5 = pSrc[x * 2 + 3];

            m2 = (m2 + m3) * 22;
            m1 = (m1 + m4) * 9;
            m0 += m5 + m2 + m1 + 32;
            m0 >>= 6;

            pSrc[x] = m0;
        }

        pSrc[0] = pSrc0;

        for (x = max(nWidth - 1, 1); x < nWidth; x++)
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        pSrc += nSrcPitch;
    }
}


// separable filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
template <typename PixelType>
static void RB2Quadratic(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
                         int nSrcPitch, int nWidth, int nHeight, int opt) {
    RB2QuadraticVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, opt); /* intermediate half height */
    RB2QuadraticHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, opt);             /* inpace width reduction */
}


// filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
// nHeight is dst height which is reduced by 2 source height
template <typename PixelType>
static void RB2CubicVertical(uint8_t *pDst8, const uint8_t *pSrc8, int nDstPitch,
                             int nSrcPitch, int nWidth, int nHeight, int opt) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nDstPitch /= sizeof(PixelType);
    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = (nWidth / 8) * 8;
    for (int y = 0; y < 1 && y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }

    for (int y = 1; y < nHeight - 1; y++) {
        int xstart = 0;

        if (sizeof(PixelType) == 1 && opt && nWidthMMX >= 8) {
#if defined(MVTOOLS_X86)
            RB2CubicVerticalLine_sse2((uint8_t *)pDst, (const uint8_t *)pSrc, nSrcPitch, nWidthMMX);
            xstart = nWidthMMX;
#endif
        }

        for (int x = xstart; x < nWidth; x++) {
            int m0 = pSrc[x - nSrcPitch * 2];
            int m1 = pSrc[x - nSrcPitch];
            int m2 = pSrc[x];
            int m3 = pSrc[x + nSrcPitch];
            int m4 = pSrc[x + nSrcPitch * 2];
            int m5 = pSrc[x + nSrcPitch * 3];

            m2 = (m2 + m3) * 10;
            m1 = (m1 + m4) * 5;
            m0 += m5 + m2 + m1 + 16;
            m0 >>= 5;

            pDst[x] = m0;
        }

        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
    for (int y = max(nHeight - 1, 1); y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++)
            pDst[x] = (pSrc[x] + pSrc[x + nSrcPitch] + 1) / 2;
        pDst += nDstPitch;
        pSrc += nSrcPitch * 2;
    }
}



// filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
// nWidth is dst height which is reduced by 2 source width
template <typename PixelType>
static void RB2CubicHorizontalInplace(uint8_t *pSrc8, int nSrcPitch, int nWidth, int nHeight, int opt) {
    PixelType *pSrc = (PixelType *)pSrc8;

    nSrcPitch /= sizeof(PixelType);

    int nWidthMMX = 1 + ((nWidth - 2) / 8) * 8;
    for (int y = 0; y < nHeight; y++) {
        int x = 0;
        int pSrcw0 = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2; /* store temporary */

        int xstart = 1;

        if (sizeof(PixelType) == 1 && opt) {
#if defined(MVTOOLS_X86)
            RB2CubicHorizontalInplaceLine_sse2((uint8_t *)pSrc, nWidthMMX);
            xstart = nWidthMMX;
#endif
        }

        for (x = xstart; x < nWidth - 1; x++) {
            int m0 = pSrc[x * 2 - 2];
            int m1 = pSrc[x * 2 - 1];
            int m2 = pSrc[x * 2];
            int m3 = pSrc[x * 2 + 1];
            int m4 = pSrc[x * 2 + 2];
            int m5 = pSrc[x * 2 + 3];

            m2 = (m2 + m3) * 10;
            m1 = (m1 + m4) * 5;
            m0 += m5 + m2 + m1 + 16;
            m0 >>= 5;

            pSrc[x] = m0;
        }

        pSrc[0] = pSrcw0;

        for (x = max(nWidth - 1, 1); x < nWidth; x++)
            pSrc[x] = (pSrc[x * 2] + pSrc[x * 2 + 1] + 1) / 2;

        pSrc += nSrcPitch;
    }
}


// separable filtered cubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick v.2.5.2
// assume he have enough horizontal dimension for intermediate results (double as final)
template <typename PixelType>
static void RB2Cubic(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch,
                     int nSrcPitch, int nWidth, int nHeight, int opt) {
    RB2CubicVertical<PixelType>(pDst, pSrc, nDstPitch, nSrcPitch, nWidth * 2, nHeight, opt); /* intermediate half height */
    RB2CubicHorizontalInplace<PixelType>(pDst, nDstPitch, nWidth, nHeight, opt);             /* inpace width reduction */
}


// so called Wiener interpolation. (sharp, similar to Lanczos ?)
// invarint simplified, 6 taps. Weights: (1, -5, 20, 20, -5, 1)/32 - added by Fizick
template <typename PixelType>
static void VerticalWiener(uint8_t *pDst8, const uint8_t *pSrc8,
                           intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int j = 2; j < nHeight - 4; j++) {
        for (int i = 0; i < nWidth; i++) {
            int m0 = pSrc[i - nPitch * 2];
            int m1 = pSrc[i - nPitch];
            int m2 = pSrc[i];
            int m3 = pSrc[i + nPitch];
            int m4 = pSrc[i + nPitch * 2];
            int m5 = pSrc[i + nPitch * 3];

            m2 = (m2 + m3) * 4;

            m2 -= m1 + m4;
            m2 *= 5;

            m0 += m5 + m2 + 16;
            m0 >>= 5;

            pDst[i] = max(0, min(m0, pixelMax));
        }
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int j = nHeight - 4; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        }

        pDst += nPitch;
        pSrc += nPitch;
    }
    /* last row */
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
static void HorizontalWiener(uint8_t *pDst8, const uint8_t *pSrc8,
                             intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < nHeight; j++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        pDst[1] = (pSrc[1] + pSrc[2] + 1) >> 1;

        for (int i = 2; i < nWidth - 4; i++) {
            int m0 = pSrc[i - 2];
            int m1 = pSrc[i - 1];
            int m2 = pSrc[i];
            int m3 = pSrc[i + 1];
            int m4 = pSrc[i + 2];
            int m5 = pSrc[i + 3];

            m2 = (m2 + m3) * 4;

            m2 -= m1 + m4;
            m2 *= 5;

            m0 += m5 + m2 + 16;
            m0 >>= 5;

            pDst[i] = max(0, min(m0, pixelMax));
        }

        for (int i = nWidth - 4; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nPitch;
        pSrc += nPitch;
    }
}


// bicubic (Catmull-Rom 4 taps interpolation)
template <typename PixelType>
static void VerticalBicubic(uint8_t *pDst8, const uint8_t *pSrc8,
                            intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < 1; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int j = 1; j < nHeight - 3; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = min(pixelMax, max(0,
                                        (-pSrc[i - nPitch] - pSrc[i + nPitch * 2] + (pSrc[i] + pSrc[i + nPitch]) * 9 + 8) >> 4));
        }
        pDst += nPitch;
        pSrc += nPitch;
    }
    for (int j = nHeight - 3; j < nHeight - 1; j++) {
        for (int i = 0; i < nWidth; i++) {
            pDst[i] = (pSrc[i] + pSrc[i + nPitch] + 1) >> 1;
        }

        pDst += nPitch;
        pSrc += nPitch;
    }
    /* last row */
    for (int i = 0; i < nWidth; i++)
        pDst[i] = pSrc[i];
}


template <typename PixelType>
static void HorizontalBicubic(uint8_t *pDst8, const uint8_t *pSrc8,
                              intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc = (PixelType *)pSrc8;

    nPitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;

    for (int j = 0; j < nHeight; j++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        for (int i = 1; i < nWidth - 3; i++) {
            pDst[i] = min(pixelMax, max(0,
                                        (-(pSrc[i - 1] + pSrc[i + 2]) + (pSrc[i] + pSrc[i + 1]) * 9 + 8) >> 4));
        }
        for (int i = nWidth - 3; i < nWidth - 1; i++)
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nPitch;
        pSrc += nPitch;
    }
}


// assume all pitches equal
template <typename PixelType>
static void Average2(uint8_t *pDst8, const uint8_t *pSrc18, const uint8_t *pSrc28,
                     intptr_t nPitch, intptr_t nWidth, intptr_t nHeight) {
    PixelType *pDst = (PixelType *)pDst8;
    PixelType *pSrc1 = (PixelType *)pSrc18;
    PixelType *pSrc2 = (PixelType *)pSrc28;

    nPitch /= sizeof(PixelType);

    for (int j = 0; j < nHeight; j++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = (pSrc1[i] + pSrc2[i] + 1) >> 1;

        pDst += nPitch;
        pSrc1 += nPitch;
        pSrc2 += nPitch;
    }
}


#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif


int PlaneHeightLuma(int src_height, int level, int yRatioUV, int vpad) {
    int height = src_height;

    for (int i = 1; i <= level; i++) {
        height = vpad >= yRatioUV ? ((height / yRatioUV + 1) / 2) * yRatioUV : ((height / yRatioUV) / 2) * yRatioUV;
    }
    return height;
}


int PlaneWidthLuma(int src_width, int level, int xRatioUV, int hpad) {
    int width = src_width;

    for (int i = 1; i <= level; i++) {
        width = hpad >= xRatioUV ? ((width / xRatioUV + 1) / 2) * xRatioUV : ((width / xRatioUV) / 2) * xRatioUV;
    }
    return width;
}


unsigned int PlaneSuperOffset(int chroma, int src_height, int level, int pel, int vpad, int plane_pitch, int yRatioUV) {
    // storing subplanes in superframes may be implemented by various ways
    int height = src_height; // luma or chroma

    unsigned int offset;

    if (level == 0)
        offset = 0;
    else {
        offset = pel * pel * plane_pitch * (src_height + vpad * 2);

        for (int i = 1; i < level; i++) {
            height = chroma ? PlaneHeightLuma(src_height * yRatioUV, i, yRatioUV, vpad * yRatioUV) / yRatioUV : PlaneHeightLuma(src_height, i, yRatioUV, vpad);

            offset += plane_pitch * (height + vpad * 2);
        }
    }
    return offset;
}


template <typename PixelType>
static void PadCorner(PixelType *p, PixelType v, int hPad, int vPad, int refPitch) {
    for (int i = 0; i < vPad; i++) {
        if (sizeof(PixelType) == 1)
            memset(p, v, hPad); /* faster than loop */
        else
            for (int j = 0; j < hPad; j++)
                p[j] = v;

        p += refPitch;
    }
}


template <typename PixelType>
static void PadReferenceFrame(uint8_t *refFrame8, int refPitch, int hPad, int vPad, int width, int height) {
    refPitch /= sizeof(PixelType);
    PixelType *refFrame = (PixelType *)refFrame8;
    PixelType value;
    PixelType *pfoff = refFrame + vPad * refPitch + hPad;
    PixelType *p;

    /* Up-Left */
    PadCorner(refFrame, pfoff[0], hPad, vPad, refPitch);
    /* Up-Right */
    PadCorner(refFrame + hPad + width, pfoff[width - 1], hPad, vPad, refPitch);
    /* Down-Left */
    PadCorner(refFrame + (vPad + height) * refPitch,
              pfoff[(height - 1) * refPitch], hPad, vPad, refPitch);
    /* Down-Right */
    PadCorner(refFrame + hPad + width + (vPad + height) * refPitch,
              pfoff[(height - 1) * refPitch + width - 1], hPad, vPad, refPitch);

    /* Up */
    for (int i = 0; i < width; i++) {
        value = pfoff[i];
        p = refFrame + hPad + i;
        for (int j = 0; j < vPad; j++) {
            p[0] = value;
            p += refPitch;
        }
    }

    /* Left */
    for (int i = 0; i < height; i++) {
        value = pfoff[i * refPitch];
        p = refFrame + (vPad + i) * refPitch;
        for (int j = 0; j < hPad; j++)
            p[j] = value;
    }

    /* Right */
    for (int i = 0; i < height; i++) {
        value = pfoff[i * refPitch + width - 1];
        p = refFrame + (vPad + i) * refPitch + width + hPad;
        for (int j = 0; j < hPad; j++)
            p[j] = value;
    }

    /* Down */
    for (int i = 0; i < width; i++) {
        value = pfoff[i + (height - 1) * refPitch];
        p = refFrame + hPad + i + (height + vPad) * refPitch;
        for (int j = 0; j < vPad; j++) {
            p[0] = value;
            p += refPitch;
        }
    }
}


/******************************************************************************
 *                                                                             *
 *  MVPlane : manages a single plane, allowing padding and refinin             *
 *                                                                             *
 ******************************************************************************/

void mvpInit(MVPlane *mvp, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int opt, int bitsPerSample) {
    mvp->nWidth = nWidth;
    mvp->nHeight = nHeight;
    mvp->nPel = nPel;
    mvp->nHPadding = nHPad;
    mvp->nVPadding = nVPad;
    mvp->opt = opt;
    mvp->nHPaddingPel = nHPad * nPel;
    mvp->nVPaddingPel = nVPad * nPel;
    mvp->bitsPerSample = bitsPerSample;
    mvp->bytesPerSample = (bitsPerSample + 7) / 8; // Who would ever want to process 32 bit video?

    mvp->nPaddedWidth = nWidth + 2 * mvp->nHPadding;
    mvp->nPaddedHeight = nHeight + 2 * mvp->nVPadding;

    mvp->pPlane = (uint8_t **)malloc(nPel * nPel * sizeof(uint8_t *));
}


void mvpDeinit(MVPlane *mvp) {
    free(mvp->pPlane);
}


void mvpResetState(MVPlane *mvp) {
    mvp->isRefined = mvp->isFilled = mvp->isPadded = 0;
}


void mvpUpdate(MVPlane *mvp, uint8_t *pSrc, int _nPitch) { //v2.0
    mvp->nPitch = _nPitch;
    mvp->nOffsetPadding = mvp->nPitch * mvp->nVPadding + mvp->nHPadding * mvp->bytesPerSample;

    for (int i = 0; i < mvp->nPel * mvp->nPel; i++)
        mvp->pPlane[i] = pSrc + i * mvp->nPitch * mvp->nPaddedHeight;

    mvpResetState(mvp);
}


void mvpFillPlane(MVPlane *mvp, const uint8_t *pNewPlane, int nNewPitch) {
    if (!mvp->isFilled)
        vs_bitblt(mvp->pPlane[0] + mvp->nOffsetPadding, mvp->nPitch, pNewPlane, nNewPitch, mvp->nWidth * mvp->bytesPerSample, mvp->nHeight);
    mvp->isFilled = 1;
}


void mvpPad(MVPlane *mvp) {
    if (!mvp->isPadded) {
        if (mvp->bytesPerSample == 1)
            PadReferenceFrame<uint8_t>(mvp->pPlane[0], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight);
        else
            PadReferenceFrame<uint16_t>(mvp->pPlane[0], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight);
    }

    mvp->isPadded = 1;
}


void mvpRefine(MVPlane *mvp, int sharp) {
    if (mvp->isRefined)
        return;

    if (mvp->nPel == 1) {
        mvp->isRefined = 1;
        return;
    }

    typedef void (*RefineFunction)(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);

    RefineFunction refine[3];

    if (sharp == SharpBilinear) {
        if (mvp->bytesPerSample == 1) {
            refine[0] = HorizontalBilinear<uint8_t>;
            refine[1] = VerticalBilinear<uint8_t>;
            refine[2] = DiagonalBilinear<uint8_t>;

            if (mvp->opt) {
#if defined(MVTOOLS_X86)
                refine[0] = HorizontalBilinear_sse2;
                refine[1] = VerticalBilinear_sse2;
                refine[2] = DiagonalBilinear_sse2;

                if (mvp->opt >= MVOPT_AVX2 && (g_cpuinfo & X264_CPU_AVX2)) {
                    refine[0] = HorizontalBilinear_avx2;
                    refine[1] = VerticalBilinear_avx2;
                    refine[2] = DiagonalBilinear_avx2;
                }
#endif
            }
        } else {
            refine[0] = HorizontalBilinear<uint16_t>;
            refine[1] = VerticalBilinear<uint16_t>;
            refine[2] = DiagonalBilinear<uint16_t>;
        }
    } else if (sharp == SharpBicubic) {
        if (mvp->bytesPerSample == 1) {
            /* TODO: port the asm
               if (opt)
               {
               f[0] = f[2] = HorizontalBicubic_iSSE;
               f[1] = VerticalBicubic_iSSE;
               }
               else
               */
            refine[0] = refine[2] = HorizontalBicubic<uint8_t>;
            refine[1] = VerticalBicubic<uint8_t>;
        } else {
            refine[0] = refine[2] = HorizontalBicubic<uint16_t>;
            refine[1] = VerticalBicubic<uint16_t>;
        }
    } else { // Wiener
        if (mvp->bytesPerSample == 1) {
            refine[0] = refine[2] = HorizontalWiener<uint8_t>;
            refine[1] = VerticalWiener<uint8_t>;

            if (mvp->opt) {
#if defined(MVTOOLS_X86)
                refine[0] = refine[2] = HorizontalWiener_sse2;
                refine[1] = VerticalWiener_sse2;

                if (mvp->opt >= MVOPT_AVX2 && (g_cpuinfo & X264_CPU_AVX2)) {
                    refine[0] = refine[2] = HorizontalWiener_avx2;
                    refine[1] = VerticalWiener_avx2;
                }
#endif
            }
        } else {
            refine[0] = refine[2] = HorizontalWiener<uint16_t>;
            refine[1] = VerticalWiener<uint16_t>;
        }
    }

    const uint8_t *src[3];
    uint8_t *dst[3];

    if (mvp->nPel == 2) {
        dst[0] = mvp->pPlane[1];
        dst[1] = mvp->pPlane[2];
        dst[2] = mvp->pPlane[3];
        src[0] = src[1] = mvp->pPlane[0];
        if (sharp == 0)
            src[2] = mvp->pPlane[0];
        else
            src[2] = mvp->pPlane[2];
    } else if (mvp->nPel == 4) {
        dst[0] = mvp->pPlane[2];
        dst[1] = mvp->pPlane[8];
        dst[2] = mvp->pPlane[10];
        src[0] = src[1] = mvp->pPlane[0];
        if (sharp == 0)
            src[2] = mvp->pPlane[0];
        else
            src[2] = mvp->pPlane[8];
    }

    for (int i = 0; i < 3; i++)
        refine[i](dst[i], src[i], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight, mvp->bitsPerSample);

    if (mvp->nPel == 4) {
        typedef void (*AverageFunction)(uint8_t *pDst, const uint8_t *pSrc1, const uint8_t *pSrc2, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight);

        AverageFunction avg;

        if (mvp->bytesPerSample == 1) {
            avg = Average2<uint8_t>;

            if (mvp->opt) {
#if defined(MVTOOLS_X86)
                avg = Average2_sse2;

                if (mvp->opt >= MVOPT_AVX2 && (g_cpuinfo & X264_CPU_AVX2))
                    avg = Average2_avx2;
#endif
            }
        } else {
            avg = Average2<uint16_t>;
        }

        // now interpolate intermediate
        avg(mvp->pPlane[1], mvp->pPlane[0], mvp->pPlane[2], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[9], mvp->pPlane[8], mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[4], mvp->pPlane[0], mvp->pPlane[8], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[6], mvp->pPlane[2], mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[5], mvp->pPlane[4], mvp->pPlane[6], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);

        avg(mvp->pPlane[3], mvp->pPlane[0] + mvp->bytesPerSample, mvp->pPlane[2], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
        avg(mvp->pPlane[11], mvp->pPlane[8] + mvp->bytesPerSample, mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
        avg(mvp->pPlane[12], mvp->pPlane[0] + mvp->nPitch, mvp->pPlane[8], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight - 1);
        avg(mvp->pPlane[14], mvp->pPlane[2] + mvp->nPitch, mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight - 1);
        avg(mvp->pPlane[13], mvp->pPlane[12], mvp->pPlane[14], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[7], mvp->pPlane[4] + mvp->bytesPerSample, mvp->pPlane[6], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
        avg(mvp->pPlane[15], mvp->pPlane[12] + mvp->bytesPerSample, mvp->pPlane[14], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
    }

    mvp->isRefined = 1;
}


template <typename PixelType>
static void mvpRefineExtPel2(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded) {
    const PixelType *pSrc2x = (const PixelType *)pSrc2x8;
    PixelType *pp1 = (PixelType *)mvp->pPlane[1];
    PixelType *pp2 = (PixelType *)mvp->pPlane[2];
    PixelType *pp3 = (PixelType *)mvp->pPlane[3];

    nSrc2xPitch /= sizeof(PixelType);
    int nPitchTmp = mvp->nPitch / sizeof(PixelType);

    /* pel clip may be already padded (i.e. is finest clip) */
    if (!isExtPadded) {
        int offset = nPitchTmp * mvp->nVPadding + mvp->nHPadding;
        pp1 += offset;
        pp2 += offset;
        pp3 += offset;
    }

    for (int h = 0; h < mvp->nHeight; h++) {
        for (int w = 0; w < mvp->nWidth; w++) {
            pp1[w] = pSrc2x[(w << 1) + 1];
            pp2[w] = pSrc2x[(w << 1) + nSrc2xPitch];
            pp3[w] = pSrc2x[(w << 1) + nSrc2xPitch + 1];
        }
        pp1 += nPitchTmp;
        pp2 += nPitchTmp;
        pp3 += nPitchTmp;
        pSrc2x += nSrc2xPitch * 2;
    }

    if (!isExtPadded) {
        for (int i = 1; i < 4; i++)
            PadReferenceFrame<PixelType>(mvp->pPlane[i], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight);
    }

    mvp->isPadded = 1;
}


template <typename PixelType>
static void mvpRefineExtPel4(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded) {
    const PixelType *pSrc2x = (const PixelType *)pSrc2x8;
    PixelType *pp[16];
    for (int i = 1; i < 16; i++)
        pp[i] = (PixelType *)mvp->pPlane[i];

    nSrc2xPitch /= sizeof(PixelType);
    int nPitchTmp = mvp->nPitch / sizeof(PixelType);

    if (!isExtPadded) {
        int offset = nPitchTmp * mvp->nVPadding + mvp->nHPadding;
        for (int i = 1; i < 16; i++)
            pp[i] += offset;
    }

    for (int h = 0; h < mvp->nHeight; h++) {
        for (int w = 0; w < mvp->nWidth; w++) {
            pp[1][w] = pSrc2x[(w << 2) + 1];
            pp[2][w] = pSrc2x[(w << 2) + 2];
            pp[3][w] = pSrc2x[(w << 2) + 3];
            pp[4][w] = pSrc2x[(w << 2) + nSrc2xPitch];
            pp[5][w] = pSrc2x[(w << 2) + nSrc2xPitch + 1];
            pp[6][w] = pSrc2x[(w << 2) + nSrc2xPitch + 2];
            pp[7][w] = pSrc2x[(w << 2) + nSrc2xPitch + 3];
            pp[8][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2];
            pp[9][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 1];
            pp[10][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 2];
            pp[11][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 3];
            pp[12][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3];
            pp[13][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 1];
            pp[14][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 2];
            pp[15][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 3];
        }
        for (int i = 1; i < 16; i++)
            pp[i] += nPitchTmp;
        pSrc2x += nSrc2xPitch * 4;
    }
    if (!isExtPadded) {
        for (int i = 1; i < 16; i++)
            PadReferenceFrame<PixelType>(mvp->pPlane[i], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight);
    }

    mvp->isPadded = 1;
}


void mvpRefineExt(MVPlane *mvp, const uint8_t *pSrc2x, int nSrc2xPitch, int isExtPadded) // copy from external upsized clip
{
    if ((mvp->nPel == 2) && (!mvp->isRefined)) {
        if (mvp->bytesPerSample == 1)
            mvpRefineExtPel2<uint8_t>(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
        else
            mvpRefineExtPel2<uint16_t>(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
    } else if ((mvp->nPel == 4) && (!mvp->isRefined)) {
        if (mvp->bytesPerSample == 1)
            mvpRefineExtPel4<uint8_t>(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
        else
            mvpRefineExtPel4<uint16_t>(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
    }

    mvp->isRefined = 1;
}


void mvpReduceTo(MVPlane *mvp, MVPlane *pReducedPlane, int rfilter) {
    if (pReducedPlane->isFilled)
        return;

    typedef void (*ReduceFunction)(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch, int nSrcPitch, int nWidth, int nHeight, int opt);

    ReduceFunction reduce = NULL;

    if (rfilter == RfilterSimple) {
        /* TODO: port the asm
               if (opt)
               {
               RB2F_iSSE(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
               pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight);
               }
               else
               */
        {
            if (mvp->bytesPerSample == 1)
                reduce = RB2F_C<uint8_t>;
            else
                reduce = RB2F_C<uint16_t>;
        }
    } else if (rfilter == RfilterTriangle) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2Filtered<uint8_t>;
        else
            reduce = RB2Filtered<uint16_t>;
    } else if (rfilter == RfilterBilinear) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2BilinearFiltered<uint8_t>;
        else
            reduce = RB2BilinearFiltered<uint16_t>;
    } else if (rfilter == RfilterQuadratic) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2Quadratic<uint8_t>;
        else
            reduce = RB2Quadratic<uint16_t>;
    } else if (rfilter == RfilterCubic) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2Cubic<uint8_t>;
        else
            reduce = RB2Cubic<uint16_t>;
    }

    reduce(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, mvp->pPlane[0] + mvp->nOffsetPadding,
           pReducedPlane->nPitch, mvp->nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, mvp->opt);

    pReducedPlane->isFilled = 1;
}


const uint8_t *mvpGetAbsolutePointer(const MVPlane *mvp, int nX, int nY) {
    if (mvp->nPel == 1)
        return mvp->pPlane[0] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    else if (mvp->nPel == 2) {
        int idx = (nX & 1) | ((nY & 1) << 1);

        nX >>= 1;
        nY >>= 1;

        return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    } else { // nPel = 4
        int idx = (nX & 3) | ((nY & 3) << 2);

        nX >>= 2;
        nY >>= 2;

        return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    }
}


const uint8_t *mvpGetAbsolutePointerPel1(const MVPlane *mvp, int nX, int nY) {
    return mvp->pPlane[0] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
}


const uint8_t *mvpGetAbsolutePointerPel2(const MVPlane *mvp, int nX, int nY) {
    int idx = (nX & 1) | ((nY & 1) << 1);

    nX >>= 1;
    nY >>= 1;

    return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
}


const uint8_t *mvpGetAbsolutePointerPel4(const MVPlane *mvp, int nX, int nY) {
    int idx = (nX & 3) | ((nY & 3) << 2);

    nX >>= 2;
    nY >>= 2;

    return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
}


const uint8_t *mvpGetPointer(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointer(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetPointerPel1(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointerPel1(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetPointerPel2(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointerPel2(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetPointerPel4(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointerPel4(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetAbsolutePelPointer(const MVPlane *mvp, int nX, int nY) {
    const uint8_t *ret = mvp->pPlane[0] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    return ret;
}

/******************************************************************************
 *                                                                             *
 *  MVFrame : a MVFrame is a threesome of MVPlane, some undefined, some        *
 *  defined, according to the nMode value                                      *
 *                                                                             *
 ******************************************************************************/

void mvfInit(MVFrame *mvf, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int nMode, int opt, int xRatioUV, int yRatioUV, int bitsPerSample) {
    mvf->nMode = nMode;

    mvf->planes[0] = mvf->planes[1] = mvf->planes[2] = NULL;

    int width[3] = { nWidth };
    width[1] = width[2] = nWidth / xRatioUV;

    int height[3] = { nHeight };
    height[1] = height[2] = nHeight / yRatioUV;

    int hpad[3] = { nHPad };
    hpad[1] = hpad[2] = nHPad / xRatioUV;

    int vpad[3] = { nVPad };
    vpad[1] = vpad[2] = nVPad / yRatioUV;

    for (int i = 0; i < 3; i++) {
        if (nMode & (1 << i)) {
            mvf->planes[i] = (MVPlane *)malloc(sizeof(MVPlane));
            mvpInit(mvf->planes[i], width[i], height[i], nPel, hpad[i], vpad[i], opt, bitsPerSample);
        }
    }
}


void mvfDeinit(MVFrame *mvf) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i]) {
            mvpDeinit(mvf->planes[i]);
            free(mvf->planes[i]);
            mvf->planes[i] = NULL;
        }
    }
}


void mvfUpdate(MVFrame *mvf, uint8_t **pSrc, int *pitch) {
    for (int i = 0; i < 3; i++) {
        if (pSrc[i] && mvf->planes[i])
            mvpUpdate(mvf->planes[i], pSrc[i], pitch[i]);
    }
}


void mvfFillPlane(MVFrame *mvf, const uint8_t *pNewPlane, int nNewPitch, int plane) {
    if (mvf->planes[plane])
        mvpFillPlane(mvf->planes[plane], pNewPlane, nNewPitch);
}


void mvfRefine(MVFrame *mvf, MVPlaneSet nMode, int sharp) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i] && (nMode & (1 << i)))
            mvpRefine(mvf->planes[i], sharp);
    }
}


void mvfPad(MVFrame *mvf, MVPlaneSet nMode) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i] && (nMode & (1 << i)))
            mvpPad(mvf->planes[i]);
    }
}


void mvfResetState(MVFrame *mvf) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i])
            mvpResetState(mvf->planes[i]);
    }
}


void mvfReduceTo(MVFrame *mvf, MVFrame *pFrame, MVPlaneSet nMode, int rfilter) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i] && (nMode & (1 << i)))
            mvpReduceTo(mvf->planes[i], pFrame->planes[i], rfilter);
    }
}

/******************************************************************************
 *                                                                             *
 *  MVGroupOfFrames : manage a hierachal frame structure                       *
 *                                                                             *
 ******************************************************************************/

void mvgofInit(MVGroupOfFrames *mvgof, int nLevelCount, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int nMode, int opt, int xRatioUV, int yRatioUV, int bitsPerSample) {
    mvgof->nLevelCount = nLevelCount;
    mvgof->nWidth[0] = nWidth;
    mvgof->nWidth[1] = mvgof->nWidth[2] = nWidth / xRatioUV;
    mvgof->nHeight[0] = nHeight;
    mvgof->nHeight[1] = mvgof->nHeight[2] = nHeight / yRatioUV;
    mvgof->nPel = nPel;
    mvgof->nHPad[0] = nHPad;
    mvgof->nHPad[1] = mvgof->nHPad[2] = nHPad / xRatioUV;
    mvgof->nVPad[0] = nVPad;
    mvgof->nVPad[1] = mvgof->nVPad[2] = nVPad / yRatioUV;
    mvgof->xRatioUV = xRatioUV;
    mvgof->yRatioUV = yRatioUV;

    mvgof->frames = (MVFrame **)malloc(mvgof->nLevelCount * sizeof(MVFrame *));

    mvgof->frames[0] = (MVFrame *)malloc(sizeof(MVFrame));
    mvfInit(mvgof->frames[0], mvgof->nWidth[0], mvgof->nHeight[0], mvgof->nPel, mvgof->nHPad[0], mvgof->nVPad[0], nMode, opt, mvgof->xRatioUV, mvgof->yRatioUV, bitsPerSample);

    for (int i = 1; i < mvgof->nLevelCount; i++) {
        int nWidthi = PlaneWidthLuma(mvgof->nWidth[0], i, mvgof->xRatioUV, mvgof->nHPad[0]);    //(nWidthi / 2) - ((nWidthi / 2) % xRatioUV); //  even for YV12
        int nHeighti = PlaneHeightLuma(mvgof->nHeight[0], i, mvgof->yRatioUV, mvgof->nVPad[0]); //(nHeighti / 2) - ((nHeighti / 2) % yRatioUV); // even for YV12

        mvgof->frames[i] = (MVFrame *)malloc(sizeof(MVFrame));
        mvfInit(mvgof->frames[i], nWidthi, nHeighti, 1, mvgof->nHPad[0], mvgof->nVPad[0], nMode, opt, mvgof->xRatioUV, mvgof->yRatioUV, bitsPerSample);
    }
}


void mvgofDeinit(MVGroupOfFrames *mvgof) {
    for (int i = 0; i < mvgof->nLevelCount; i++) {
        mvfDeinit(mvgof->frames[i]);
        free(mvgof->frames[i]);
    }

    free(mvgof->frames);
    mvgof->frames = NULL;
}


void mvgofUpdate(MVGroupOfFrames *mvgof, uint8_t **pSrc, int *pitch) {
    for (int i = 0; i < mvgof->nLevelCount; i++) {
        uint8_t *planes[3] = { NULL };

        for (int plane = 0; plane < 3; plane++) {
            if (pSrc[plane])
                planes[plane] = pSrc[plane] + PlaneSuperOffset(plane, mvgof->nHeight[plane], i, mvgof->nPel, mvgof->nVPad[plane], pitch[plane], mvgof->yRatioUV);
        }

        mvfUpdate(mvgof->frames[i], planes, pitch);
    }
}


MVFrame *mvgofGetFrame(MVGroupOfFrames *mvgof, int nLevel) {
    if ((nLevel < 0) || (nLevel >= mvgof->nLevelCount))
        return NULL;
    return mvgof->frames[nLevel];
}


void mvgofSetPlane(MVGroupOfFrames *mvgof, const uint8_t *pNewSrc, int nNewPitch, int plane) {
    mvfFillPlane(mvgof->frames[0], pNewSrc, nNewPitch, plane);
}


void mvgofRefine(MVGroupOfFrames *mvgof, MVPlaneSet nMode, int sharp) {
    mvfRefine(mvgof->frames[0], nMode, sharp);
}


void mvgofPad(MVGroupOfFrames *mvgof, MVPlaneSet nMode) {
    mvfPad(mvgof->frames[0], nMode);
}


void mvgofReduce(MVGroupOfFrames *mvgof, MVPlaneSet nMode, int rfilter) {
    for (int i = 0; i < mvgof->nLevelCount - 1; i++) {
        mvfReduceTo(mvgof->frames[i], mvgof->frames[i + 1], nMode, rfilter);
        mvfPad(mvgof->frames[i + 1], YUVPLANES);
    }
}


void mvgofResetState(MVGroupOfFrames *mvgof) {
    for (int i = 0; i < mvgof->nLevelCount; i++)
        mvfResetState(mvgof->frames[i]);
}
