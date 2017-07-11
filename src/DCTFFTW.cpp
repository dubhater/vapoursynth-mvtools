// DCT calculation with fftw (real)
// Copyright(c)2006 A.G.Balakhnin aka Fizick
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

#include <algorithm>
#include <cmath>
#include <mutex>

#include "DCTFFTW.h"


static const float sqrt_2_div_2 = 0.70710678118654752440084436210485f;


template <typename PixelType>
static void Float2Pixels_C(const DCTFFTW *dct, uint8_t *dstp8, int dst_pitch, float *realdata) {
    PixelType *dstp = (PixelType *)dstp8;
    dst_pitch /= sizeof(PixelType);

    PixelType *dstp_orig = dstp;
    float *realdata_orig = realdata;

    int pixelMax = (1 << dct->bitsPerSample) - 1;
    int pixelHalf = 1 << (dct->bitsPerSample - 1);

    for (int j = 0; j < dct->sizey; j++) {
        for (int i = 0; i < dct->sizex; i++) {
            float f = realdata[i] * sqrt_2_div_2; // to be compatible with integer DCTINT8
            int integ = (int)(nearbyintf(f));
            dstp[i] = std::min(pixelMax, std::max(0, (integ >> dct->dctshift) + pixelHalf));
        }
        dstp += dst_pitch;
        realdata += dct->sizex;
    }

    float f = realdata_orig[0] * 0.5f; // to be compatible with integer DCTINT8
    int integ = (int)(nearbyintf(f));
    dstp_orig[0] = std::min(pixelMax, std::max(0, (integ >> dct->dctshift0) + pixelHalf)); // DC
}


#if defined(MVTOOLS_X86)

#include <emmintrin.h>

template <typename PixelType>
static void Float2Pixels_SSE2(const DCTFFTW *dct, uint8_t *dstp8, int dst_pitch, float *realdata) {
    PixelType *dstp = (PixelType *)dstp8;
    dst_pitch /= sizeof(PixelType);

    unsigned width = dct->sizex;
    unsigned height = dct->sizey;

    PixelType *dstp_orig = dstp;
    float *realdata_orig = realdata;

    int pixel_max, pixel_half, pixel_min;
    __m128i words_pixel_max, words_pixel_half, words_pixel_min;

    if (sizeof(PixelType) == 1) {
        pixel_max = 255;
        pixel_half = 128;
        pixel_min = 0;

        words_pixel_max = _mm_set1_epi16(pixel_max);
        words_pixel_half = _mm_set1_epi16(pixel_half);
        words_pixel_min = _mm_set1_epi16(pixel_min);
    } else {
        pixel_max = (1 << dct->bitsPerSample) - 1;
        pixel_half = 1 << (dct->bitsPerSample - 1);
        pixel_min = 0;

        // Shitty because of pminsw/pmaxsw.
        words_pixel_max = _mm_set1_epi16(pixel_max - pixel_half);
        words_pixel_half = _mm_set1_epi16(pixel_half);
        words_pixel_min = _mm_set1_epi16(pixel_min - pixel_half);
    }

    __m128i dwords_dctshift = _mm_cvtsi32_si128(dct->dctshift);

    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x += 4) {
            __m128 f = _mm_load_ps(&realdata[x]);
            f = _mm_mul_ps(f, _mm_set1_ps(sqrt_2_div_2));

            __m128i i = _mm_cvtps_epi32(f);
            i = _mm_sra_epi32(i, dwords_dctshift);
            i = _mm_packs_epi32(i, i);

            if (sizeof(PixelType) == 1) {
                i = _mm_add_epi16(i, words_pixel_half);
                i = _mm_packus_epi16(i, i);
                *(int *)(dstp + x) = _mm_cvtsi128_si32(i);
            } else {
                i = _mm_min_epi16(i, words_pixel_max);
                i = _mm_max_epi16(i, words_pixel_min);
                i = _mm_add_epi16(i, words_pixel_half);
                _mm_storel_epi64((__m128i *)&dstp[x], i);
            }
        }

        dstp += dst_pitch;
        realdata += width;
    }

    int i = _mm_cvtss_si32(_mm_set_ss(realdata_orig[0] * 0.5f));
    dstp_orig[0] = std::max(0, std::min((i >> dct->dctshift0) + pixel_half, pixel_max));
}

#endif // MVTOOLS_X86


std::mutex g_fftw_plans_mutex;


void dctInit(DCTFFTW *dct, int sizex, int sizey, int bitsPerSample, int opt) {
    dct->sizex = sizex;
    dct->sizey = sizey;
    dct->bitsPerSample = bitsPerSample;

    int size2d = sizey * sizex;

    int cursize = 1;
    dct->dctshift = 0;
    while (cursize < size2d) {
        dct->dctshift++;
        cursize = (cursize << 1);
    }

    dct->dctshift0 = dct->dctshift + 2;

    dct->fSrc = (float *)fftwf_malloc(sizeof(float) * size2d);
    dct->fSrcDCT = (float *)fftwf_malloc(sizeof(float) * size2d);

    if (bitsPerSample == 8)
        dct->Float2Pixels = Float2Pixels_C<uint8_t>;
    else
        dct->Float2Pixels = Float2Pixels_C<uint16_t>;

    if (opt) {
#if defined(MVTOOLS_X86)
        if (bitsPerSample == 8)
            dct->Float2Pixels = Float2Pixels_SSE2<uint8_t>;
        else
            dct->Float2Pixels = Float2Pixels_SSE2<uint16_t>;
#endif
    }

    {
        std::lock_guard<std::mutex> guard(g_fftw_plans_mutex);
        dct->dctplan = fftwf_plan_r2r_2d(sizey, sizex, dct->fSrc, dct->fSrcDCT,
                                         FFTW_REDFT10, FFTW_REDFT10, FFTW_ESTIMATE); // direct fft
    }
}


void dctDeinit(DCTFFTW *dct) {
    {
        std::lock_guard<std::mutex> guard(g_fftw_plans_mutex);
        fftwf_destroy_plan(dct->dctplan);
    }
    fftwf_free(dct->fSrc);
    fftwf_free(dct->fSrcDCT);
}


//  put source data to real array for FFT
template <typename PixelType>
static void Pixels2Float(const DCTFFTW *dct, const uint8_t *srcp8, int src_pitch, float *realdata) {
    for (int j = 0; j < dct->sizey; j++) {
        for (int i = 0; i < dct->sizex; i++) {
            PixelType *srcp = (PixelType *)srcp8;
            realdata[i] = srcp[i];
        }
        srcp8 += src_pitch;
        realdata += dct->sizex;
    }
}


void dctBytes2D(DCTFFTW *dct, const uint8_t *srcp, int src_pitch, uint8_t *dctp, int dct_pitch) {
    if (dct->bitsPerSample == 8) {
        Pixels2Float<uint8_t>(dct, srcp, src_pitch, dct->fSrc);
    } else {
        Pixels2Float<uint16_t>(dct, srcp, src_pitch, dct->fSrc);
    }
    fftwf_execute_r2r(dct->dctplan, dct->fSrc, dct->fSrcDCT);
    dct->Float2Pixels(dct, dctp, dct_pitch, dct->fSrcDCT);
}
