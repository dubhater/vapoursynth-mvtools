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

std::mutex g_fftw_plans_mutex;


void dctInit(DCTFFTW *dct, int sizex, int sizey, int dctmode, int bitsPerSample) {
    dct->sizex = sizex;
    dct->sizey = sizey;
    dct->dctmode = dctmode;
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
static void Bytes2Float(const DCTFFTW *dct, const uint8_t *srcp8, int src_pitch, float *realdata) {
    int floatpitch = dct->sizex;
    int i, j;
    for (j = 0; j < dct->sizey; j++) {
        for (i = 0; i < dct->sizex; i += 1) {
            PixelType *srcp = (PixelType *)srcp8;
            realdata[i] = srcp[i];
        }
        srcp8 += src_pitch;
        realdata += floatpitch;
    }
}

//  put source data to real array for FFT
template <typename PixelType>
static void Float2Bytes(const DCTFFTW *dct, uint8_t *dstp8, int dst_pitch, float *realdata) {
    PixelType *dstp = (PixelType *)dstp8;

    dst_pitch /= sizeof(PixelType);

    int pixelMax = (1 << dct->bitsPerSample) - 1;
    int pixelHalf = 1 << (dct->bitsPerSample - 1);

    int floatpitch = dct->sizex;
    int i, j;
    int integ;
    float f = realdata[0] * 0.5f; // to be compatible with integer DCTINT8
    /*
       _asm fld f;
       _asm fistp integ;
       */
    // XXX function call to nearbyintf can be avoided by using cvtss2si
    integ = (int)(nearbyintf(f));
    dstp[0] = std::min(pixelMax, std::max(0, (integ >> dct->dctshift0) + pixelHalf)); // DC
    for (i = 1; i < dct->sizex; i += 1) {
        f = realdata[i] * 0.707f; // to be compatible with integer DCTINT8
        /*
           _asm fld f;
           _asm fistp integ;
           */
        integ = (int)(nearbyintf(f));
        dstp[i] = std::min(pixelMax, std::max(0, (integ >> dct->dctshift) + pixelHalf));
    }
    dstp += dst_pitch;
    realdata += floatpitch;
    for (j = 1; j < dct->sizey; j++) {
        for (i = 0; i < dct->sizex; i += 1) {
            f = realdata[i] * 0.707f; // to be compatible with integer DCTINT8
            /*
               _asm fld f;
               _asm fistp integ;
               */
            integ = (int)(nearbyintf(f));
            dstp[i] = std::min(pixelMax, std::max(0, (integ >> dct->dctshift) + pixelHalf));
        }
        dstp += dst_pitch;
        realdata += floatpitch;
    }
}


void dctBytes2D(DCTFFTW *dct, const uint8_t *srcp, int src_pitch, uint8_t *dctp, int dct_pitch) {
    if (dct->bitsPerSample == 8) {
        Bytes2Float<uint8_t>(dct, srcp, src_pitch, dct->fSrc);
        fftwf_execute_r2r(dct->dctplan, dct->fSrc, dct->fSrcDCT);
        Float2Bytes<uint8_t>(dct, dctp, dct_pitch, dct->fSrcDCT);
    } else {
        Bytes2Float<uint16_t>(dct, srcp, src_pitch, dct->fSrc);
        fftwf_execute_r2r(dct->dctplan, dct->fSrc, dct->fSrcDCT);
        Float2Bytes<uint16_t>(dct, dctp, dct_pitch, dct->fSrcDCT);
    }
}
