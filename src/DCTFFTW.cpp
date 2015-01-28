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


DCTFFTW::DCTFFTW(int _sizex, int _sizey, int _dctmode, int _bitsPerSample) :
    DCTClass(_sizex , _sizey, _dctmode, _bitsPerSample)
{
    int size2d = sizey*sizex;

    int cursize = 1;
    dctshift = 0;
    while (cursize < size2d)
    {
        dctshift++;
        cursize = (cursize<<1);
    }

    dctshift0 = dctshift + 2;

    fSrc = (float *)fftwf_malloc(sizeof(float) * size2d );
    fSrcDCT = (float *)fftwf_malloc(sizeof(float) * size2d );

    g_fftw_plans_mutex.lock();
    dctplan = fftwf_plan_r2r_2d(sizey, sizex, fSrc, fSrcDCT,
            FFTW_REDFT10, FFTW_REDFT10, FFTW_ESTIMATE); // direct fft
    g_fftw_plans_mutex.unlock();
}


DCTFFTW::~DCTFFTW()
{
    fftwf_destroy_plan(dctplan);
    fftwf_free(fSrc);
    fftwf_free(fSrcDCT);

}

//  put source data to real array for FFT
template <typename PixelType>
void DCTFFTW::Bytes2Float (const unsigned char * srcp8, int src_pitch, float * realdata)
{
    int floatpitch = sizex;
    int i, j;
    for (j = 0; j < sizey; j++) {
        for (i = 0; i < sizex; i+=1) {
            PixelType *srcp = (PixelType *)srcp8;
            realdata[i] = srcp[i];
        }
        srcp8 += src_pitch;
        realdata += floatpitch;
    }
}

//  put source data to real array for FFT
template <typename PixelType>
void DCTFFTW::Float2Bytes (unsigned char * dstp8, int dst_pitch, float * realdata)
{
    PixelType *dstp = (PixelType *)dstp8;

    dst_pitch /= sizeof(PixelType);

    int pixelMax = (1 << bitsPerSample) - 1;
    int pixelHalf = 1 << (bitsPerSample - 1);

    int floatpitch = sizex;
    int i, j;
    int integ;
    float f = realdata[0]*0.5f; // to be compatible with integer DCTINT8
    /*
       _asm fld f;
       _asm fistp integ;
       */
    // XXX function call to nearbyintf can be avoided by using cvtss2si
    integ = (int)(nearbyintf(f));
    dstp[0] = std::min(pixelMax, std::max(0, (integ>>dctshift0) + pixelHalf)); // DC
    for (i = 1; i < sizex; i+=1) {
        f = realdata[i]*0.707f; // to be compatible with integer DCTINT8
        /*
           _asm fld f;
           _asm fistp integ;
           */
        integ = (int)(nearbyintf(f));
        dstp[i] = std::min(pixelMax, std::max(0, (integ>>dctshift) + pixelHalf));
    }
    dstp += dst_pitch;
    realdata += floatpitch;
    for (j = 1; j < sizey; j++) {
        for (i = 0; i < sizex; i+=1) {
            f = realdata[i]*0.707f; // to be compatible with integer DCTINT8
            /*
               _asm fld f;
               _asm fistp integ;
               */
            integ = (int)(nearbyintf(f));
            dstp[i] = std::min(pixelMax, std::max(0, (integ>>dctshift) + pixelHalf));
        }
        dstp += dst_pitch;
        realdata += floatpitch;
    }
}


void DCTFFTW::DCTBytes2D(const unsigned char *srcp, int src_pitch, unsigned char *dctp, int dct_pitch)
{
    if (bitsPerSample == 8) {
        Bytes2Float<uint8_t>(srcp, src_pitch, fSrc);
        fftwf_execute_r2r(dctplan, fSrc, fSrcDCT);
        Float2Bytes<uint8_t>(dctp, dct_pitch, fSrcDCT);
    } else {
        Bytes2Float<uint16_t>(srcp, src_pitch, fSrc);
        fftwf_execute_r2r(dctplan, fSrc, fSrcDCT);
        Float2Bytes<uint16_t>(dctp, dct_pitch, fSrcDCT);
    }
}
