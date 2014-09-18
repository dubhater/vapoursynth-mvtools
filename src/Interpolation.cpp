// Functions that interpolates a frame
// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - bicubic, Wiener, separable

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

#include "Interpolation.h"

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif


void RB2F_C(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight)
{
	for ( int y = 0; y < nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
}
/*
void RB2Filtered_C(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight)
{ // sort of Reduceby2 with 1/4, 1/2, 1/4 filter for smoothing - Fizick v.1.10.3

		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;

	for ( int y = 1; y < nHeight; y++ )
	{
        int x = 0;
            pDst[x] = (2*pSrc[x*2-nSrcPitch] + 2*pSrc[x*2-nSrcPitch+1] +
                        4*pSrc[x*2] + 4*pSrc[x*2+1] +
                        2*pSrc[x*2+nSrcPitch+1] + 2*pSrc[x*2+nSrcPitch] + 8) / 16;

		for ( x = 1; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2-nSrcPitch-1] + pSrc[x*2-nSrcPitch]*2 + pSrc[x*2-nSrcPitch+1] +
                       pSrc[x*2-1]*2 + pSrc[x*2]*4 + pSrc[x*2+1]*2 +
                       pSrc[x*2+nSrcPitch-1] + pSrc[x*2+nSrcPitch]*2 + pSrc[x*2+nSrcPitch+1] + 8) / 16;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
}

void RB2BilinearFiltered_C(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight)
{ // filtered bilinear with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick v.2.3.1

	for ( int y = 0; y < 1; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

	for ( int y = 1; y < nHeight-1; y++ )
	{
		for ( int x = 0; x < 1; x++ )
           pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;

		for ( x = 1; x < nWidth-1; x++ )
            pDst[x] = (unsigned int)(pSrc[x*2-nSrcPitch-1] + pSrc[x*2-nSrcPitch]*3 + pSrc[x*2-nSrcPitch+1]*3 + pSrc[x*2-nSrcPitch+2] +
                       pSrc[x*2-1]*3 + pSrc[x*2]*9 + pSrc[x*2+1]*9 + pSrc[x*2+2]*3 +
                       pSrc[x*2+nSrcPitch-1]*3 + pSrc[x*2+nSrcPitch]*9 + pSrc[x*2+nSrcPitch+1]*9 + pSrc[x*2+nSrcPitch+2]*3 +
                       pSrc[x*2+nSrcPitch*2-1] + pSrc[x*2+nSrcPitch*2]*3 + pSrc[x*2+nSrcPitch*2+1]*3 + pSrc[x*2+nSrcPitch*2+2] + 32) / 64;

		for ( int x = max(nWidth-1,1); x < nWidth; x++ )
           pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;

		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
	for ( int y = max(nHeight-1,1); y < nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

}

void RB2Quadratic_C(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight)
{ // filtered quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick v.2.3.1

	for ( int y = 0; y < 1; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

	for ( int y = 1; y < nHeight-1; y++ )
	{
		for ( int x = 0; x < 1; x++ )
           pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;

		for ( int x = 1; x < nWidth-1; x++ )
            pDst[x] =
            (pSrc[x*2-nSrcPitch*2-2] + pSrc[x*2-nSrcPitch*2-1]*9 + pSrc[x*2-nSrcPitch*2]*22 + pSrc[x*2-nSrcPitch*2+1]*22 + pSrc[x*2-nSrcPitch*2+2]*9 + pSrc[x*2-nSrcPitch*2+3] +
            pSrc[x*2-nSrcPitch-2]*9 + pSrc[x*2-nSrcPitch-1]*81 + pSrc[x*2-nSrcPitch]*198 + pSrc[x*2-nSrcPitch+1]*198 + pSrc[x*2-nSrcPitch+2]*81 + pSrc[x*2-nSrcPitch+3]*9 +
            pSrc[x*2-2]*22 + pSrc[x*2-1]*198 + pSrc[x*2]*484 + pSrc[x*2+1]*484 + pSrc[x*2+2]*198 + pSrc[x*2+3]*22 +
            pSrc[x*2+nSrcPitch-2]*22 + pSrc[x*2+nSrcPitch-1]*198 + pSrc[x*2+nSrcPitch]*484 + pSrc[x*2+nSrcPitch+1]*484 + pSrc[x*2+nSrcPitch+2]*198 + pSrc[x*2+nSrcPitch+3]*22 +
            pSrc[x*2+nSrcPitch*2-2]*9 + pSrc[x*2+nSrcPitch*2-1]*81 + pSrc[x*2+nSrcPitch*2]*198 + pSrc[x*2+nSrcPitch*2+1]*198 + pSrc[x*2+nSrcPitch*2+2]*81 + pSrc[x*2+nSrcPitch*2+3]*9 +
            pSrc[x*2+nSrcPitch*3-2] + pSrc[x*2+nSrcPitch*3-1]*9 + pSrc[x*2+nSrcPitch*3]*22 + pSrc[x*2+nSrcPitch*3+1]*22 + pSrc[x*2+nSrcPitch*3+2]*9 + pSrc[x*2+nSrcPitch*3+3] + 2048) /4096;

		for ( int x = max(nWidth-1,1); x < nWidth; x++ )
           pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;

		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
	for ( int y = max(nHeight-1,1); y < nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

}

void RB2Cubic_C(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight)
{ // filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick v.2.3.1

	for ( int y = 0; y < 1; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

	for ( int y = 1; y < nHeight-1; y++ )
	{
		for ( int x = 0; x < 1; x++ )
           pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;

		for ( int x = 1; x < nWidth-1; x++ )
            pDst[x] =
            (pSrc[x*2-nSrcPitch*2-2] + pSrc[x*2-nSrcPitch*2-1]*5 + pSrc[x*2-nSrcPitch*2]*10 + pSrc[x*2-nSrcPitch*2+1]*10 + pSrc[x*2-nSrcPitch*2+2]*5 + pSrc[x*2-nSrcPitch*2+3] +
            pSrc[x*2-nSrcPitch-2]*5 + pSrc[x*2-nSrcPitch-1]*25 + pSrc[x*2-nSrcPitch]*50 + pSrc[x*2-nSrcPitch+1]*50 + pSrc[x*2-nSrcPitch+2]*25 + pSrc[x*2-nSrcPitch+3]*5 +
            pSrc[x*2-2]*10 + pSrc[x*2-1]*50 + pSrc[x*2]*100 + pSrc[x*2+1]*100 + pSrc[x*2+2]*50 + pSrc[x*2+3]*10 +
            pSrc[x*2+nSrcPitch-2]*10 + pSrc[x*2+nSrcPitch-1]*50 + pSrc[x*2+nSrcPitch]*100 + pSrc[x*2+nSrcPitch+1]*100 + pSrc[x*2+nSrcPitch+2]*50 + pSrc[x*2+nSrcPitch+3]*10 +
            pSrc[x*2+nSrcPitch*2-2]*5 + pSrc[x*2+nSrcPitch*2-1]*25 + pSrc[x*2+nSrcPitch*2]*50 + pSrc[x*2+nSrcPitch*2+1]*50 + pSrc[x*2+nSrcPitch*2+2]*25 + pSrc[x*2+nSrcPitch*2+3]*5 +
            pSrc[x*2+nSrcPitch*3-2] + pSrc[x*2+nSrcPitch*3-1]*5 + pSrc[x*2+nSrcPitch*3]*10 + pSrc[x*2+nSrcPitch*3+1]*10 + pSrc[x*2+nSrcPitch*3+2]*5 + pSrc[x*2+nSrcPitch*3+3] + 512) /1024;

		for ( int x = max(nWidth-1,1); x < nWidth; x++ )
           pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;

		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
	for ( int y = max(nHeight-1,1); y < nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x*2] + pSrc[x*2+1] + pSrc[x*2+nSrcPitch+1] + pSrc[x*2+nSrcPitch] + 2) / 4;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

}
*/
void RB2FilteredVertical(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ //  Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
            // nHeight is dst height which is reduced by 2 source height
    //int nWidthMMX = (nWidth/4)*4;

	for ( int y = 0; y < 1; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x] + pSrc[x+nSrcPitch] + 1) / 2;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

    /* TODO: port the asm
   if (isse && nWidthMMX>=4)
    {
        for ( int y = 1; y < nHeight; y++ )
        {
                RB2FilteredVerticalLine_SSE(pDst, pSrc, nSrcPitch, nWidthMMX);

             for ( int x = nWidthMMX; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch] + pSrc[x]*2 + pSrc[x+nSrcPitch] + 2) /4;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
    else
    */
    {
        for ( int y = 1; y < nHeight; y++ )
        {
            for ( int x = 0; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch] + pSrc[x]*2 + pSrc[x+nSrcPitch] + 2) /4;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
}

void RB2FilteredHorizontalInplace(unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick
            // nWidth is dst height which is reduced by 2 source width
    //int nWidthMMX = 1 + ((nWidth-2)/4)*4;

	for ( int y = 0; y < nHeight; y++ )
	{
		int x = 0;
           int pSrc0 = (pSrc[x*2] + pSrc[x*2+1] + 1) / 2;

           /* TODO: port the asm
        if (isse)
        {
            RB2FilteredHorizontalInplaceLine_SSE(pSrc, nWidthMMX); // very first is skipped
            for ( int x = nWidthMMX; x < nWidth; x++ )
                pSrc[x] = (pSrc[x*2-1] + pSrc[x*2]*2 + pSrc[x*2+1] + 2) /4;
        }
        else
        */
        {
            for ( int x = 1; x < nWidth; x++ )
                pSrc[x] = (pSrc[x*2-1] + pSrc[x*2]*2 + pSrc[x*2+1] + 2) /4;
        }
        pSrc[0] = pSrc0;

		pSrc += nSrcPitch;
	}
}

void RB2Filtered(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // separable Filtered with 1/4, 1/2, 1/4 filter for smoothing and anti-aliasing - Fizick v.2.5.2
 // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2FilteredVertical(pDst, pSrc, nDstPitch, nSrcPitch, nWidth*2, nHeight, isse); // intermediate half height
    RB2FilteredHorizontalInplace(pDst, nDstPitch, nWidth, nHeight, isse); // inpace width reduction
}


void RB2BilinearFilteredVertical(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ //  BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
            // nHeight is dst height which is reduced by 2 source height
    int nWidthMMX = (nWidth/8)*8;

	for ( int y = 0; y < 1  && y<nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x] + pSrc[x+nSrcPitch] + 1) / 2;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

    if (isse && nWidthMMX>=8)
    {
        for ( int y = 1; y < nHeight-1; y++ )
        {
                mvtools_RB2BilinearFilteredVerticalLine_sse2(pDst, pSrc, nSrcPitch, nWidthMMX);

             for ( int x = nWidthMMX; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch] + pSrc[x]*3 + pSrc[x+nSrcPitch]*3 + pSrc[x+nSrcPitch*2] + 4) /8;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
    else
    {
        for ( int y = 1; y < nHeight-1; y++ )
        {
            for ( int x = 0; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch] + pSrc[x]*3 + pSrc[x+nSrcPitch]*3 + pSrc[x+nSrcPitch*2] + 4) /8;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
	for ( int y = max(nHeight-1,1); y < nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x] + pSrc[x+nSrcPitch] + 1) / 2;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
}

void RB2BilinearFilteredHorizontalInplace(unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick
            // nWidth is dst height which is reduced by 2 source width
    int nWidthMMX = 1 + ((nWidth-2)/8)*8;

	for ( int y = 0; y < nHeight; y++ )
	{
		int x = 0;
           int pSrc0 = (pSrc[x*2] + pSrc[x*2+1] + 1) / 2;

        if (isse)
        {
            mvtools_RB2BilinearFilteredHorizontalInplaceLine_sse2(pSrc, nWidthMMX); // very first is skipped
            for ( int x = nWidthMMX; x < nWidth-1; x++ )
                pSrc[x] = (pSrc[x*2-1] + pSrc[x*2]*3 + pSrc[x*2+1]*3 + pSrc[x*2+2] + 4) /8;
        }
        else
        {
            for ( int x = 1; x < nWidth-1; x++ )
                pSrc[x] = (pSrc[x*2-1] + pSrc[x*2]*3 + pSrc[x*2+1]*3 + pSrc[x*2+2] + 4) /8;
        }
        pSrc[0] = pSrc0;

		for ( int x = max(nWidth-1,1); x < nWidth; x++ )
           pSrc[x] = (pSrc[x*2] + pSrc[x*2+1] + 1) / 2;

		pSrc += nSrcPitch;
	}
}

void RB2BilinearFiltered(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // separable BilinearFiltered with 1/8, 3/8, 3/8, 1/8 filter for smoothing and anti-aliasing - Fizick v.2.5.2
 // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2BilinearFilteredVertical(pDst, pSrc, nDstPitch, nSrcPitch, nWidth*2, nHeight, isse); // intermediate half height
    RB2BilinearFilteredHorizontalInplace(pDst, nDstPitch, nWidth, nHeight, isse); // inpace width reduction
}

void RB2QuadraticVertical(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
            // nHeight is dst height which is reduced by 2 source height
    int nWidthMMX = (nWidth/8)*8;

	for ( int y = 0; y < 1  && y<nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x] + pSrc[x+nSrcPitch] + 1) / 2;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

    if (isse && nWidthMMX>=8)
    {
        for ( int y = 1; y < nHeight-1; y++ )
        {
                mvtools_RB2QuadraticVerticalLine_sse2(pDst, pSrc, nSrcPitch, nWidthMMX);

             for ( int x = nWidthMMX; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch*2] + pSrc[x-nSrcPitch]*9 + pSrc[x]*22 +
                 pSrc[x+nSrcPitch]*22 + pSrc[x+nSrcPitch*2]*9 + pSrc[x+nSrcPitch*3] + 32) /64;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
    else
    {

        for ( int y = 1; y < nHeight-1; y++ )
        {
            for ( int x = 0; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch*2] + pSrc[x-nSrcPitch]*9 + pSrc[x]*22 +
                 pSrc[x+nSrcPitch]*22 + pSrc[x+nSrcPitch*2]*9 + pSrc[x+nSrcPitch*3] + 32) /64;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
	for ( int y = max(nHeight-1,1); y < nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x] + pSrc[x+nSrcPitch] + 1) / 2;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
}

void RB2QuadraticHorizontalInplace(unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, int isse)
{ // filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick
            // nWidth is dst height which is reduced by 2 source width
    int nWidthMMX = 1 + ((nWidth-2)/8)*8;

	for ( int y = 0; y < nHeight; y++ )
	{
		int x = 0;
           int pSrc0 = (pSrc[x*2] + pSrc[x*2+1] + 1) / 2; // store temporary

        if (isse)
        {
            mvtools_RB2QuadraticHorizontalInplaceLine_sse2(pSrc, nWidthMMX);
            for ( int x = nWidthMMX; x < nWidth-1; x++ )
                pSrc[x] = (pSrc[x*2-2] + pSrc[x*2-1]*9 + pSrc[x*2]*22 + pSrc[x*2+1]*22 + pSrc[x*2+2]*9 + pSrc[x*2+3] + 32) /64;
        }
        else
        {
		for ( int x = 1; x < nWidth-1; x++ )
            pSrc[x] = (pSrc[x*2-2] + pSrc[x*2-1]*9 + pSrc[x*2]*22 + pSrc[x*2+1]*22 + pSrc[x*2+2]*9 + pSrc[x*2+3] + 32) /64;
        }
        pSrc[0] = pSrc0;

		for ( int x = max(nWidth-1,1); x < nWidth; x++ )
           pSrc[x] = (pSrc[x*2] + pSrc[x*2+1] + 1) / 2;

		pSrc += nSrcPitch;
	}
}

void RB2Quadratic(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // separable filtered Quadratic with 1/64, 9/64, 22/64, 22/64, 9/64, 1/64 filter for smoothing and anti-aliasing - Fizick v.2.5.2
 // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2QuadraticVertical(pDst, pSrc, nDstPitch, nSrcPitch, nWidth*2, nHeight, isse); // intermediate half height
    RB2QuadraticHorizontalInplace(pDst, nDstPitch, nWidth, nHeight, isse); // inpace width reduction
}


void RB2CubicVertical(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
            // nHeight is dst height which is reduced by 2 source height
    int nWidthMMX = (nWidth/8)*8;
	for ( int y = 0; y < 1  && y<nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x] + pSrc[x+nSrcPitch] + 1) / 2;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}

    if (isse && nWidthMMX>=8)
    {
        for ( int y = 1; y < nHeight-1; y++ )
        {
                mvtools_RB2CubicVerticalLine_sse2(pDst, pSrc, nSrcPitch, nWidthMMX);

             for ( int x = nWidthMMX; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch*2] + pSrc[x-nSrcPitch]*5 + pSrc[x]*10 +
                 pSrc[x+nSrcPitch]*10 + pSrc[x+nSrcPitch*2]*5 + pSrc[x+nSrcPitch*3] + 16) /32;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
    }
    else
    {
        for ( int y = 1; y < nHeight-1; y++ )
        {
            for ( int x = 0; x < nWidth; x++ )
                pDst[x] = (pSrc[x-nSrcPitch*2] + pSrc[x-nSrcPitch]*5 + pSrc[x]*10 +
                 pSrc[x+nSrcPitch]*10 + pSrc[x+nSrcPitch*2]*5 + pSrc[x+nSrcPitch*3] + 16) /32;

            pDst += nDstPitch;
            pSrc += nSrcPitch * 2;
        }
     }
	for ( int y = max(nHeight-1,1); y < nHeight; y++ )
	{
		for ( int x = 0; x < nWidth; x++ )
            pDst[x] = (pSrc[x] + pSrc[x+nSrcPitch] + 1) / 2;
		pDst += nDstPitch;
		pSrc += nSrcPitch * 2;
	}
}

void RB2CubicHorizontalInplace(unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // filtered qubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick
            // nWidth is dst height which is reduced by 2 source width
    int nWidthMMX = 1 + ((nWidth-2)/8)*8;
	for ( int y = 0; y < nHeight; y++ )
	{
		int x = 0;
           int pSrcw0 = (pSrc[x*2] + pSrc[x*2+1] + 1) / 2; // store temporary
        if (isse)
        {
            mvtools_RB2CubicHorizontalInplaceLine_sse2(pSrc, nWidthMMX);
            for ( int x = nWidthMMX; x < nWidth-1; x++ )
                pSrc[x] = (pSrc[x*2-2] + pSrc[x*2-1]*5 + pSrc[x*2]*10 + pSrc[x*2+1]*10 + pSrc[x*2+2]*5 + pSrc[x*2+3] + 16) /32;
        }
        else
        {
		for ( int x = 1; x < nWidth-1; x++ )
            pSrc[x] = (pSrc[x*2-2] + pSrc[x*2-1]*5 + pSrc[x*2]*10 + pSrc[x*2+1]*10 + pSrc[x*2+2]*5 + pSrc[x*2+3] + 16) /32;
        }
        pSrc[0] = pSrcw0;

		for ( int x = max(nWidth-1,1); x < nWidth; x++ )
           pSrc[x] = (pSrc[x*2] + pSrc[x*2+1] + 1) / 2;

		pSrc += nSrcPitch;
	}
}

void RB2Cubic(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
            int nSrcPitch, int nWidth, int nHeight, bool isse)
{ // separable filtered cubic with 1/32, 5/32, 10/32, 10/32, 5/32, 1/32 filter for smoothing and anti-aliasing - Fizick v.2.5.2
 // assume he have enough horizontal dimension for intermediate results (double as final)
    RB2CubicVertical(pDst, pSrc, nDstPitch, nSrcPitch, nWidth*2, nHeight, isse); // intermediate half height
    RB2CubicHorizontalInplace(pDst, nDstPitch, nWidth, nHeight, isse); // inpace width reduction
}


void VerticalBilinear(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                   int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < nHeight - 1; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    // last row
    for ( int i = 0; i < nWidth; i++ )
        pDst[i] = pSrc[i];
}

void HorizontalBilinear(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                     int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < nHeight; j++ )
    {
        for ( int i = 0; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}

void DiagonalBilinear(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                   int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < nHeight - 1; j++ )
    {
        for ( int i = 0; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int i = 0; i < nWidth - 1; i++ )
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}

// so called Wiener interpolation. (sharp, similar to Lanczos ?)
// invarint simplified, 6 taps. Weights: (1, -5, 20, 20, -5, 1)/32 - added by Fizick
void VerticalWiener(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                   int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < 2; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = 2; j < nHeight - 4; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
		{
 			pDst[i] = min(255,max(0,
				( (pSrc[i-nSrcPitch*2])
				+ (-(pSrc[i-nSrcPitch]) + (pSrc[i]<<2) + (pSrc[i+nSrcPitch]<<2) - (pSrc[i+nSrcPitch*2]) )*5
				+ (pSrc[i+nSrcPitch*3]) + 16)>>5) );
		}
       pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = nHeight - 4; j < nHeight - 1; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
		{
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
		}

        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    // last row
    for ( int i = 0; i < nWidth; i++ )
        pDst[i] = pSrc[i];
}

void HorizontalWiener(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                     int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < nHeight; j++ )
    {
            pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
            pDst[1] = (pSrc[1] + pSrc[2] + 1) >> 1;
        for ( int i = 2; i < nWidth - 4; i++ )
		{
			pDst[i] = min(255,max(0,((pSrc[i-2]) + (-(pSrc[i-1]) + (pSrc[i]<<2)
				+ (pSrc[i+1]<<2) - (pSrc[i+2]))*5 + (pSrc[i+3]) + 16)>>5));
		}
        for ( int i = nWidth - 4; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}

void DiagonalWiener(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                   int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < 2; j++ )
    {
        for ( int i = 0; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = 2; j < nHeight - 4; j++ )
    {
        for ( int i = 0; i < 2; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;
        for ( int i = 2; i < nWidth - 4; i++ )
		{
 			pDst[i] = min(255,max(0,
				((pSrc[i-2-nSrcPitch*2]) + (-(pSrc[i-1-nSrcPitch]) + (pSrc[i]<<2)
			+ (pSrc[i+1+nSrcPitch]<<2) - (pSrc[i+2+nSrcPitch*2]<<2))*5 + (pSrc[i+3+nSrcPitch*3])
				+ (pSrc[i+3-nSrcPitch*2]) + (-(pSrc[i+2-nSrcPitch]) + (pSrc[i+1]<<2)
			+ (pSrc[i+nSrcPitch]<<2) - (pSrc[i-1+nSrcPitch*2]))*5 + (pSrc[i-2+nSrcPitch*3])
			+ 32)>>6));
		}
        for ( int i = nWidth - 4; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = nHeight - 4; j < nHeight - 1; j++ )
    {
        for ( int i = 0; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int i = 0; i < nWidth - 1; i++ )
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}

// bicubic (Catmull-Rom 4 taps interpolation)
void VerticalBicubic(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                   int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < 1; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = 1; j < nHeight - 3; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
		{
 			pDst[i] = min(255,max(0,
				( -pSrc[i-nSrcPitch] - pSrc[i+nSrcPitch*2] + (pSrc[i] + pSrc[i+nSrcPitch])*9 + 8)>>4) );
		}
       pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = nHeight - 3; j < nHeight - 1; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
		{
            pDst[i] = (pSrc[i] + pSrc[i + nSrcPitch] + 1) >> 1;
		}

        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    // last row
    for ( int i = 0; i < nWidth; i++ )
        pDst[i] = pSrc[i];
}

void HorizontalBicubic(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                     int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < nHeight; j++ )
    {
            pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        for ( int i = 1; i < nWidth - 3; i++ )
		{
			pDst[i] = min(255,max(0,
				( -(pSrc[i-1] + pSrc[i+2]) + (pSrc[i] + pSrc[i+1])*9 + 8)>>4));
		}
        for ( int i = nWidth - 3; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}

void DiagonalBicubic(unsigned char *pDst, const unsigned char *pSrc, int nDstPitch,
                   int nSrcPitch, int nWidth, int nHeight)
{
    for ( int j = 0; j < 1; j++ )
    {
        for ( int i = 0; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = 1; j < nHeight - 3; j++ )
    {
        for ( int i = 0; i < 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;
        for ( int i = 1; i < nWidth - 3; i++ )
		{
 			pDst[i] = min(255,max(0,
				( -pSrc[i-1-nSrcPitch] - pSrc[i+2+nSrcPitch*2] + (pSrc[i] + pSrc[i+1+nSrcPitch])*9
				- pSrc[i-1+nSrcPitch*2] - pSrc[i+2-nSrcPitch] + (pSrc[i+1] + pSrc[i+nSrcPitch])*9
			+ 16)>>5));
		}
        for ( int i = nWidth - 3; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int j = nHeight - 3; j < nHeight - 1; j++ )
    {
        for ( int i = 0; i < nWidth - 1; i++ )
            pDst[i] = (pSrc[i] + pSrc[i + 1] + pSrc[i + nSrcPitch] + pSrc[i + nSrcPitch + 1] + 2) >> 2;

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth + nSrcPitch - 1] + 1) >> 1;
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
    for ( int i = 0; i < nWidth - 1; i++ )
        pDst[i] = (pSrc[i] + pSrc[i + 1] + 1) >> 1;
    pDst[nWidth - 1] = pSrc[nWidth - 1];
}

void Average2(unsigned char *pDst, const unsigned char *pSrc1, const unsigned char *pSrc2,
                     int nPitch, int nWidth, int nHeight)
{ // assume all pitches equal
    for ( int j = 0; j < nHeight; j++ )
    {
        for ( int i = 0; i < nWidth; i++ )
            pDst[i] = (pSrc1[i] + pSrc2[i] + 1) >> 1;

        pDst += nPitch;
        pSrc1 += nPitch;
        pSrc2 += nPitch;
    }
}


