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

#include <VSHelper.h>

#include "MVInterface.h"
#include "Padding.h"
#include "Interpolation.h"
#include "MVSuper.h"

/******************************************************************************
 *                                                                             *
 *  MVPlane : manages a single plane, allowing padding and refinin             *
 *                                                                             *
 ******************************************************************************/

MVPlane::MVPlane(int _nWidth, int _nHeight, int _nPel, int _nHPad, int _nVPad, bool _isse, int _bitsPerSample)
{
    nWidth = _nWidth;
    nHeight = _nHeight;
    nPel = _nPel;
    nHPadding = _nHPad;
    nVPadding = _nVPad;
    isse = _isse;
    nHPaddingPel = _nHPad * nPel;
    nVPaddingPel = _nVPad * nPel;
    bitsPerSample = _bitsPerSample;
    bytesPerSample = (bitsPerSample + 7) / 8; // Who would ever want to process 32 bit video?

    nExtendedWidth = nWidth + 2 * nHPadding;
    nExtendedHeight = nHeight + 2 * nVPadding;
    pPlane = new uint8_t *[nPel * nPel];
    //   InitializeCriticalSection(&cs);
    }

MVPlane::~MVPlane()
{
    delete[] pPlane;

    //    DeleteCriticalSection(&cs);
}

void MVPlane::Update(uint8_t* pSrc, int _nPitch) //v2.0
{
    //    EnterCriticalSection(&cs);
    nPitch = _nPitch;
    nOffsetPadding = nPitch * nVPadding + nHPadding * bytesPerSample;

    for ( int i = 0; i < nPel * nPel; i++ )
        pPlane[i] = pSrc + i*nPitch * nExtendedHeight;

    ResetState();
    //   LeaveCriticalSection(&cs);
}
void MVPlane::ChangePlane(const uint8_t *pNewPlane, int nNewPitch)
{
    //    EnterCriticalSection(&cs);
    if ( !isFilled )
        vs_bitblt(pPlane[0] + nOffsetPadding, nPitch, pNewPlane, nNewPitch, nWidth * bytesPerSample, nHeight);
    isFilled = true;
    //   LeaveCriticalSection(&cs);
}

void MVPlane::Pad()
{
    //    EnterCriticalSection(&cs);
    if ( !isPadded ) {
        if (bytesPerSample == 1)
            PadReferenceFrame<uint8_t>(pPlane[0], nPitch, nHPadding, nVPadding, nWidth, nHeight);
        else
            PadReferenceFrame<uint16_t>(pPlane[0], nPitch, nHPadding, nVPadding, nWidth, nHeight);
    }

    isPadded = true;
    //   LeaveCriticalSection(&cs);
}

void MVPlane::Refine(int sharp)
{
    //    EnterCriticalSection(&cs);
    if (( nPel == 2 ) && ( !isRefined ))
    {
        if (sharp == 0) // bilinear
        {
            if (isse)
            {
                mvtools_HorizontalBilinear_sse2(pPlane[1], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_VerticalBilinear_sse2(pPlane[2], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_DiagonalBilinear_sse2(pPlane[3], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
            }
            else
            {
                if (bytesPerSample == 1) {
                    HorizontalBilinear<uint8_t>(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    VerticalBilinear<uint8_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    DiagonalBilinear<uint8_t>(pPlane[3], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                } else {
                    HorizontalBilinear<uint16_t>(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    VerticalBilinear<uint16_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    DiagonalBilinear<uint16_t>(pPlane[3], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                }
            }
        }
        else if(sharp==1) // bicubic
        {
            /* TODO: port the asm
               if (isse)
               {
               HorizontalBicubic_iSSE(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
               VerticalBicubic_iSSE(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
               HorizontalBicubic_iSSE(pPlane[3], pPlane[2], nPitch, nPitch, nExtendedWidth, nExtendedHeight); // faster from ready-made horizontal
               }
               else
               */
            {
                if (bytesPerSample == 1) {
                    HorizontalBicubic<uint8_t>(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalBicubic<uint8_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalBicubic<uint8_t>(pPlane[3], pPlane[2], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                } else {
                    HorizontalBicubic<uint16_t>(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalBicubic<uint16_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalBicubic<uint16_t>(pPlane[3], pPlane[2], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                }
            }
        }
        else // Wiener
        {
            if (isse)
            {
                mvtools_HorizontalWiener_sse2(pPlane[1], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_VerticalWiener_sse2(pPlane[2], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_HorizontalWiener_sse2(pPlane[3], pPlane[2], nPitch, nExtendedWidth, nExtendedHeight);// faster from ready-made horizontal
            }
            else
            {
                if (bytesPerSample == 1) {
                    HorizontalWiener<uint8_t>(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalWiener<uint8_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalWiener<uint8_t>(pPlane[3], pPlane[2], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                } else {
                    HorizontalWiener<uint16_t>(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalWiener<uint16_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalWiener<uint16_t>(pPlane[3], pPlane[2], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                }
            }
        }
    }
    else if (( nPel == 4 ) && ( !isRefined )) // firstly pel2 interpolation
    {
        if (sharp == 0) // bilinear
        {
            if (isse)
            {
                mvtools_HorizontalBilinear_sse2(pPlane[2], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_VerticalBilinear_sse2(pPlane[8], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_DiagonalBilinear_sse2(pPlane[10], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
            }
            else
            {
                if (bytesPerSample == 1) {
                    HorizontalBilinear<uint8_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    VerticalBilinear<uint8_t>(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    DiagonalBilinear<uint8_t>(pPlane[10], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                } else {
                    HorizontalBilinear<uint16_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    VerticalBilinear<uint16_t>(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                    DiagonalBilinear<uint16_t>(pPlane[10], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
                }
            }
        }
        else if(sharp==1) // bicubic
        {
            /* TODO: port the asm
               if (isse)
               {
               HorizontalBicubic_iSSE(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
               VerticalBicubic_iSSE(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
               HorizontalBicubic_iSSE(pPlane[10], pPlane[8], nPitch, nPitch, nExtendedWidth, nExtendedHeight); // faster from ready-made horizontal
               }
               else
               */
            {
                if (bytesPerSample == 1) {
                    HorizontalBicubic<uint8_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalBicubic<uint8_t>(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalBicubic<uint8_t>(pPlane[10], pPlane[8], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                } else {
                    HorizontalBicubic<uint16_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalBicubic<uint16_t>(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalBicubic<uint16_t>(pPlane[10], pPlane[8], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                }
            }
        }
        else // Wiener
        {
            if (isse)
            {
                mvtools_HorizontalWiener_sse2(pPlane[2], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_VerticalWiener_sse2(pPlane[8], pPlane[0], nPitch, nExtendedWidth, nExtendedHeight);
                mvtools_HorizontalWiener_sse2(pPlane[10], pPlane[8], nPitch, nExtendedWidth, nExtendedHeight);// faster from ready-made horizontal
            }
            else
            {
                if (bytesPerSample == 1) {
                    HorizontalWiener<uint8_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalWiener<uint8_t>(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalWiener<uint8_t>(pPlane[10], pPlane[8], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                } else {
                    HorizontalWiener<uint16_t>(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    VerticalWiener<uint16_t>(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample);
                    HorizontalWiener<uint16_t>(pPlane[10], pPlane[8], nPitch, nPitch, nExtendedWidth, nExtendedHeight, bitsPerSample); // faster from ready-made horizontal
                }
            }
        }
        // now interpolate intermediate
        if (isse) {
            mvtools_Average2_sse2(pPlane[1], pPlane[0], pPlane[2], nPitch, nExtendedWidth, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[9], pPlane[8], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[4], pPlane[0], pPlane[8], nPitch, nExtendedWidth, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[6], pPlane[2], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[5], pPlane[4], pPlane[6], nPitch, nExtendedWidth, nExtendedHeight);

            mvtools_Average2_sse2(pPlane[3], pPlane[0] + 1, pPlane[2], nPitch, nExtendedWidth-1, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[11], pPlane[8] + 1, pPlane[10], nPitch, nExtendedWidth-1, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[12], pPlane[0] + nPitch, pPlane[8], nPitch, nExtendedWidth, nExtendedHeight-1);
            mvtools_Average2_sse2(pPlane[14], pPlane[2] + nPitch, pPlane[10], nPitch, nExtendedWidth, nExtendedHeight-1);
            mvtools_Average2_sse2(pPlane[13], pPlane[12], pPlane[14], nPitch, nExtendedWidth, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[7], pPlane[4] + 1, pPlane[6], nPitch, nExtendedWidth-1, nExtendedHeight);
            mvtools_Average2_sse2(pPlane[15], pPlane[12] + 1, pPlane[14], nPitch, nExtendedWidth-1, nExtendedHeight);
        } else {
            if (bytesPerSample == 1) {
                Average2<uint8_t>(pPlane[1], pPlane[0], pPlane[2], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint8_t>(pPlane[9], pPlane[8], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint8_t>(pPlane[4], pPlane[0], pPlane[8], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint8_t>(pPlane[6], pPlane[2], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint8_t>(pPlane[5], pPlane[4], pPlane[6], nPitch, nExtendedWidth, nExtendedHeight);

                Average2<uint8_t>(pPlane[3], pPlane[0] + 1, pPlane[2], nPitch, nExtendedWidth-1, nExtendedHeight);
                Average2<uint8_t>(pPlane[11], pPlane[8] + 1, pPlane[10], nPitch, nExtendedWidth-1, nExtendedHeight);
                Average2<uint8_t>(pPlane[12], pPlane[0] + nPitch, pPlane[8], nPitch, nExtendedWidth, nExtendedHeight-1);
                Average2<uint8_t>(pPlane[14], pPlane[2] + nPitch, pPlane[10], nPitch, nExtendedWidth, nExtendedHeight-1);
                Average2<uint8_t>(pPlane[13], pPlane[12], pPlane[14], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint8_t>(pPlane[7], pPlane[4] + 1, pPlane[6], nPitch, nExtendedWidth-1, nExtendedHeight);
                Average2<uint8_t>(pPlane[15], pPlane[12] + 1, pPlane[14], nPitch, nExtendedWidth-1, nExtendedHeight);
            } else {
                Average2<uint16_t>(pPlane[1], pPlane[0], pPlane[2], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint16_t>(pPlane[9], pPlane[8], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint16_t>(pPlane[4], pPlane[0], pPlane[8], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint16_t>(pPlane[6], pPlane[2], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint16_t>(pPlane[5], pPlane[4], pPlane[6], nPitch, nExtendedWidth, nExtendedHeight);

                Average2<uint16_t>(pPlane[3], pPlane[0] + bytesPerSample, pPlane[2], nPitch, nExtendedWidth - 1, nExtendedHeight);
                Average2<uint16_t>(pPlane[11], pPlane[8] + bytesPerSample, pPlane[10], nPitch, nExtendedWidth - 1, nExtendedHeight);
                Average2<uint16_t>(pPlane[12], pPlane[0] + nPitch, pPlane[8], nPitch, nExtendedWidth, nExtendedHeight - 1);
                Average2<uint16_t>(pPlane[14], pPlane[2] + nPitch, pPlane[10], nPitch, nExtendedWidth, nExtendedHeight - 1);
                Average2<uint16_t>(pPlane[13], pPlane[12], pPlane[14], nPitch, nExtendedWidth, nExtendedHeight);
                Average2<uint16_t>(pPlane[7], pPlane[4] + bytesPerSample, pPlane[6], nPitch, nExtendedWidth - 1, nExtendedHeight);
                Average2<uint16_t>(pPlane[15], pPlane[12] + bytesPerSample, pPlane[14], nPitch, nExtendedWidth - 1, nExtendedHeight);

            }
        }

    }
    isRefined = true;
    //   LeaveCriticalSection(&cs);
}


template <typename PixelType>
void MVPlane::RefineExtPel2(const uint8_t *pSrc2x8, int nSrc2xPitch, bool isExtPadded) {
    const PixelType *pSrc2x = (const PixelType *)pSrc2x8;
    PixelType *pp1 = (PixelType *)pPlane[1];
    PixelType *pp2 = (PixelType *)pPlane[2];
    PixelType *pp3 = (PixelType *)pPlane[3];

    nSrc2xPitch /= sizeof(PixelType);
    int nPitchTmp = nPitch / sizeof(PixelType);

    // pel clip may be already padded (i.e. is finest clip)
    if (!isExtPadded) {
        int offset = nPitchTmp * nVPadding + nHPadding;
        pp1 += offset;
        pp2 += offset;
        pp3 += offset;
    }

    for (int h = 0; h < nHeight; h++) {// assembler optimization?
        for (int w = 0; w < nWidth; w++) {
            pp1[w] = pSrc2x[(w<<1) + 1];
            pp2[w] = pSrc2x[(w<<1) + nSrc2xPitch];
            pp3[w] = pSrc2x[(w<<1) + nSrc2xPitch + 1];
        }
        pp1 += nPitchTmp;
        pp2 += nPitchTmp;
        pp3 += nPitchTmp;
        pSrc2x += nSrc2xPitch * 2;
    }

    if (!isExtPadded) {
        PadReferenceFrame<PixelType>(pPlane[1], nPitch, nHPadding, nVPadding, nWidth, nHeight);
        PadReferenceFrame<PixelType>(pPlane[2], nPitch, nHPadding, nVPadding, nWidth, nHeight);
        PadReferenceFrame<PixelType>(pPlane[3], nPitch, nHPadding, nVPadding, nWidth, nHeight);
    }
    isPadded = true;
}


template <typename PixelType>
void MVPlane::RefineExtPel4(const uint8_t *pSrc2x8, int nSrc2xPitch, bool isExtPadded) {
    const PixelType *pSrc2x = (const PixelType *)pSrc2x8;
    PixelType *pp[16];
    for (int i = 1; i < 16; i++)
        pp[i] = (PixelType *)pPlane[i];

    nSrc2xPitch /= sizeof(PixelType);
    int nPitchTmp = nPitch / sizeof(PixelType);

    if (!isExtPadded) {
        int offset = nPitchTmp * nVPadding + nHPadding;
        for (int i = 1; i < 16; i++)
            pp[i] += offset;
    }

    for (int h = 0; h < nHeight; h++) {// assembler optimization?
        for (int w = 0; w < nWidth; w++) {
            pp[1][w] = pSrc2x[(w<<2) + 1];
            pp[2][w] = pSrc2x[(w<<2) + 2];
            pp[3][w] = pSrc2x[(w<<2) + 3];
            pp[4][w] = pSrc2x[(w<<2) + nSrc2xPitch];
            pp[5][w] = pSrc2x[(w<<2) + nSrc2xPitch + 1];
            pp[6][w] = pSrc2x[(w<<2) + nSrc2xPitch + 2];
            pp[7][w] = pSrc2x[(w<<2) + nSrc2xPitch + 3];
            pp[8][w] = pSrc2x[(w<<2) + nSrc2xPitch*2];
            pp[9][w] = pSrc2x[(w<<2) + nSrc2xPitch*2 + 1];
            pp[10][w] = pSrc2x[(w<<2) + nSrc2xPitch*2 + 2];
            pp[11][w] = pSrc2x[(w<<2) + nSrc2xPitch*2 + 3];
            pp[12][w] = pSrc2x[(w<<2) + nSrc2xPitch*3];
            pp[13][w] = pSrc2x[(w<<2) + nSrc2xPitch*3 + 1];
            pp[14][w] = pSrc2x[(w<<2) + nSrc2xPitch*3 + 2];
            pp[15][w] = pSrc2x[(w<<2) + nSrc2xPitch*3 + 3];
        }
        for (int i = 1; i < 16; i++)
            pp[i] += nPitchTmp;
        pSrc2x += nSrc2xPitch*4;
    }
    if (!isExtPadded) {
        for (int i = 1; i < 16; i++)
            PadReferenceFrame<PixelType>(pPlane[i], nPitch, nHPadding, nVPadding, nWidth, nHeight);
    }
    isPadded = true;
}


void MVPlane::RefineExt(const uint8_t *pSrc2x, int nSrc2xPitch, bool isExtPadded) // copy from external upsized clip
{
    //    EnterCriticalSection(&cs);
    if (( nPel == 2 ) && ( !isRefined ))
    {
        if (bytesPerSample == 1)
            RefineExtPel2<uint8_t>(pSrc2x, nSrc2xPitch, isExtPadded);
        else
            RefineExtPel2<uint16_t>(pSrc2x, nSrc2xPitch, isExtPadded);
    }
    else if (( nPel == 4 ) && ( !isRefined ))
    {
        if (bytesPerSample == 1)
            RefineExtPel4<uint8_t>(pSrc2x, nSrc2xPitch, isExtPadded);
        else
            RefineExtPel4<uint16_t>(pSrc2x, nSrc2xPitch, isExtPadded);
    }
    isRefined = true;
    //   LeaveCriticalSection(&cs);

}

void MVPlane::ReduceTo(MVPlane *pReducedPlane, int rfilter)
{
    //    EnterCriticalSection(&cs);
    if ( !pReducedPlane->isFilled )
    {
        if (rfilter==0)
        {
            /* TODO: port the asm
               if (isse)
               {
               RB2F_iSSE(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
               pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight);
               }
               else
               */
            {
                if (bytesPerSample == 1)
                    RB2F_C<uint8_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                            pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight);
                else
                    RB2F_C<uint16_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                            pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight);
            }
        }
        else if (rfilter==1)
        {
            if (bytesPerSample == 1)
                RB2Filtered<uint8_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                        pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
            else
                RB2Filtered<uint16_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                        pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
        }
        else if (rfilter==2)
        {
            if (bytesPerSample == 1)
                RB2BilinearFiltered<uint8_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                    pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
            else
                RB2BilinearFiltered<uint16_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                    pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
        }
        else if (rfilter==3)
        {
            if (bytesPerSample == 1)
                RB2Quadratic<uint8_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                        pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
            else
                RB2Quadratic<uint16_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                        pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
        }
        else if (rfilter==4)
        {
            if (bytesPerSample == 1)
                RB2Cubic<uint8_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                        pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
            else
                RB2Cubic<uint16_t>(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
                        pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
        }
    }
    pReducedPlane->isFilled = true;
    //   LeaveCriticalSection(&cs);
}

void MVPlane::WritePlane(FILE *pFile)
{
    for ( int i = 0; i < nHeight; i++ )
        fwrite(pPlane[0] + i * nPitch + nOffsetPadding, 1, nWidth, pFile);
}

/******************************************************************************
 *                                                                             *
 *  MVFrame : a MVFrame is a threesome of MVPlane, some undefined, some        *
 *  defined, according to the nMode value                                      *
 *                                                                             *
 ******************************************************************************/

MVFrame::MVFrame(int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int _nMode, bool _isse, int _xRatioUV, int _yRatioUV, int _bitsPerSample)
{
    nMode = _nMode;
    isse = _isse;
    xRatioUV = _xRatioUV;
    yRatioUV = _yRatioUV;
    bitsPerSample = _bitsPerSample;

    if ( nMode & YPLANE )
        pYPlane = new MVPlane(nWidth, nHeight, nPel, nHPad, nVPad, isse, bitsPerSample);
    else
        pYPlane = 0;

    if ( nMode & UPLANE )
        pUPlane = new MVPlane(nWidth / xRatioUV, nHeight / yRatioUV, nPel, nHPad / xRatioUV, nVPad / yRatioUV, isse, bitsPerSample);
    else
        pUPlane = 0;

    if ( nMode & VPLANE )
        pVPlane = new MVPlane(nWidth / xRatioUV, nHeight / yRatioUV, nPel, nHPad / xRatioUV, nVPad / yRatioUV, isse, bitsPerSample);
    else
        pVPlane = 0;
}

void MVFrame::Update(int _nMode, uint8_t * pSrcY, int pitchY, uint8_t * pSrcU, int pitchU, uint8_t *pSrcV, int pitchV)
{
    if ( _nMode & nMode & YPLANE  ) //v2.0.8
        pYPlane->Update(pSrcY, pitchY);

    if ( _nMode & nMode & UPLANE  )
        pUPlane->Update(pSrcU, pitchU);

    if ( _nMode & nMode & VPLANE  )
        pVPlane->Update(pSrcV, pitchV);
}

MVFrame::~MVFrame()
{
    if ( nMode & YPLANE )
        delete pYPlane;

    if ( nMode & UPLANE )
        delete pUPlane;

    if ( nMode & VPLANE )
        delete pVPlane;
}

void MVFrame::ChangePlane(const unsigned char *pNewPlane, int nNewPitch, MVPlaneSet _nMode)
{
    if ( _nMode & nMode & YPLANE )
        pYPlane->ChangePlane(pNewPlane, nNewPitch);

    if ( _nMode & nMode & UPLANE )
        pUPlane->ChangePlane(pNewPlane, nNewPitch);

    if ( _nMode & nMode & VPLANE )
        pVPlane->ChangePlane(pNewPlane, nNewPitch);
}

void MVFrame::Refine(MVPlaneSet _nMode, int sharp)
{
    if (nMode & YPLANE & _nMode)
        pYPlane->Refine(sharp);

    if (nMode & UPLANE & _nMode)
        pUPlane->Refine(sharp);

    if (nMode & VPLANE & _nMode)
        pVPlane->Refine(sharp);
}

void MVFrame::Pad(MVPlaneSet _nMode)
{
    if (nMode & YPLANE & _nMode)
        pYPlane->Pad();

    if (nMode & UPLANE & _nMode)
        pUPlane->Pad();

    if (nMode & VPLANE & _nMode)
        pVPlane->Pad();
}

void MVFrame::ResetState()
{
    if ( nMode & YPLANE )
        pYPlane->ResetState();

    if ( nMode & UPLANE )
        pUPlane->ResetState();

    if ( nMode & VPLANE )
        pVPlane->ResetState();
}

void MVFrame::WriteFrame(FILE *pFile)
{
    if ( nMode & YPLANE )
        pYPlane->WritePlane(pFile);

    if ( nMode & UPLANE )
        pUPlane->WritePlane(pFile);

    if ( nMode & VPLANE )
        pVPlane->WritePlane(pFile);
}

void MVFrame::ReduceTo(MVFrame *pFrame, MVPlaneSet _nMode, int rfilter)
{
    if (nMode & YPLANE & _nMode)
        pYPlane->ReduceTo(pFrame->GetPlane(YPLANE), rfilter);

    if (nMode & UPLANE & _nMode)
        pUPlane->ReduceTo(pFrame->GetPlane(UPLANE), rfilter);

    if (nMode & VPLANE & _nMode)
        pVPlane->ReduceTo(pFrame->GetPlane(VPLANE), rfilter);
}

/******************************************************************************
 *                                                                             *
 *  MVGroupOfFrames : manage a hierachal frame structure                       *
 *                                                                             *
 ******************************************************************************/

MVGroupOfFrames::MVGroupOfFrames(int _nLevelCount, int _nWidth, int _nHeight, int _nPel, int _nHPad, int _nVPad, int nMode, bool isse, int _xRatioUV, int _yRatioUV, int _bitsPerSample)
{
    nLevelCount = _nLevelCount;
    nWidth = _nWidth;
    nHeight = _nHeight;
    nPel = _nPel;
    nHPad = _nHPad;
    nVPad = _nVPad;
    xRatioUV = _xRatioUV;
    yRatioUV = _yRatioUV;
    bitsPerSample = _bitsPerSample;
    pFrames = new MVFrame *[nLevelCount];

    pFrames[0] = new MVFrame(nWidth, nHeight, nPel, nHPad, nVPad, nMode, isse, xRatioUV, yRatioUV, bitsPerSample);
    for ( int i = 1; i < nLevelCount; i++ )
    {
        int nWidthi = PlaneWidthLuma(nWidth, i, xRatioUV, nHPad);//(nWidthi / 2) - ((nWidthi / 2) % xRatioUV); //  even for YV12
        int nHeighti = PlaneHeightLuma(nHeight, i, yRatioUV, nVPad);//(nHeighti / 2) - ((nHeighti / 2) % yRatioUV); // even for YV12
        pFrames[i] = new MVFrame(nWidthi, nHeighti, 1, nHPad, nVPad, nMode, isse, xRatioUV, yRatioUV, bitsPerSample);
    }
}

void MVGroupOfFrames::Update(int nMode, uint8_t * pSrcY, int pitchY, uint8_t * pSrcU, int pitchU, uint8_t *pSrcV, int pitchV) // v2.0
{
    for ( int i = 0; i < nLevelCount; i++ )
    {
        unsigned int offY = PlaneSuperOffset(false, nHeight, i, nPel, nVPad, pitchY, yRatioUV);
        unsigned int offU = PlaneSuperOffset(true, nHeight/yRatioUV, i, nPel, nVPad/yRatioUV, pitchU, yRatioUV);
        unsigned int offV = PlaneSuperOffset(true, nHeight/yRatioUV, i, nPel, nVPad/yRatioUV, pitchV, yRatioUV);
        pFrames[i]->Update (nMode, pSrcY+offY, pitchY, pSrcU+offU, pitchU, pSrcV+offV, pitchV);
    }
}

MVGroupOfFrames::~MVGroupOfFrames()
{
    for ( int i = 0; i < nLevelCount; i++ )
        delete pFrames[i];

    delete[] pFrames;
}

MVFrame *MVGroupOfFrames::GetFrame(int nLevel)
{
    if (( nLevel < 0 ) || ( nLevel >= nLevelCount )) return 0;
    return pFrames[nLevel];
}

void MVGroupOfFrames::SetPlane(const uint8_t *pNewSrc, int nNewPitch, MVPlaneSet nMode)
{
    pFrames[0]->ChangePlane(pNewSrc, nNewPitch, nMode);
}

void MVGroupOfFrames::Refine(MVPlaneSet nMode, int sharp)
{
    pFrames[0]->Refine(nMode, sharp);
}

void MVGroupOfFrames::Pad(MVPlaneSet nMode)
{
    pFrames[0]->Pad(nMode);
}

void MVGroupOfFrames::Reduce(MVPlaneSet _nMode, int rfilter)
{
    for (int i = 0; i < nLevelCount - 1; i++ )
    {
        pFrames[i]->ReduceTo(pFrames[i+1], _nMode, rfilter);
        pFrames[i+1]->Pad(YUVPLANES);
    }
}

void MVGroupOfFrames::ResetState()
{
    for ( int i = 0; i < nLevelCount; i++ )
        pFrames[i]->ResetState();
}
