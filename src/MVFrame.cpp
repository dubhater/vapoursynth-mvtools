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

#include "MVInterface.h"
#include "Padding.h"
#include "CopyCode.h"
#include "Interpolation.h"
#include "MVSuper.h"

/******************************************************************************
*                                                                             *
*  MVPlane : manages a single plane, allowing padding and refinin             *
*                                                                             *
******************************************************************************/

MVPlane::MVPlane(int _nWidth, int _nHeight, int _nPel, int _nHPad, int _nVPad, bool _isse)
{
   nWidth = _nWidth;
   nHeight = _nHeight;
   nPel = _nPel;
   nHPadding = _nHPad;
   nVPadding = _nVPad;
   isse = _isse;
   nHPaddingPel = _nHPad * nPel;
   nVPaddingPel = _nVPad * nPel;

   nExtendedWidth = nWidth + 2 * nHPadding;
   nExtendedHeight = nHeight + 2 * nVPadding;
/*
#ifdef	ALIGN_PLANES
   nPitch = (nExtendedWidth + ALIGN_PLANES-1) & (~(ALIGN_PLANES-1)); //ensure that each row gets aligned (ie. row length in memory is on a border)
   nOffsetPadding = nPitch * nVPadding + nHPadding;
   int numberOfPlanes = nPel * nPel;
   pPlane = new uint8_t *[numberOfPlanes * 2];	//add 2 second set of pointers, those point to the start of the memory (for freeing)
   for ( int i = 0; i < numberOfPlanes; i++ ){
		pPlane[numberOfPlanes+i] = new uint8_t[(nPitch * nExtendedHeight) + ALIGN_PLANES - 1];//add enough space for the shifted beginning (worst case)
		pPlane[i]=(uint8_t *)(((((int)pPlane[numberOfPlanes+i]) + nHPadding + ALIGN_PLANES - 1)&(~(ALIGN_PLANES - 1)))- nHPadding); //forces the pointer to the plane to be aligned relative to cache size
   }
#else

   nPitch = _nPitch;
   nOffsetPadding = nPitch * nVPadding + nHPadding;
*/
   pPlane = new uint8_t *[nPel * nPel];
//   for ( int i = 0; i < nPel * nPel; i++ )
//      pPlane[i] = new uint8_t[nPitch * nExtendedHeight];
//      pPlane[i] = pSrc + i*nPitch * nExtendedHeight;
//#endif
//    ResetState(); // 1.11.1
//   InitializeCriticalSection(&cs);
}

MVPlane::~MVPlane()
{
/*
#ifdef	ALIGN_PLANES
	for ( int i = nPel * nPel; i < nPel * nPel * 2; i++ ) //use the real aress of the allocated memory
		delete[] pPlane[i];
#else
//	for ( int i = 0; i < nPel * nPel; i++ )
//		delete[] pPlane[i];
#endif
*/
	delete[] pPlane;

//	DeleteCriticalSection(&cs);
}

void MVPlane::Update(uint8_t* pSrc, int _nPitch) //v2.0
{
//	EnterCriticalSection(&cs);
   nPitch = _nPitch;
   nOffsetPadding = nPitch * nVPadding + nHPadding;

   for ( int i = 0; i < nPel * nPel; i++ )
      pPlane[i] = pSrc + i*nPitch * nExtendedHeight;

    ResetState();
//   LeaveCriticalSection(&cs);
}
void MVPlane::ChangePlane(const uint8_t *pNewPlane, int nNewPitch)
{
//	EnterCriticalSection(&cs);
   if ( !isFilled )
      BitBlt(pPlane[0] + nOffsetPadding, nPitch, pNewPlane, nNewPitch, nWidth, nHeight, isse);
   isFilled = true;
//   LeaveCriticalSection(&cs);
}

void MVPlane::Pad()
{
//	EnterCriticalSection(&cs);
   if ( !isPadded )
      PadReferenceFrame(pPlane[0], nPitch, nHPadding, nVPadding, nWidth, nHeight);
   isPadded = true;
//   LeaveCriticalSection(&cs);
}

void MVPlane::Refine(int sharp)
{
//	EnterCriticalSection(&cs);
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
         HorizontalBilinear(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         VerticalBilinear(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         DiagonalBilinear(pPlane[3], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
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
         HorizontalBicubic(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         VerticalBicubic(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         HorizontalBicubic(pPlane[3], pPlane[2], nPitch, nPitch, nExtendedWidth, nExtendedHeight); // faster from ready-made horizontal
//         DiagonalBicubic(pPlane[3], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
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
         HorizontalWiener(pPlane[1], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         VerticalWiener(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         HorizontalWiener(pPlane[3], pPlane[2], nPitch, nPitch, nExtendedWidth, nExtendedHeight); // faster from ready-made horizontal
//         DiagonalWiener(pPlane[3], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
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
         HorizontalBilinear(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         VerticalBilinear(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         DiagonalBilinear(pPlane[10], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
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
         HorizontalBicubic(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         VerticalBicubic(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         HorizontalBicubic(pPlane[10], pPlane[8], nPitch, nPitch, nExtendedWidth, nExtendedHeight); // faster from ready-made horizontal
//         DiagonalBicubic(pPlane[3], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
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
         HorizontalWiener(pPlane[2], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         VerticalWiener(pPlane[8], pPlane[0], nPitch, nPitch, nExtendedWidth, nExtendedHeight);
         HorizontalWiener(pPlane[10], pPlane[8], nPitch, nPitch, nExtendedWidth, nExtendedHeight); // faster from ready-made horizontal
	  }
	}
     // now interpolate intermediate
     void (*Average2x)(unsigned char*, const unsigned char*, const unsigned char*, intptr_t, intptr_t, intptr_t) = isse ? mvtools_Average2_sse2 : Average2;
     Average2x(pPlane[1], pPlane[0], pPlane[2], nPitch, nExtendedWidth, nExtendedHeight);
     Average2x(pPlane[9], pPlane[8], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
     Average2x(pPlane[4], pPlane[0], pPlane[8], nPitch, nExtendedWidth, nExtendedHeight);
     Average2x(pPlane[6], pPlane[2], pPlane[10], nPitch, nExtendedWidth, nExtendedHeight);
     Average2x(pPlane[5], pPlane[4], pPlane[6], nPitch, nExtendedWidth, nExtendedHeight);

     Average2x(pPlane[3], pPlane[0] + 1, pPlane[2], nPitch, nExtendedWidth-1, nExtendedHeight);
     Average2x(pPlane[11], pPlane[8] + 1, pPlane[10], nPitch, nExtendedWidth-1, nExtendedHeight);
     Average2x(pPlane[12], pPlane[0] + nPitch, pPlane[8], nPitch, nExtendedWidth, nExtendedHeight-1);
     Average2x(pPlane[14], pPlane[2] + nPitch, pPlane[10], nPitch, nExtendedWidth, nExtendedHeight-1);
     Average2x(pPlane[13], pPlane[12], pPlane[14], nPitch, nExtendedWidth, nExtendedHeight);
     Average2x(pPlane[7], pPlane[4] + 1, pPlane[6], nPitch, nExtendedWidth-1, nExtendedHeight);
     Average2x(pPlane[15], pPlane[12] + 1, pPlane[14], nPitch, nExtendedWidth-1, nExtendedHeight);

   }
   isRefined = true;
//   LeaveCriticalSection(&cs);
}

void MVPlane::RefineExt(const uint8_t *pSrc2x, int nSrc2xPitch, bool isExtPadded) // copy from external upsized clip
{
//	EnterCriticalSection(&cs);
   if (( nPel == 2 ) && ( !isRefined ))
   {
        // pel clip may be already padded (i.e. is finest clip)
       int offset = isExtPadded ? 0 : nPitch*nVPadding + nHPadding;
	   uint8_t* pp1 = pPlane[1] + offset;
	   uint8_t* pp2 = pPlane[2] + offset;
	   uint8_t* pp3 = pPlane[3] + offset;

	   for (int h=0; h<nHeight; h++) // assembler optimization?
	   {
			   for (int w=0; w<nWidth; w++)
			   {
					pp1[w] = pSrc2x[(w<<1) + 1];
					pp2[w] = pSrc2x[(w<<1) + nSrc2xPitch];
					pp3[w] = pSrc2x[(w<<1) + nSrc2xPitch + 1];
			   }
			   pp1 += nPitch;
			   pp2 += nPitch;
			   pp3 += nPitch;
			   pSrc2x += nSrc2xPitch*2;
	   }
     if (!isExtPadded)
     {
	   PadReferenceFrame(pPlane[1], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[2], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[3], nPitch, nHPadding, nVPadding, nWidth, nHeight);
     }
	   isPadded = true;
   }
   else if (( nPel == 4 ) && ( !isRefined ))
   {
        // pel clip may be already padded (i.e. is finest clip)
       int offset = isExtPadded ? 0 : nPitch*nVPadding + nHPadding;
	   uint8_t* pp1 = pPlane[1] + offset;
	   uint8_t* pp2 = pPlane[2] + offset;
	   uint8_t* pp3 = pPlane[3] + offset;
	   uint8_t* pp4 = pPlane[4] + offset;
	   uint8_t* pp5 = pPlane[5] + offset;
	   uint8_t* pp6 = pPlane[6] + offset;
	   uint8_t* pp7 = pPlane[7] + offset;
	   uint8_t* pp8 = pPlane[8] + offset;
	   uint8_t* pp9 = pPlane[9] + offset;
	   uint8_t* pp10 = pPlane[10] + offset;
	   uint8_t* pp11 = pPlane[11] + offset;
	   uint8_t* pp12 = pPlane[12] + offset;
	   uint8_t* pp13 = pPlane[13] + offset;
	   uint8_t* pp14 = pPlane[14] + offset;
	   uint8_t* pp15 = pPlane[15] + offset;

	   for (int h=0; h<nHeight; h++) // assembler optimization?
	   {
			   for (int w=0; w<nWidth; w++)
			   {
					pp1[w] = pSrc2x[(w<<2) + 1];
					pp2[w] = pSrc2x[(w<<2) + 2];
					pp3[w] = pSrc2x[(w<<2) + 3];
					pp4[w] = pSrc2x[(w<<2) + nSrc2xPitch];
					pp5[w] = pSrc2x[(w<<2) + nSrc2xPitch + 1];
					pp6[w] = pSrc2x[(w<<2) + nSrc2xPitch + 2];
					pp7[w] = pSrc2x[(w<<2) + nSrc2xPitch + 3];
					pp8[w] = pSrc2x[(w<<2) + nSrc2xPitch*2];
					pp9[w] = pSrc2x[(w<<2) + nSrc2xPitch*2 + 1];
					pp10[w] = pSrc2x[(w<<2) + nSrc2xPitch*2 + 2];
					pp11[w] = pSrc2x[(w<<2) + nSrc2xPitch*2 + 3];
					pp12[w] = pSrc2x[(w<<2) + nSrc2xPitch*3];
					pp13[w] = pSrc2x[(w<<2) + nSrc2xPitch*3 + 1];
					pp14[w] = pSrc2x[(w<<2) + nSrc2xPitch*3 + 2];
					pp15[w] = pSrc2x[(w<<2) + nSrc2xPitch*3 + 3];
			   }
			   pp1 += nPitch;
			   pp2 += nPitch;
			   pp3 += nPitch;
			   pp4 += nPitch;
			   pp5 += nPitch;
			   pp6 += nPitch;
			   pp7 += nPitch;
			   pp8 += nPitch;
			   pp9 += nPitch;
			   pp10 += nPitch;
			   pp11 += nPitch;
			   pp12 += nPitch;
			   pp13 += nPitch;
			   pp14 += nPitch;
			   pp15 += nPitch;
			   pSrc2x += nSrc2xPitch*4;
	   }
     if (!isExtPadded)
     {
	   PadReferenceFrame(pPlane[1], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[2], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[3], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[4], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[5], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[6], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[7], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[8], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[9], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[10], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[11], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[12], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[13], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[14], nPitch, nHPadding, nVPadding, nWidth, nHeight);
	   PadReferenceFrame(pPlane[15], nPitch, nHPadding, nVPadding, nWidth, nHeight);
     }
	   isPadded = true;
   }
   isRefined = true;
//   LeaveCriticalSection(&cs);

}

void MVPlane::ReduceTo(MVPlane *pReducedPlane, int rfilter)
{
//	EnterCriticalSection(&cs);
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
         RB2F_C(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
            pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight);
      }
    }
    else if (rfilter==1)
    {
         RB2Filtered(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
            pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
    }
    else if (rfilter==2)
    {
         RB2BilinearFiltered(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
            pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
    }
    else if (rfilter==3)
    {
         RB2Quadratic(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
            pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, isse);
    }
    else if (rfilter==4)
    {
         RB2Cubic(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
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

MVFrame::MVFrame(int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int _nMode, bool _isse, int _yRatioUV)
{
   nMode = _nMode;
   isse = _isse;
   yRatioUV = _yRatioUV;

   if ( nMode & YPLANE )
      pYPlane = new MVPlane(nWidth, nHeight, nPel, nHPad, nVPad, isse);
   else
      pYPlane = 0;

   if ( nMode & UPLANE )
      pUPlane = new MVPlane(nWidth / 2, nHeight / yRatioUV, nPel, nHPad / 2, nVPad / yRatioUV, isse);
   else
      pUPlane = 0;

   if ( nMode & VPLANE )
      pVPlane = new MVPlane(nWidth / 2, nHeight / yRatioUV, nPel, nHPad / 2, nVPad / yRatioUV, isse);
   else
      pVPlane = 0;
}

void MVFrame::Update(int _nMode, uint8_t * pSrcY, int pitchY, uint8_t * pSrcU, int pitchU, uint8_t *pSrcV, int pitchV)
{
//   nMode = _nMode;

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

MVGroupOfFrames::MVGroupOfFrames(int _nLevelCount, int _nWidth, int _nHeight, int _nPel, int _nHPad, int _nVPad, int nMode, bool isse, int _yRatioUV)
{
   nLevelCount = _nLevelCount;
//   nRefCount = 0;
//   nFrameIdx = -1;
   nWidth = _nWidth;
   nHeight = _nHeight;
   nPel = _nPel;
   nHPad = _nHPad;
   nVPad = _nVPad;
   yRatioUV = _yRatioUV;
   pFrames = new MVFrame *[nLevelCount];

   pFrames[0] = new MVFrame(nWidth, nHeight, nPel, nHPad, nVPad, nMode, isse, yRatioUV);
   for ( int i = 1; i < nLevelCount; i++ )
   {
      int nWidthi = PlaneWidthLuma(nWidth, i, 2, nHPad);//(nWidthi / 2) - ((nWidthi / 2) % 2); //  even for YV12, YUY2
      int nHeighti = PlaneHeightLuma(nHeight, i, yRatioUV, nVPad);//(nHeighti / 2) - ((nHeighti / 2) % yRatioUV); // even for YV12
      pFrames[i] = new MVFrame(nWidthi, nHeighti, 1, nHPad, nVPad, nMode, isse, yRatioUV);
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

//void MVGroupOfFrames::SetFrameIdx(int _nFrameIdx)
//{
//   nRefCount = 0;
//   nFrameIdx = _nFrameIdx;
//}

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
