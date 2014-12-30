// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - YUY2

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

#include <string.h>
//#include "Padding.h"


void PadCorner(unsigned char *p, unsigned char v, int hPad, int vPad, int refPitch)
{
    for ( int i = 0; i < vPad; i++ )
    {
        memset(p, v, hPad); // faster than loop
        //        for ( int j = 0; j < hPad; j++ )
        //        {
        //            p[j] = v;
        //        }
        p += refPitch;
    }
}

#if 0
Padding::Padding(PClip _child, int hPad, int vPad, bool _planar, IScriptEnvironment* env) :
    GenericVideoFilter(_child)
{
    if ( !vi.IsYV12() && !vi.IsYUY2() )
        env->ThrowError("Padding : clip must be in the YV12 or YUY2 color format");

    horizontalPadding = hPad;
    verticalPadding = vPad;
    width = vi.width;
    height = vi.height;
    planar = _planar;

    if ( (vi.pixel_type & VideoInfo::CS_YUY2) == VideoInfo::CS_YUY2 && !planar)
    {
        SrcPlanes =  new YUY2Planes(vi.width, vi.height);
    }
    vi.width += horizontalPadding * 2;
    vi.height += verticalPadding * 2;
    if ( (vi.pixel_type & VideoInfo::CS_YUY2) == VideoInfo::CS_YUY2 && !planar)
    {
        DstPlanes =  new YUY2Planes(vi.width, vi.height);
    }
}

Padding::~Padding()
{
    if ( (vi.pixel_type & VideoInfo::CS_YUY2) == VideoInfo::CS_YUY2 && !planar)
    {
        delete SrcPlanes;
        delete DstPlanes;
    }
}

PVideoFrame __stdcall Padding::GetFrame(int n, IScriptEnvironment *env)
{
    PVideoFrame src = child->GetFrame(n, env);
    PVideoFrame dst = env->NewVideoFrame(vi);

    const unsigned char *pSrc[3];
    unsigned char *pDst[3];
    int nDstPitches[3];
    int nSrcPitches[3];
    unsigned char *pDstYUY2;
    int nDstPitchYUY2;

    bool isse = env->GetCPUFlags() >= CPUF_INTEGER_SSE;

    int yRatioUV = (vi.IsYV12()) ? 2 : 1;

    if ( (vi.pixel_type & VideoInfo::CS_YUY2) == VideoInfo::CS_YUY2 )
    {
        if (!planar)
        {
            const unsigned char *pSrcYUY2 = src->GetReadPtr();
            int nSrcPitchYUY2 = src->GetPitch();
            pSrc[0] = SrcPlanes->GetPtr();
            pSrc[1] = SrcPlanes->GetPtrU();
            pSrc[2] = SrcPlanes->GetPtrV();
            nSrcPitches[0]  = SrcPlanes->GetPitch();
            nSrcPitches[1]  = SrcPlanes->GetPitchUV();
            nSrcPitches[2]  = SrcPlanes->GetPitchUV();
            YUY2ToPlanes(pSrcYUY2, nSrcPitchYUY2, width, height,
                    pSrc[0], nSrcPitches[0], pSrc[1], pSrc[2], nSrcPitches[1], isse);
            pDst[0] = DstPlanes->GetPtr();
            pDst[1] = DstPlanes->GetPtrU();
            pDst[2] = DstPlanes->GetPtrV();
            nDstPitches[0]  = DstPlanes->GetPitch();
            nDstPitches[1]  = DstPlanes->GetPitchUV();
            nDstPitches[2]  = DstPlanes->GetPitchUV();
        }
        else // planar YUY2
        {
            pSrc[0] = src->GetReadPtr();
            nSrcPitches[0]  = src->GetPitch();
            nSrcPitches[1]  = src->GetPitch();
            nSrcPitches[2]  = src->GetPitch();
            pSrc[1] = pSrc[0] + src->GetRowSize()/2;
            pSrc[2] = pSrc[1] + src->GetRowSize()/4;
            pDst[0] = dst->GetWritePtr();
            nDstPitches[0]  = dst->GetPitch();
            nDstPitches[1]  = dst->GetPitch();
            nDstPitches[2]  = dst->GetPitch();
            pDst[1] = pDst[0] + dst->GetRowSize()/2;
            pDst[2] = pDst[1] + dst->GetRowSize()/4;
        }
    }
    else
    {
        pDst[0] = dst->GetWritePtr(PLANAR_Y);
        pDst[1] = dst->GetWritePtr(PLANAR_U);
        pDst[2] = dst->GetWritePtr(PLANAR_V);
        nDstPitches[0] = dst->GetPitch(PLANAR_Y);
        nDstPitches[1] = dst->GetPitch(PLANAR_U);
        nDstPitches[2] = dst->GetPitch(PLANAR_V);
        pSrc[0] = src->GetReadPtr(PLANAR_Y);
        pSrc[1] = src->GetReadPtr(PLANAR_U);
        pSrc[2] = src->GetReadPtr(PLANAR_V);
        nSrcPitches[0] = src->GetPitch(PLANAR_Y);
        nSrcPitches[1] = src->GetPitch(PLANAR_U);
        nSrcPitches[2] = src->GetPitch(PLANAR_V);
    }


    env->BitBlt(pDst[0] + horizontalPadding + verticalPadding * nDstPitches[0], nDstPitches[0],
            pSrc[0], nSrcPitches[0], width, height);
    PadReferenceFrame(pDst[0], nDstPitches[0], horizontalPadding, verticalPadding, width, height);


    env->BitBlt(pDst[1] + horizontalPadding/2 + verticalPadding/yRatioUV * nDstPitches[1],
            nDstPitches[1],    pSrc[1], nSrcPitches[1], width/2, height/yRatioUV);
    PadReferenceFrame(pDst[1], nDstPitches[1], horizontalPadding/2, verticalPadding/yRatioUV, width/2, height/yRatioUV);


    env->BitBlt(pDst[2] + horizontalPadding/2 + verticalPadding/yRatioUV * nDstPitches[2],
            nDstPitches[2],    pSrc[2], nSrcPitches[2], width/2, height/yRatioUV);
    PadReferenceFrame(pDst[2], nDstPitches[2], horizontalPadding/2, verticalPadding/yRatioUV, width/2, height/yRatioUV);

    if ( (vi.pixel_type & VideoInfo::CS_YUY2) == VideoInfo::CS_YUY2 && !planar)
    {
        pDstYUY2 = dst->GetWritePtr();
        nDstPitchYUY2 = dst->GetPitch();
        YUY2FromPlanes(pDstYUY2, nDstPitchYUY2, vi.width, vi.height,
                pDst[0], nDstPitches[0], pDst[1], pDst[2], nDstPitches[1], isse);
    }
    return dst;
}
#endif

void PadReferenceFrame(unsigned char *refFrame, int refPitch, int hPad, int vPad, int width, int height)
{
    unsigned char value;
    unsigned char *pfoff = refFrame + vPad * refPitch + hPad;
    unsigned char *p;

    // Up-Left
    PadCorner(refFrame, pfoff[0], hPad, vPad, refPitch);
    // Up-Right
    PadCorner(refFrame + hPad + width, pfoff[width - 1], hPad, vPad, refPitch);
    // Down-Left
    PadCorner(refFrame + (vPad + height) * refPitch,
            pfoff[(height - 1) * refPitch], hPad, vPad, refPitch);
    // Down-Right
    PadCorner(refFrame + hPad + width + (vPad + height) * refPitch,
            pfoff[(height - 1) * refPitch + width - 1], hPad, vPad, refPitch);

    // Up
    for ( int i = 0; i < width; i++ )
    {
        value = pfoff[i];
        p = refFrame + hPad + i;
        for ( int j = 0; j < vPad; j++ )
        {
            p[0] = value;
            p += refPitch;
        }
    }

    // Left
    for ( int i = 0; i < height; i++ )
    {
        value = pfoff[i*refPitch];
        p = refFrame + (vPad + i) * refPitch;
        for ( int j = 0; j < hPad; j++ )
            p[j] = value;
    }

    // Right
    for ( int i = 0; i < height; i++ )
    {
        value = pfoff[i * refPitch + width - 1];
        p = refFrame + (vPad + i) * refPitch + width + hPad;
        for ( int j = 0; j < hPad; j++ )
            p[j] = value;
    }

    // Down
    for ( int i = 0; i < width; i++ )
    {
        value = pfoff[i + (height - 1) * refPitch];
        p = refFrame + hPad + i + (height + vPad) * refPitch;
        for ( int j = 0; j < vPad; j++ )
        {
            p[0] = value;
            p += refPitch;
        }
    }
}
