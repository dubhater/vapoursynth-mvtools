// Make a motion compensate temporal denoiser
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


#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "MaskFun.h"
#include "MVInterface.h"
#include "SimpleResize.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *finest;
    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    float time;
    float ml;
    int blend;
    int thscd1;
    int thscd2;
    int isse;

    MVClipDicks *mvClipB;
    MVClipDicks *mvClipF;

    MVFilter *bleh;

    int nSuperHPad;

    int nBlkXP;
    int nBlkYP;
    int nWidthP;
    int nHeightP;
    int nWidthPUV;
    int nHeightPUV;
    int nWidthUV;
    int nHeightUV;
    int nVPaddingUV;
    int nHPaddingUV;
    int VPitchY;
    int VPitchUV;

    int time256;

    int *LUTVB;
    int *LUTVF;

    SimpleResize *upsizer;
    SimpleResize *upsizerUV;
} MVFlowInterData;


static void VS_CC mvflowinterInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVFlowInterData *d = (MVFlowInterData *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvflowinterGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVFlowInterData *d = (MVFlowInterData *) * instanceData;

    if (activationReason == arInitial) {
        int off = d->mvClipB->GetDeltaFrame(); // integer offset of reference frame

        if (n + off < d->vi->numFrames || !d->vi->numFrames) {
            vsapi->requestFrameFilter(n, d->mvfw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvfw, frameCtx);

            vsapi->requestFrameFilter(n, d->mvbw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvbw, frameCtx);

            vsapi->requestFrameFilter(n, d->finest, frameCtx);
            vsapi->requestFrameFilter(n + off, d->finest, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->node, frameCtx);
        vsapi->requestFrameFilter(d->vi->numFrames ? VSMIN(n + off, d->vi->numFrames - 1) : n + off, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        uint8_t *pDst[3];
        const uint8_t *pRef[3], *pSrc[3];
        int nDstPitches[3];
        int nRefPitches[3];
        int nSrcPitches[3];

        MVClipBalls ballsF(d->mvClipF, vsapi);
        MVClipBalls ballsB(d->mvClipB, vsapi);

        bool isUsableB = false;
        bool isUsableF = false;

        int off = d->mvClipB->GetDeltaFrame(); // integer offset of reference frame

        if (n + off < d->vi->numFrames || !d->vi->numFrames) {
            const VSFrameRef *mvF = vsapi->getFrameFilter(n + off, d->mvfw, frameCtx);
            ballsF.Update(mvF);
            vsapi->freeFrame(mvF);
            isUsableF = ballsF.IsUsable();

            const VSFrameRef *mvB = vsapi->getFrameFilter(n, d->mvbw, frameCtx);
            ballsB.Update(mvB);
            vsapi->freeFrame(mvB);
            isUsableB = ballsB.IsUsable();
        }

        const int nWidth = d->bleh->nWidth;
        const int nHeight = d->bleh->nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int time256 = d->time256;
        const int blend = d->blend;
        const int isse = d->isse;

        if ( isUsableB && isUsableF )
        {
            const VSFrameRef *src = vsapi->getFrameFilter(n, d->finest, frameCtx);
            const VSFrameRef *ref = vsapi->getFrameFilter(n + off, d->finest, frameCtx); //  ref for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

            for (int i = 0; i < 3; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
            }

            const float ml = d->ml;
            const int yRatioUV = d->bleh->yRatioUV;
            const int nBlkX = d->bleh->nBlkX;
            const int nBlkY = d->bleh->nBlkY;
            const int nBlkSizeX = d->bleh->nBlkSizeX;
            const int nBlkSizeY = d->bleh->nBlkSizeY;
            const int nOverlapX = d->bleh->nOverlapX;
            const int nOverlapY = d->bleh->nOverlapY;
            const int nVPadding = d->bleh->nVPadding;
            const int nHPadding = d->bleh->nHPadding;
            const int nVPaddingUV = d->nVPaddingUV;
            const int nHPaddingUV = d->nHPaddingUV;
            const int nPel = d->bleh->nPel;
            const int VPitchY = d->VPitchY;
            const int VPitchUV = d->VPitchUV;
            const int nHeightP = d->nHeightP;
            const int nHeightPUV = d->nHeightPUV;
            const int nBlkXP = d->nBlkXP;
            const int nBlkYP = d->nBlkYP;
            SimpleResize *upsizer = d->upsizer;
            SimpleResize *upsizerUV = d->upsizerUV;
            const int *LUTVB = d->LUTVB;
            const int *LUTVF = d->LUTVF;

            int nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * nPel;
            int nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * nPel;


            uint8_t *VXFullYB = new uint8_t [nHeightP * VPitchY];
            uint8_t *VXFullUVB = new uint8_t [nHeightPUV * VPitchUV];
            uint8_t *VYFullYB = new uint8_t [nHeightP * VPitchY];
            uint8_t *VYFullUVB = new uint8_t [nHeightPUV * VPitchUV];

            uint8_t *VXFullYF = new uint8_t [nHeightP * VPitchY];
            uint8_t *VXFullUVF = new uint8_t [nHeightPUV * VPitchUV];
            uint8_t *VYFullYF = new uint8_t [nHeightP * VPitchY];
            uint8_t *VYFullUVF = new uint8_t [nHeightPUV * VPitchUV];

            uint8_t *VXSmallYB = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *VYSmallYB = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *VXSmallUVB = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *VYSmallUVB = new uint8_t [nBlkXP * nBlkYP];

            uint8_t *VXSmallYF = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *VYSmallYF = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *VXSmallUVF = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *VYSmallUVF = new uint8_t [nBlkXP * nBlkYP];

            uint8_t *MaskSmallB = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *MaskFullYB = new uint8_t [nHeightP * VPitchY];
            uint8_t *MaskFullUVB = new uint8_t [nHeightPUV * VPitchUV];

            uint8_t *MaskSmallF = new uint8_t [nBlkXP * nBlkYP];
            uint8_t *MaskFullYF = new uint8_t [nHeightP * VPitchY];
            uint8_t *MaskFullUVF = new uint8_t [nHeightPUV * VPitchUV];


            // make  vector vx and vy small masks
            // 1. ATTENTION: vectors are assumed SHORT (|vx|, |vy| < 127) !
            // 2. they will be zeroed if not
            // 3. added 128 to all values
            MakeVectorSmallMasks(&ballsB, nBlkX, nBlkY, VXSmallYB, nBlkXP, VYSmallYB, nBlkXP);
            MakeVectorSmallMasks(&ballsF, nBlkX, nBlkY, VXSmallYF, nBlkXP, VYSmallYF, nBlkXP);
            if (nBlkXP > nBlkX) // fill right
            {
                for (int j=0; j<nBlkY; j++)
                {
                    VXSmallYB[j*nBlkXP + nBlkX] = VSMIN(VXSmallYB[j*nBlkXP + nBlkX-1],128);
                    VYSmallYB[j*nBlkXP + nBlkX] = VYSmallYB[j*nBlkXP + nBlkX-1];
                    VXSmallYF[j*nBlkXP + nBlkX] = VSMIN(VXSmallYF[j*nBlkXP + nBlkX-1],128);
                    VYSmallYF[j*nBlkXP + nBlkX] = VYSmallYF[j*nBlkXP + nBlkX-1];
                }
            }
            if (nBlkYP > nBlkY) // fill bottom
            {
                for (int i=0; i<nBlkXP; i++)
                {
                    VXSmallYB[nBlkXP*nBlkY +i] = VXSmallYB[nBlkXP*(nBlkY-1) +i];
                    VYSmallYB[nBlkXP*nBlkY +i] = VSMIN(VYSmallYB[nBlkXP*(nBlkY-1) +i],128);
                    VXSmallYF[nBlkXP*nBlkY +i] = VXSmallYF[nBlkXP*(nBlkY-1) +i];
                    VYSmallYF[nBlkXP*nBlkY +i] = VSMIN(VYSmallYF[nBlkXP*(nBlkY-1) +i],128);
                }
            }
            VectorSmallMaskYToHalfUV(VXSmallYB, nBlkXP, nBlkYP, VXSmallUVB, 2);
            VectorSmallMaskYToHalfUV(VYSmallYB, nBlkXP, nBlkYP, VYSmallUVB, yRatioUV);
            VectorSmallMaskYToHalfUV(VXSmallYF, nBlkXP, nBlkYP, VXSmallUVF, 2);
            VectorSmallMaskYToHalfUV(VYSmallYF, nBlkXP, nBlkYP, VYSmallUVF, yRatioUV);

            // analyse vectors field to detect occlusion
            //      double occNormB = (256-time256)/(256*ml);
            MakeVectorOcclusionMaskTime(&ballsB, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallB, nBlkXP, (256-time256), nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            //      double occNormF = time256/(256*ml);
            MakeVectorOcclusionMaskTime(&ballsF, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallF, nBlkXP, time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            if (nBlkXP > nBlkX) // fill right
            {
                for (int j=0; j<nBlkY; j++)
                {
                    MaskSmallB[j*nBlkXP + nBlkX] = MaskSmallB[j*nBlkXP + nBlkX-1];
                    MaskSmallF[j*nBlkXP + nBlkX] = MaskSmallF[j*nBlkXP + nBlkX-1];
                }
            }
            if (nBlkYP > nBlkY) // fill bottom
            {
                for (int i=0; i<nBlkXP; i++)
                {
                    MaskSmallB[nBlkXP*nBlkY +i] = MaskSmallB[nBlkXP*(nBlkY-1) +i];
                    MaskSmallF[nBlkXP*nBlkY +i] = MaskSmallF[nBlkXP*(nBlkY-1) +i];
                }
            }
            // upsize (bilinear interpolate) vector masks to fullframe size


            upsizer->Resize(VXFullYB, VPitchY, VXSmallYB, nBlkXP);
            upsizer->Resize(VYFullYB, VPitchY, VYSmallYB, nBlkXP);
            upsizerUV->Resize(VXFullUVB, VPitchUV, VXSmallUVB, nBlkXP);
            upsizerUV->Resize(VYFullUVB, VPitchUV, VYSmallUVB, nBlkXP);

            upsizer->Resize(VXFullYF, VPitchY, VXSmallYF, nBlkXP);
            upsizer->Resize(VYFullYF, VPitchY, VYSmallYF, nBlkXP);
            upsizerUV->Resize(VXFullUVF, VPitchUV, VXSmallUVF, nBlkXP);
            upsizerUV->Resize(VYFullUVF, VPitchUV, VYSmallUVF, nBlkXP);

            upsizer->Resize(MaskFullYB, VPitchY, MaskSmallB, nBlkXP);
            upsizerUV->Resize(MaskFullUVB, VPitchUV, MaskSmallB, nBlkXP);

            upsizer->Resize(MaskFullYF, VPitchY, MaskSmallF, nBlkXP);
            upsizerUV->Resize(MaskFullUVF, VPitchUV, MaskSmallF, nBlkXP);


            const VSFrameRef *mvFF = vsapi->getFrameFilter(n, d->mvfw, frameCtx);
            ballsF.Update(mvFF);
            vsapi->freeFrame(mvFF);

            const VSFrameRef *mvBB = vsapi->getFrameFilter(n + off, d->mvbw, frameCtx);
            ballsB.Update(mvBB);
            vsapi->freeFrame(mvBB);


            if ( ballsB.IsUsable() && ballsF.IsUsable() )
            {
                uint8_t *VXFullYBB = new uint8_t [nHeightP * VPitchY];
                uint8_t *VXFullUVBB = new uint8_t [nHeightPUV * VPitchUV];
                uint8_t *VYFullYBB = new uint8_t [nHeightP * VPitchY];
                uint8_t *VYFullUVBB = new uint8_t [nHeightPUV * VPitchUV];

                uint8_t *VXFullYFF = new uint8_t [nHeightP * VPitchY];
                uint8_t *VXFullUVFF = new uint8_t [nHeightPUV * VPitchUV];
                uint8_t *VYFullYFF = new uint8_t [nHeightP * VPitchY];
                uint8_t *VYFullUVFF = new uint8_t [nHeightPUV * VPitchUV];

                uint8_t *VXSmallYBB = new uint8_t [nBlkXP * nBlkYP];
                uint8_t *VYSmallYBB = new uint8_t [nBlkXP * nBlkYP];
                uint8_t *VXSmallUVBB = new uint8_t [nBlkXP * nBlkYP];
                uint8_t *VYSmallUVBB = new uint8_t [nBlkXP * nBlkYP];

                uint8_t *VXSmallYFF = new uint8_t [nBlkXP * nBlkYP];
                uint8_t *VYSmallYFF = new uint8_t [nBlkXP * nBlkYP];
                uint8_t *VXSmallUVFF = new uint8_t [nBlkXP * nBlkYP];
                uint8_t *VYSmallUVFF = new uint8_t [nBlkXP * nBlkYP];

                // get vector mask from extra frames
                MakeVectorSmallMasks(&ballsB, nBlkX, nBlkY, VXSmallYBB, nBlkXP, VYSmallYBB, nBlkXP);
                MakeVectorSmallMasks(&ballsF, nBlkX, nBlkY, VXSmallYFF, nBlkXP, VYSmallYFF, nBlkXP);
                if (nBlkXP > nBlkX) // fill right
                {
                    for (int j=0; j<nBlkY; j++)
                    {
                        VXSmallYBB[j*nBlkXP + nBlkX] = VSMIN(VXSmallYBB[j*nBlkXP + nBlkX-1],128);
                        VYSmallYBB[j*nBlkXP + nBlkX] = VYSmallYBB[j*nBlkXP + nBlkX-1];
                        VXSmallYFF[j*nBlkXP + nBlkX] = VSMIN(VXSmallYFF[j*nBlkXP + nBlkX-1],128);
                        VYSmallYFF[j*nBlkXP + nBlkX] = VYSmallYFF[j*nBlkXP + nBlkX-1];
                    }
                }
                if (nBlkYP > nBlkY) // fill bottom
                {
                    for (int i=0; i<nBlkXP; i++)
                    {
                        VXSmallYBB[nBlkXP*nBlkY +i] = VXSmallYBB[nBlkXP*(nBlkY-1) +i];
                        VYSmallYBB[nBlkXP*nBlkY +i] = VSMIN(VYSmallYBB[nBlkXP*(nBlkY-1) +i],128);
                        VXSmallYFF[nBlkXP*nBlkY +i] = VXSmallYFF[nBlkXP*(nBlkY-1) +i];
                        VYSmallYFF[nBlkXP*nBlkY +i] = VSMIN(VYSmallYFF[nBlkXP*(nBlkY-1) +i],128);
                    }
                }
                VectorSmallMaskYToHalfUV(VXSmallYBB, nBlkXP, nBlkYP, VXSmallUVBB, 2);
                VectorSmallMaskYToHalfUV(VYSmallYBB, nBlkXP, nBlkYP, VYSmallUVBB, yRatioUV);
                VectorSmallMaskYToHalfUV(VXSmallYFF, nBlkXP, nBlkYP, VXSmallUVFF, 2);
                VectorSmallMaskYToHalfUV(VYSmallYFF, nBlkXP, nBlkYP, VYSmallUVFF, yRatioUV);

                // upsize vectors to full frame
                upsizer->Resize(VXFullYBB, VPitchY, VXSmallYBB, nBlkXP);
                upsizer->Resize(VYFullYBB, VPitchY, VYSmallYBB, nBlkXP);
                upsizerUV->Resize(VXFullUVBB, VPitchUV, VXSmallUVBB, nBlkXP);
                upsizerUV->Resize(VYFullUVBB, VPitchUV, VYSmallUVBB, nBlkXP);

                upsizer->Resize(VXFullYFF, VPitchY, VXSmallYFF, nBlkXP);
                upsizer->Resize(VYFullYFF, VPitchY, VYSmallYFF, nBlkXP);
                upsizerUV->Resize(VXFullUVFF, VPitchUV, VXSmallUVFF, nBlkXP);
                upsizerUV->Resize(VYFullUVFF, VPitchUV, VYSmallUVFF, nBlkXP);

                FlowInterExtra(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                        VXFullYB, VXFullYF, VYFullYB, VYFullYF, MaskFullYB, MaskFullYF, VPitchY,
                        nWidth, nHeight, time256, nPel, LUTVB, LUTVF, VXFullYBB, VXFullYFF, VYFullYBB, VYFullYFF);
                FlowInterExtra(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                        VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                        nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF);
                FlowInterExtra(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                        VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                        nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF);

                delete [] VXFullYBB;
                delete [] VXFullUVBB;
                delete [] VYFullYBB;
                delete [] VYFullUVBB;
                delete [] VXSmallYBB;
                delete [] VYSmallYBB;
                delete [] VXSmallUVBB;
                delete [] VYSmallUVBB;
                delete [] VXFullYFF;
                delete [] VXFullUVFF;
                delete [] VYFullYFF;
                delete [] VYFullUVFF;
                delete [] VXSmallYFF;
                delete [] VYSmallYFF;
                delete [] VXSmallUVFF;
                delete [] VYSmallUVFF;
            }
            else // bad extra frames, use old method without extra frames
            {
                FlowInter(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                        VXFullYB, VXFullYF, VYFullYB, VYFullYF, MaskFullYB, MaskFullYF, VPitchY,
                        nWidth, nHeight, time256, nPel, LUTVB, LUTVF);
                FlowInter(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                        VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                        nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF);
                FlowInter(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                        VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                        nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF);
            }


            delete [] VXFullYB;
            delete [] VXFullUVB;
            delete [] VYFullYB;
            delete [] VYFullUVB;
            delete [] VXSmallYB;
            delete [] VYSmallYB;
            delete [] VXSmallUVB;
            delete [] VYSmallUVB;
            delete [] VXFullYF;
            delete [] VXFullUVF;
            delete [] VYFullYF;
            delete [] VYFullUVF;
            delete [] VXSmallYF;
            delete [] VYSmallYF;
            delete [] VXSmallUVF;
            delete [] VYSmallUVF;

            delete [] MaskSmallB;
            delete [] MaskFullYB;
            delete [] MaskFullUVB;
            delete [] MaskSmallF;
            delete [] MaskFullYF;
            delete [] MaskFullUVF;

            vsapi->freeFrame(src);
            vsapi->freeFrame(ref);

            return dst;
        }
        else // not usable
        {
            // poor estimation

            const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);

            if (blend) //let's blend src with ref frames like ConvertFPS
            {
                const VSFrameRef *ref = vsapi->getFrameFilter(d->vi->numFrames ? VSMIN(n + off, d->vi->numFrames - 1) : n + off, d->node, frameCtx);

                VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

                for (int i = 0; i < 3; i++) {
                    pDst[i] = vsapi->getWritePtr(dst, i);
                    pRef[i] = vsapi->getReadPtr(ref, i);
                    pSrc[i] = vsapi->getReadPtr(src, i);
                    nDstPitches[i] = vsapi->getStride(dst, i);
                    nRefPitches[i] = vsapi->getStride(ref, i);
                    nSrcPitches[i] = vsapi->getStride(src, i);
                }

                // blend with time weight
                Blend(pDst[0], pSrc[0], pRef[0], nHeight, nWidth, nDstPitches[0], nSrcPitches[0], nRefPitches[0], time256, isse);
                Blend(pDst[1], pSrc[1], pRef[1], nHeightUV, nWidthUV, nDstPitches[1], nSrcPitches[1], nRefPitches[1], time256, isse);
                Blend(pDst[2], pSrc[2], pRef[2], nHeightUV, nWidthUV, nDstPitches[2], nSrcPitches[2], nRefPitches[2], time256, isse);

                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);

                return dst;
            }
            else // no blend
            {
                return src; // like ChangeFPS
            }

        }
    }

    return 0;
}


static void VS_CC mvflowinterFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVFlowInterData *d = (MVFlowInterData *)instanceData;

    delete d->mvClipB;
    delete d->mvClipF;

    delete d->bleh;

    delete d->upsizer;
    delete d->upsizerUV;

    delete d->LUTVB;
    delete d->LUTVF;

    vsapi->freeNode(d->finest);
    vsapi->freeNode(d->super);
    vsapi->freeNode(d->mvfw);
    vsapi->freeNode(d->mvbw);
    vsapi->freeNode(d->node);
    free(d);
}


static void VS_CC mvflowinterCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVFlowInterData d;
    MVFlowInterData *data;

    int err;

    d.time = (float)vsapi->propGetFloat(in, "time", 0, &err);
    if (err)
        d.time = 50.0f;

    d.ml = (float)vsapi->propGetFloat(in, "ml", 0, &err);
    if (err)
        d.ml = 100.0f;

    d.blend = !!vsapi->propGetInt(in, "blend", 0, &err);
    if (err)
        d.blend = 1;

    d.thscd1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = vsapi->propGetInt(in, "thscd2", 0, &err);
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.isse = !!vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;


    if (d.time < 0.0f || d.time > 100.0f) {
        vsapi->setError(out, "FlowInter: time must be between 0 and 100 % (inclusive).");
        return;
    }

    if (d.ml <= 0.0f) {
        vsapi->setError(out, "FlowInter: ml must be greater than 0.");
        return;
    }

    d.time256 = (int)(d.time * 256.0f / 100.0f);


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("FlowInter: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[2];
    int nHeightS = vsapi->propGetInt(props, "Super height", 0, &evil_err[0]);
    d.nSuperHPad = vsapi->propGetInt(props, "Super hpad", 0, &evil_err[1]);
    vsapi->freeFrame(evil);

    for (int i = 0; i < 2; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "FlowInter: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);

    // XXX Fuck all this trying.
    try {
        d.mvClipB = new MVClipDicks(d.mvbw, d.thscd1, d.thscd2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw);
        return;
    }

    try {
        d.mvClipF = new MVClipDicks(d.mvfw, d.thscd1, d.thscd2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        return;
    }

    // XXX Alternatively, use both clips' delta as offsets in GetFrame.
    if (d.mvClipF->GetDeltaFrame() != d.mvClipB->GetDeltaFrame()) {
        vsapi->setError(out, "FlowInter: mvbw and mvfw must be generated with the same delta.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    // Make sure the motion vector clips are correct.
    if (!d.mvClipB->IsBackward() || d.mvClipF->IsBackward()) {
        if (!d.mvClipB->IsBackward())
            vsapi->setError(out, "FlowInter: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "FlowInter: mvfw must be generated with isb=False.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    try {
        d.bleh = new MVFilter(d.mvfw, "FlowInter", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    try {
        // So it checks the similarity of mvfw and mvfw? ?????
        // Copied straight from 2.5.11.3...
        d.bleh->CheckSimilarity(d.mvClipF, "mvfw");
        d.bleh->CheckSimilarity(d.mvClipB, "mvbw");
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    if (d.bleh->nPel == 1)
        d.finest = vsapi->cloneNodeRef(d.super); // v2.0.9.1
    else
    {
        VSPlugin *mvtoolsPlugin = vsapi->getPluginById("com.nodame.mvtools", core);
        VSPlugin *stdPlugin = vsapi->getPluginById("com.vapoursynth.std", core);

        VSMap *args = vsapi->createMap();
        vsapi->propSetNode(args, "super", d.super, paReplace);
        vsapi->propSetInt(args, "isse", d.isse, paReplace);
        VSMap *ret = vsapi->invoke(mvtoolsPlugin, "Finest", args);
        if (vsapi->getError(ret)) {
            vsapi->setError(out, std::string("FlowInter: ").append(vsapi->getError(ret)).c_str());

            delete d.bleh;
            delete d.mvClipB;
            delete d.mvClipF;
            vsapi->freeNode(d.super);
            vsapi->freeNode(d.mvfw);
            vsapi->freeNode(d.mvbw);
            vsapi->freeMap(args);
            vsapi->freeMap(ret);
            return;
        }
        d.finest = vsapi->propGetNode(ret, "clip", 0, NULL);
        vsapi->freeMap(ret);

        vsapi->clearMap(args);
        vsapi->propSetNode(args, "clip", d.finest, paReplace);
        vsapi->freeNode(d.finest);
        ret = vsapi->invoke(stdPlugin, "Cache", args);
        vsapi->freeMap(args);
        if (vsapi->getError(ret)) {
            // prefix the error messages
            vsapi->setError(out, std::string("FlowInter: ").append(vsapi->getError(ret)).c_str());

            delete d.bleh;
            delete d.mvClipB;
            delete d.mvClipF;
            vsapi->freeNode(d.super);
            vsapi->freeNode(d.mvfw);
            vsapi->freeNode(d.mvbw);
            vsapi->freeMap(ret);
            return;
        }
        d.finest = vsapi->propGetNode(ret, "clip", 0, NULL);
        vsapi->freeMap(ret);
    }

    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.bleh->nHeight != nHeightS || d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2) {
        vsapi->setError(out, "FlowInter: wrong source or super clip frame size.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    int id = d.vi->format->id;
    if (!isConstantFormat(d.vi) || (id != pfYUV420P8 && id != pfYUV422P8)) {
        vsapi->setError(out, "FlowInter: input clip must be YUV420P8 or YUV422P8, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }



    d.nBlkXP = (d.bleh->nBlkX * (d.bleh->nBlkSizeX - d.bleh->nOverlapX) + d.bleh->nOverlapX < d.bleh->nWidth) ? d.bleh->nBlkX + 1 : d.bleh->nBlkX;
    d.nBlkYP = (d.bleh->nBlkY * (d.bleh->nBlkSizeY - d.bleh->nOverlapY) + d.bleh->nOverlapY < d.bleh->nHeight) ? d.bleh->nBlkY + 1 : d.bleh->nBlkY;
    d.nWidthP = d.nBlkXP * (d.bleh->nBlkSizeX - d.bleh->nOverlapX) + d.bleh->nOverlapX;
    d.nHeightP = d.nBlkYP * (d.bleh->nBlkSizeY - d.bleh->nOverlapY) + d.bleh->nOverlapY;

    d.nWidthPUV = d.nWidthP / 2;
    d.nHeightPUV = d.nHeightP / d.bleh->yRatioUV;
    d.nHeightUV = d.bleh->nHeight / d.bleh->yRatioUV;
    d.nWidthUV = d.bleh->nWidth / 2;

    d.nHPaddingUV = d.bleh->nHPadding / 2;
    d.nVPaddingUV = d.bleh->nVPadding / d.bleh->yRatioUV;

    d.VPitchY = (d.nWidthP + 15) & (~15);
    d.VPitchUV = (d.nWidthPUV + 15) & (~15);


    d.upsizer = new SimpleResize(d.nWidthP, d.nHeightP, d.nBlkXP, d.nBlkYP);
    d.upsizerUV = new SimpleResize(d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP);


    d.LUTVB = new int[256];
    d.LUTVF = new int[256];
    Create_LUTV(d.time256, d.LUTVB, d.LUTVF);


    data = (MVFlowInterData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "FlowInter", mvflowinterInit, mvflowinterGetFrame, mvflowinterFree, fmParallel, 0, data, core);
}


void mvflowinterRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("FlowInter",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "time:float:opt;"
                 "ml:float:opt;"
                 "blend:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "isse:int:opt;"
                 , mvflowinterCreate, 0, plugin);
}
