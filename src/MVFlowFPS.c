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

#include "Bullshit.h"
#include "MVAnalysisData.h"
#include "CommonFunctions.h"
#include "MaskFun.h"
#include "SimpleResize.h"


typedef struct MVFlowFPSData {
    VSNodeRef *node;
    VSVideoInfo vi;

    VSNodeRef *finest;
    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    int64_t num, den;
    int maskmode;
    double ml;
    int blend;
    int thscd1, thscd2;
    int isse;

    MVAnalysisData mvbw_data;
    MVAnalysisData mvfw_data;

    int nSuperHPad;

    int nWidthUV;
    int nHeightUV;
    int nVPaddingUV;
    int nHPaddingUV;
    int VPitchY;
    int VPitchUV;
    int nWidthP;
    int nHeightP;
    int nWidthPUV;
    int nHeightPUV;
    int nBlkXP;
    int nBlkYP;
    int *LUTVB;
    int *LUTVF;

    uint8_t *VXFullYB;
    uint8_t *VXFullUVB;
    uint8_t *VYFullYB;
    uint8_t *VYFullUVB;
    uint8_t *VXSmallYB;
    uint8_t *VYSmallYB;
    uint8_t *VXSmallUVB;
    uint8_t *VYSmallUVB;
    uint8_t *VXFullYF;
    uint8_t *VXFullUVF;
    uint8_t *VYFullYF;
    uint8_t *VYFullUVF;
    uint8_t *VXSmallYF;
    uint8_t *VYSmallYF;
    uint8_t *VXSmallUVF;
    uint8_t *VYSmallUVF;

    uint8_t *VXFullYBB;
    uint8_t *VXFullUVBB;
    uint8_t *VYFullYBB;
    uint8_t *VYFullUVBB;
    uint8_t *VXSmallYBB;
    uint8_t *VYSmallYBB;
    uint8_t *VXSmallUVBB;
    uint8_t *VYSmallUVBB;
    uint8_t *VXFullYFF;
    uint8_t *VXFullUVFF;
    uint8_t *VYFullYFF;
    uint8_t *VYFullUVFF;
    uint8_t *VXSmallYFF;
    uint8_t *VYSmallYFF;
    uint8_t *VXSmallUVFF;
    uint8_t *VYSmallUVFF;

    uint8_t *MaskSmallB;
    uint8_t *MaskFullYB;
    uint8_t *MaskFullUVB;
    uint8_t *MaskSmallF;
    uint8_t *MaskFullYF;
    uint8_t *MaskFullUVF;

    SimpleResize upsizer;
    SimpleResize upsizerUV;

    int64_t fa, fb;
    int nleftLast, nrightLast;
} MVFlowFPSData;


static void VS_CC mvflowfpsInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVFlowFPSData *d = (MVFlowFPSData *)*instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvflowfpsGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVFlowFPSData *d = (MVFlowFPSData *)*instanceData;

    if (activationReason == arInitial) {
        int off = d->mvbw_data.nDeltaFrame; // integer offset of reference frame

        int nleft = (int)(n * d->fa / d->fb);
        int nright = nleft + off;

        int time256 = (int)(((double)n * d->fa / d->fb - nleft) * 256 + 0.5);
        if (off > 1)
            time256 = time256 / off;

        if (time256 == 0) {
            vsapi->requestFrameFilter(d->vi.numFrames ? VSMIN(nleft, d->vi.numFrames - 1) : nleft, d->node, frameCtx);
            return 0;
        } else if (time256 == 256) {
            vsapi->requestFrameFilter(d->vi.numFrames ? VSMIN(nright, d->vi.numFrames - 1) : nright, d->node, frameCtx);
            return 0;
        }

        if ((nleft < d->vi.numFrames && nright < d->vi.numFrames) || !d->vi.numFrames) { // for the good estimation case
            if (d->maskmode == 2)
                vsapi->requestFrameFilter(nleft, d->mvfw, frameCtx); // requests nleft - off, nleft
            vsapi->requestFrameFilter(nright, d->mvfw, frameCtx);    // requests nleft, nleft + off
            vsapi->requestFrameFilter(nleft, d->mvbw, frameCtx);     // requests nleft, nleft + off
            if (d->maskmode == 2)
                vsapi->requestFrameFilter(nright, d->mvbw, frameCtx); // requests nleft + off, nleft + off + off

            vsapi->requestFrameFilter(nleft, d->finest, frameCtx);
            vsapi->requestFrameFilter(nright, d->finest, frameCtx);
        }

        vsapi->requestFrameFilter(d->vi.numFrames ? VSMIN(nleft, d->vi.numFrames - 1) : nleft, d->node, frameCtx);

        if (d->blend)
            vsapi->requestFrameFilter(d->vi.numFrames ? VSMIN(nright, d->vi.numFrames - 1) : nright, d->node, frameCtx);

    } else if (activationReason == arAllFramesReady) {
        int nleft = (int)(n * d->fa / d->fb);
        // intermediate product may be very large! Now I know how to multiply int64
        int time256 = (int)(((double)n * d->fa / d->fb - nleft) * 256 + 0.5);

        int off = d->mvbw_data.nDeltaFrame; // integer offset of reference frame
        // usually off must be = 1
        if (off > 1)
            time256 = time256 / off;

        int nright = nleft + off;

        if (time256 == 0) {
            return vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nleft, d->vi.numFrames - 1) : nleft, d->node, frameCtx); // simply left
        } else if (time256 == 256) {
            return vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nright, d->vi.numFrames - 1) : nright, d->node, frameCtx); // simply right
        }

        FakeGroupOfPlanes fgopF, fgopB;

        fgopInit(&fgopF, &d->mvfw_data);
        fgopInit(&fgopB, &d->mvbw_data);

        int isUsableF = 0;
        int isUsableB = 0;

        if ((nleft < d->vi.numFrames && nright < d->vi.numFrames) || !d->vi.numFrames) {
            const int *mvs;

            // forward from current to next
            const VSFrameRef *mvF = vsapi->getFrameFilter(nright, d->mvfw, frameCtx);
            mvs = (const int *)vsapi->getReadPtr(mvF, 0);
            fgopUpdate(&fgopF, mvs + mvs[0] / sizeof(int));
            isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);
            vsapi->freeFrame(mvF);

            // backward from next to current
            const VSFrameRef *mvB = vsapi->getFrameFilter(nleft, d->mvbw, frameCtx);
            mvs = (const int *)vsapi->getReadPtr(mvB, 0);
            fgopUpdate(&fgopB, mvs + mvs[0] / sizeof(int));
            isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
            vsapi->freeFrame(mvB);
        }

        const int nWidth = d->mvbw_data.nWidth;
        const int nHeight = d->mvbw_data.nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int maskmode = d->maskmode;
        const int blend = d->blend;
        const int isse = d->isse;
        const double ml = d->ml;
        const int xRatioUV = d->mvbw_data.xRatioUV;
        const int yRatioUV = d->mvbw_data.yRatioUV;
        const int nBlkX = d->mvbw_data.nBlkX;
        const int nBlkY = d->mvbw_data.nBlkY;
        const int nBlkSizeX = d->mvbw_data.nBlkSizeX;
        const int nBlkSizeY = d->mvbw_data.nBlkSizeY;
        const int nOverlapX = d->mvbw_data.nOverlapX;
        const int nOverlapY = d->mvbw_data.nOverlapY;
        const int nVPadding = d->mvbw_data.nVPadding;
        const int nHPadding = d->mvbw_data.nHPadding;
        const int nVPaddingUV = d->nVPaddingUV;
        const int nHPaddingUV = d->nHPaddingUV;
        const int nPel = d->mvbw_data.nPel;
        const int VPitchY = d->VPitchY;
        const int VPitchUV = d->VPitchUV;
        const int nBlkXP = d->nBlkXP;
        const int nBlkYP = d->nBlkYP;
        SimpleResize *upsizer = &d->upsizer;
        SimpleResize *upsizerUV = &d->upsizerUV;
        int *LUTVB = d->LUTVB;
        int *LUTVF = d->LUTVF;

        uint8_t *VXFullYB = d->VXFullYB;
        uint8_t *VXFullUVB = d->VXFullUVB;
        uint8_t *VYFullYB = d->VYFullYB;
        uint8_t *VYFullUVB = d->VYFullUVB;
        uint8_t *VXSmallYB = d->VXSmallYB;
        uint8_t *VYSmallYB = d->VYSmallYB;
        uint8_t *VXSmallUVB = d->VXSmallUVB;
        uint8_t *VYSmallUVB = d->VYSmallUVB;
        uint8_t *VXFullYF = d->VXFullYF;
        uint8_t *VXFullUVF = d->VXFullUVF;
        uint8_t *VYFullYF = d->VYFullYF;
        uint8_t *VYFullUVF = d->VYFullUVF;
        uint8_t *VXSmallYF = d->VXSmallYF;
        uint8_t *VYSmallYF = d->VYSmallYF;
        uint8_t *VXSmallUVF = d->VXSmallUVF;
        uint8_t *VYSmallUVF = d->VYSmallUVF;

        uint8_t *VXFullYBB = d->VXFullYBB;
        uint8_t *VXFullUVBB = d->VXFullUVBB;
        uint8_t *VYFullYBB = d->VYFullYBB;
        uint8_t *VYFullUVBB = d->VYFullUVBB;
        uint8_t *VXSmallYBB = d->VXSmallYBB;
        uint8_t *VYSmallYBB = d->VYSmallYBB;
        uint8_t *VXSmallUVBB = d->VXSmallUVBB;
        uint8_t *VYSmallUVBB = d->VYSmallUVBB;
        uint8_t *VXFullYFF = d->VXFullYFF;
        uint8_t *VXFullUVFF = d->VXFullUVFF;
        uint8_t *VYFullYFF = d->VYFullYFF;
        uint8_t *VYFullUVFF = d->VYFullUVFF;
        uint8_t *VXSmallYFF = d->VXSmallYFF;
        uint8_t *VYSmallYFF = d->VYSmallYFF;
        uint8_t *VXSmallUVFF = d->VXSmallUVFF;
        uint8_t *VYSmallUVFF = d->VYSmallUVFF;

        uint8_t *MaskSmallB = d->MaskSmallB;
        uint8_t *MaskFullYB = d->MaskFullYB;
        uint8_t *MaskFullUVB = d->MaskFullUVB;
        uint8_t *MaskSmallF = d->MaskSmallF;
        uint8_t *MaskFullYF = d->MaskFullYF;
        uint8_t *MaskFullUVF = d->MaskFullUVF;

        int bitsPerSample = d->vi.format->bitsPerSample;
        int bytesPerSample = d->vi.format->bytesPerSample;

        if (isUsableB && isUsableF) {
            uint8_t *pDst[3];
            const uint8_t *pRef[3], *pSrc[3];
            int nDstPitches[3], nRefPitches[3];

            // If both are usable, that means both nleft and nright are less than numFrames, or that we don't have numFrames. Thus there is no need to check nleft and nright here.
            const VSFrameRef *src = vsapi->getFrameFilter(nleft, d->finest, frameCtx);
            const VSFrameRef *ref = vsapi->getFrameFilter(nright, d->finest, frameCtx); //  right frame for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

            Create_LUTV(time256, LUTVB, LUTVF); // lookup table

            for (int i = 0; i < d->vi.format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
            }

            int nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * bytesPerSample * nPel;
            int nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * bytesPerSample * nPel;

            if (nright != d->nrightLast) {
                // make  vector vx and vy small masks
                // 1. ATTENTION: vectors are assumed SHORT (|vx|, |vy| < 127) !
                // 2. they will be zeroed if not
                // 3. added 128 to all values
                MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYB, nBlkXP, VYSmallYB, nBlkXP);
                if (nBlkXP > nBlkX) { // fill right
                    for (int j = 0; j < nBlkY; j++) {
                        VXSmallYB[j * nBlkXP + nBlkX] = VSMIN(VXSmallYB[j * nBlkXP + nBlkX - 1], 128);
                        VYSmallYB[j * nBlkXP + nBlkX] = VYSmallYB[j * nBlkXP + nBlkX - 1];
                    }
                }
                if (nBlkYP > nBlkY) { // fill bottom
                    for (int i = 0; i < nBlkXP; i++) {
                        VXSmallYB[nBlkXP * nBlkY + i] = VXSmallYB[nBlkXP * (nBlkY - 1) + i];
                        VYSmallYB[nBlkXP * nBlkY + i] = VSMIN(VYSmallYB[nBlkXP * (nBlkY - 1) + i], 128);
                    }
                }

                simpleResize(upsizer, VXFullYB, VPitchY, VXSmallYB, nBlkXP);
                simpleResize(upsizer, VYFullYB, VPitchY, VYSmallYB, nBlkXP);

                if (d->vi.format->colorFamily != cmGray) {
                    VectorSmallMaskYToHalfUV(VXSmallYB, nBlkXP, nBlkYP, VXSmallUVB, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYB, nBlkXP, nBlkYP, VYSmallUVB, yRatioUV);

                    simpleResize(upsizerUV, VXFullUVB, VPitchUV, VXSmallUVB, nBlkXP);
                    simpleResize(upsizerUV, VYFullUVB, VPitchUV, VYSmallUVB, nBlkXP);
                }
            }
            // analyse vectors field to detect occlusion
            //        double occNormB = (256-time256)/(256*ml);
            //        MakeVectorOcclusionMask(mvClipB, nBlkX, nBlkY, occNormB, 1.0, nPel, MaskSmallB, nBlkXP);
            MakeVectorOcclusionMaskTime(&fgopB, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallB, nBlkXP, (256 - time256), nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            if (nBlkXP > nBlkX) // fill right
                for (int j = 0; j < nBlkY; j++)
                    MaskSmallB[j * nBlkXP + nBlkX] = MaskSmallB[j * nBlkXP + nBlkX - 1];
            if (nBlkYP > nBlkY) // fill bottom
                for (int i = 0; i < nBlkXP; i++)
                    MaskSmallB[nBlkXP * nBlkY + i] = MaskSmallB[nBlkXP * (nBlkY - 1) + i];

            simpleResize(upsizer, MaskFullYB, VPitchY, MaskSmallB, nBlkXP);
            if (d->vi.format->colorFamily != cmGray)
                simpleResize(upsizerUV, MaskFullUVB, VPitchUV, MaskSmallB, nBlkXP);

            d->nrightLast = nright;

            if (nleft != d->nleftLast) {
                // make  vector vx and vy small masks
                // 1. ATTENTION: vectors are assumed SHORT (|vx|, |vy| < 127) !
                // 2. they will be zeroed if not
                // 3. added 128 to all values
                MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYF, nBlkXP, VYSmallYF, nBlkXP);
                if (nBlkXP > nBlkX) { // fill right
                    for (int j = 0; j < nBlkY; j++) {
                        VXSmallYF[j * nBlkXP + nBlkX] = VSMIN(VXSmallYF[j * nBlkXP + nBlkX - 1], 128);
                        VYSmallYF[j * nBlkXP + nBlkX] = VYSmallYF[j * nBlkXP + nBlkX - 1];
                    }
                }
                if (nBlkYP > nBlkY) { // fill bottom
                    for (int i = 0; i < nBlkXP; i++) {
                        VXSmallYF[nBlkXP * nBlkY + i] = VXSmallYF[nBlkXP * (nBlkY - 1) + i];
                        VYSmallYF[nBlkXP * nBlkY + i] = VSMIN(VYSmallYF[nBlkXP * (nBlkY - 1) + i], 128);
                    }
                }

                simpleResize(upsizer, VXFullYF, VPitchY, VXSmallYF, nBlkXP);
                simpleResize(upsizer, VYFullYF, VPitchY, VYSmallYF, nBlkXP);

                if (d->vi.format->colorFamily != cmGray) {
                    VectorSmallMaskYToHalfUV(VXSmallYF, nBlkXP, nBlkYP, VXSmallUVF, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYF, nBlkXP, nBlkYP, VYSmallUVF, yRatioUV);

                    simpleResize(upsizerUV, VXFullUVF, VPitchUV, VXSmallUVF, nBlkXP);
                    simpleResize(upsizerUV, VYFullUVF, VPitchUV, VYSmallUVF, nBlkXP);
                }
            }
            // analyse vectors field to detect occlusion
            //        double occNormF = time256/(256*ml);
            //        MakeVectorOcclusionMask(mvClipF, nBlkX, nBlkY, occNormF, 1.0, nPel, MaskSmallF, nBlkXP);
            MakeVectorOcclusionMaskTime(&fgopF, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallF, nBlkXP, time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            if (nBlkXP > nBlkX) // fill right
                for (int j = 0; j < nBlkY; j++)
                    MaskSmallF[j * nBlkXP + nBlkX] = MaskSmallF[j * nBlkXP + nBlkX - 1];
            if (nBlkYP > nBlkY) // fill bottom
                for (int i = 0; i < nBlkXP; i++)
                    MaskSmallF[nBlkXP * nBlkY + i] = MaskSmallF[nBlkXP * (nBlkY - 1) + i];

            simpleResize(upsizer, MaskFullYF, VPitchY, MaskSmallF, nBlkXP);
            if (d->vi.format->colorFamily != cmGray)
                simpleResize(upsizerUV, MaskFullUVF, VPitchUV, MaskSmallF, nBlkXP);

            d->nleftLast = nleft;

            if (maskmode == 2) { // These motion vectors should only be needed with maskmode 2. Why was the Avisynth plugin requesting them for all mask modes?
                // Get motion info from more frames for occlusion areas
                const int *mvs;

                // forward from previous to current
                const VSFrameRef *mvFF = vsapi->getFrameFilter(nleft, d->mvfw, frameCtx);
                mvs = (const int *)vsapi->getReadPtr(mvFF, 0);
                fgopUpdate(&fgopF, mvs + mvs[0] / sizeof(int));
                isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvFF);

                // backward from next next to next
                const VSFrameRef *mvBB = vsapi->getFrameFilter(nright, d->mvbw, frameCtx);
                mvs = (const int *)vsapi->getReadPtr(mvBB, 0);
                fgopUpdate(&fgopB, mvs + mvs[0] / sizeof(int));
                isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvBB);
            }

            if (maskmode == 2 && isUsableB && isUsableF) { // slow method with extra frames
                // get vector mask from extra frames
                MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYBB, nBlkXP, VYSmallYBB, nBlkXP);
                MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYFF, nBlkXP, VYSmallYFF, nBlkXP);
                if (nBlkXP > nBlkX) { // fill right
                    for (int j = 0; j < nBlkY; j++) {
                        VXSmallYBB[j * nBlkXP + nBlkX] = VSMIN(VXSmallYBB[j * nBlkXP + nBlkX - 1], 128);
                        VYSmallYBB[j * nBlkXP + nBlkX] = VYSmallYBB[j * nBlkXP + nBlkX - 1];
                        VXSmallYFF[j * nBlkXP + nBlkX] = VSMIN(VXSmallYFF[j * nBlkXP + nBlkX - 1], 128);
                        VYSmallYFF[j * nBlkXP + nBlkX] = VYSmallYFF[j * nBlkXP + nBlkX - 1];
                    }
                }
                if (nBlkYP > nBlkY) { // fill bottom
                    for (int i = 0; i < nBlkXP; i++) {
                        VXSmallYBB[nBlkXP * nBlkY + i] = VXSmallYBB[nBlkXP * (nBlkY - 1) + i];
                        VYSmallYBB[nBlkXP * nBlkY + i] = VSMIN(VYSmallYBB[nBlkXP * (nBlkY - 1) + i], 128);
                        VXSmallYFF[nBlkXP * nBlkY + i] = VXSmallYFF[nBlkXP * (nBlkY - 1) + i];
                        VYSmallYFF[nBlkXP * nBlkY + i] = VSMIN(VYSmallYFF[nBlkXP * (nBlkY - 1) + i], 128);
                    }
                }

                simpleResize(upsizer, VXFullYBB, VPitchY, VXSmallYBB, nBlkXP);
                simpleResize(upsizer, VYFullYBB, VPitchY, VYSmallYBB, nBlkXP);

                simpleResize(upsizer, VXFullYFF, VPitchY, VXSmallYFF, nBlkXP);
                simpleResize(upsizer, VYFullYFF, VPitchY, VYSmallYFF, nBlkXP);

                FlowInterExtra(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                               VXFullYB, VXFullYF, VYFullYB, VYFullYF, MaskFullYB, MaskFullYF, VPitchY,
                               nWidth, nHeight, time256, nPel, LUTVB, LUTVF, VXFullYBB, VXFullYFF, VYFullYBB, VYFullYFF, bitsPerSample);
                if (d->vi.format->colorFamily != cmGray) {
                    VectorSmallMaskYToHalfUV(VXSmallYBB, nBlkXP, nBlkYP, VXSmallUVBB, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYBB, nBlkXP, nBlkYP, VYSmallUVBB, yRatioUV);
                    VectorSmallMaskYToHalfUV(VXSmallYFF, nBlkXP, nBlkYP, VXSmallUVFF, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYFF, nBlkXP, nBlkYP, VYSmallUVFF, yRatioUV);

                    simpleResize(upsizerUV, VXFullUVBB, VPitchUV, VXSmallUVBB, nBlkXP);
                    simpleResize(upsizerUV, VYFullUVBB, VPitchUV, VYSmallUVBB, nBlkXP);

                    simpleResize(upsizerUV, VXFullUVFF, VPitchUV, VXSmallUVFF, nBlkXP);
                    simpleResize(upsizerUV, VYFullUVFF, VPitchUV, VYSmallUVFF, nBlkXP);

                    FlowInterExtra(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                                   VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                                   nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF, bitsPerSample);
                    FlowInterExtra(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                                   VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                                   nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF, bitsPerSample);
                }
            } else if (maskmode == 1) { // old method without extra frames
                FlowInter(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                          VXFullYB, VXFullYF, VYFullYB, VYFullYF, MaskFullYB, MaskFullYF, VPitchY,
                          nWidth, nHeight, time256, nPel, LUTVB, LUTVF, bitsPerSample);
                if (d->vi.format->colorFamily != cmGray) {
                    FlowInter(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                              VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                              nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, bitsPerSample);
                    FlowInter(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                              VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                              nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, bitsPerSample);
                }
            } else { // mode=0, faster simple method
                FlowInterSimple(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                                VXFullYB, VXFullYF, VYFullYB, VYFullYF, MaskFullYB, MaskFullYF, VPitchY,
                                nWidth, nHeight, time256, nPel, LUTVB, LUTVF, bitsPerSample);
                if (d->vi.format->colorFamily != cmGray) {
                    FlowInterSimple(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                                    VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                                    nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, bitsPerSample);
                    FlowInterSimple(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                                    VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, MaskFullUVB, MaskFullUVF, VPitchUV,
                                    nWidthUV, nHeightUV, time256, nPel, LUTVB, LUTVF, bitsPerSample);
                }
            }

            vsapi->freeFrame(src);
            vsapi->freeFrame(ref);

            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            return dst;
        } else { // poor estimation
            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            const VSFrameRef *src = vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nleft, d->vi.numFrames - 1) : nleft, d->node, frameCtx);

            if (blend) { //let's blend src with ref frames like ConvertFPS
                uint8_t *pDst[3];
                const uint8_t *pRef[3], *pSrc[3];
                int nDstPitches[3], nRefPitches[3], nSrcPitches[3];

                const VSFrameRef *ref = vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nright, d->vi.numFrames - 1) : nright, d->node, frameCtx);

                VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

                for (int i = 0; i < d->vi.format->numPlanes; i++) {
                    pDst[i] = vsapi->getWritePtr(dst, i);
                    pRef[i] = vsapi->getReadPtr(ref, i);
                    pSrc[i] = vsapi->getReadPtr(src, i);
                    nDstPitches[i] = vsapi->getStride(dst, i);
                    nRefPitches[i] = vsapi->getStride(ref, i);
                    nSrcPitches[i] = vsapi->getStride(src, i);
                }

                // blend with time weight
                Blend(pDst[0], pSrc[0], pRef[0], nHeight, nWidth, nDstPitches[0], nSrcPitches[0], nRefPitches[0], time256, isse, bitsPerSample);
                if (d->vi.format->colorFamily != cmGray) {
                    Blend(pDst[1], pSrc[1], pRef[1], nHeightUV, nWidthUV, nDstPitches[1], nSrcPitches[1], nRefPitches[1], time256, isse, bitsPerSample);
                    Blend(pDst[2], pSrc[2], pRef[2], nHeightUV, nWidthUV, nDstPitches[2], nSrcPitches[2], nRefPitches[2], time256, isse, bitsPerSample);
                }

                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);

                return dst;
            } else {
                return src; // like ChangeFPS
            }
        }
    }

    return 0;
}


static void VS_CC mvflowfpsFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVFlowFPSData *d = (MVFlowFPSData *)instanceData;

    free(d->VXFullYB);
    free(d->VYFullYB);
    free(d->VXSmallYB);
    free(d->VYSmallYB);
    free(d->VXFullYF);
    free(d->VYFullYF);
    free(d->VXSmallYF);
    free(d->VYSmallYF);

    if (d->maskmode == 2) {
        free(d->VXFullYBB);
        free(d->VYFullYBB);
        free(d->VXSmallYBB);
        free(d->VYSmallYBB);
        free(d->VXFullYFF);
        free(d->VYFullYFF);
        free(d->VXSmallYFF);
        free(d->VYSmallYFF);
    }

    free(d->MaskSmallB);
    free(d->MaskFullYB);
    free(d->MaskSmallF);
    free(d->MaskFullYF);

    if (d->vi.format->colorFamily != cmGray) {
        free(d->VXFullUVB);
        free(d->VYFullUVB);
        free(d->VXSmallUVB);
        free(d->VYSmallUVB);
        free(d->VXFullUVF);
        free(d->VYFullUVF);
        free(d->VXSmallUVF);
        free(d->VYSmallUVF);

        if (d->maskmode == 2) {
            free(d->VXFullUVBB);
            free(d->VYFullUVBB);
            free(d->VXSmallUVBB);
            free(d->VYSmallUVBB);
            free(d->VXFullUVFF);
            free(d->VYFullUVFF);
            free(d->VXSmallUVFF);
            free(d->VYSmallUVFF);
        }

        free(d->MaskFullUVB);
        free(d->MaskFullUVF);

        simpleDeinit(&d->upsizerUV);
    }

    free(d->LUTVB);
    free(d->LUTVF);

    simpleDeinit(&d->upsizer);

    vsapi->freeNode(d->finest);
    vsapi->freeNode(d->super);
    vsapi->freeNode(d->mvfw);
    vsapi->freeNode(d->mvbw);
    vsapi->freeNode(d->node);
    free(d);
}


static inline void setFPS(VSVideoInfo *vi, int64_t num, int64_t den) {
    if (num <= 0 || den <= 0) {
        vi->fpsNum = 0;
        vi->fpsDen = 1;
    } else {
        int64_t x = num;
        int64_t y = den;
        while (y) {
            int64_t t = x % y;
            x = y;
            y = t;
        }
        vi->fpsNum = num / x;
        vi->fpsDen = den / x;
    }
}


static void VS_CC mvflowfpsCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVFlowFPSData d;
    MVFlowFPSData *data;

    int err;

    d.num = vsapi->propGetInt(in, "num", 0, &err);
    if (err)
        d.num = 25;

    d.den = vsapi->propGetInt(in, "den", 0, &err);
    if (err)
        d.den = 1;

    d.maskmode = int64ToIntS(vsapi->propGetInt(in, "mask", 0, &err));
    if (err)
        d.maskmode = 2;

    d.ml = vsapi->propGetFloat(in, "ml", 0, &err);
    if (err)
        d.ml = 100.0;

    d.blend = !!vsapi->propGetInt(in, "blend", 0, &err);
    if (err)
        d.blend = 1;

    d.thscd1 = int64ToIntS(vsapi->propGetInt(in, "thscd1", 0, &err));
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.isse = !!vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;


    if (d.maskmode < 0 || d.maskmode > 2) {
        vsapi->setError(out, "FlowFPS: mask must be 0, 1, or 2.");
        return;
    }

    if (d.ml <= 0.0) {
        vsapi->setError(out, "FlowFPS: ml must be greater than 0.");
        return;
    }


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "FlowFPS: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg + errorLen, ERROR_SIZE - errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->setError(out, errorMsg);
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[2];
    int nHeightS = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    vsapi->freeFrame(evil);

    for (int i = 0; i < 2; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "FlowFPS: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);

    {
#define ERROR_SIZE 512
        char error[ERROR_SIZE + 1] = { 0 };
        const char *filter_name = "FlowFPS";

        adataFromVectorClip(&d.mvbw_data, d.mvbw, filter_name, "mvbw", vsapi, error, ERROR_SIZE);
        adataFromVectorClip(&d.mvfw_data, d.mvfw, filter_name, "mvfw", vsapi, error, ERROR_SIZE);

        scaleThSCD(&d.thscd1, &d.thscd2, &d.mvbw_data, filter_name, error, ERROR_SIZE);

        adataCheckSimilarity(&d.mvbw_data, &d.mvfw_data, filter_name, "mvbw", "mvfw", error, ERROR_SIZE);
#undef ERROR_SIZE

        if (error[0]) {
            vsapi->setError(out, error);

            vsapi->freeNode(d.super);
            vsapi->freeNode(d.mvfw);
            vsapi->freeNode(d.mvbw);
            return;
        }
    }


    // XXX Alternatively, use both clips' delta as offsets in GetFrame.
    if (d.mvbw_data.nDeltaFrame != d.mvfw_data.nDeltaFrame) {
        vsapi->setError(out, "FlowFPS: mvbw and mvfw must be generated with the same delta.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    // Make sure the motion vector clips are correct.
    if (!d.mvbw_data.isBackward || d.mvfw_data.isBackward) {
        if (!d.mvbw_data.isBackward)
            vsapi->setError(out, "FlowFPS: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "FlowFPS: mvfw must be generated with isb=False.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }


    if (d.mvbw_data.nPel == 1)
        d.finest = vsapi->cloneNodeRef(d.super); // v2.0.9.1
    else {
        VSPlugin *mvtoolsPlugin = vsapi->getPluginById("com.nodame.mvtools", core);
        VSPlugin *stdPlugin = vsapi->getPluginById("com.vapoursynth.std", core);

        VSMap *args = vsapi->createMap();
        vsapi->propSetNode(args, "super", d.super, paReplace);
        vsapi->propSetInt(args, "isse", d.isse, paReplace);
        VSMap *ret = vsapi->invoke(mvtoolsPlugin, "Finest", args);
        if (vsapi->getError(ret)) {
#define ERROR_SIZE 512
            char error_msg[ERROR_SIZE + 1] = { 0 };
            snprintf(error_msg, ERROR_SIZE, "FlowFPS: %s", vsapi->getError(ret));
#undef ERROR_SIZE
            vsapi->setError(out, error_msg);

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
#define ERROR_SIZE 512
            char error_msg[ERROR_SIZE + 1] = { 0 };
            snprintf(error_msg, ERROR_SIZE, "FlowFPS: %s", vsapi->getError(ret));
#undef ERROR_SIZE
            vsapi->setError(out, error_msg);

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
    d.vi = *vsapi->getVideoInfo(d.node);


    if (d.vi.fpsNum == 0 || d.vi.fpsDen == 0) {
        vsapi->setError(out, "FlowFPS: The input clip must have a frame rate. Invoke AssumeFPS if necessary.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }

    int64_t numeratorOld = d.vi.fpsNum;
    int64_t denominatorOld = d.vi.fpsDen;
    int64_t numerator, denominator;

    if (d.num != 0 && d.den != 0) {
        numerator = d.num;
        denominator = d.den;
    } else {
        numerator = numeratorOld * 2; // double fps by default
        denominator = denominatorOld;
    }

    //  safe for big numbers since v2.1
    d.fa = denominator * numeratorOld;
    d.fb = numerator * denominatorOld;
    int64_t fgcd = gcd(d.fa, d.fb); // general common divisor
    d.fa /= fgcd;
    d.fb /= fgcd;

    setFPS(&d.vi, numerator, denominator);

    if (d.vi.numFrames)
        d.vi.numFrames = (int)(1 + (d.vi.numFrames - 1) * d.fb / d.fa);


    if (d.mvbw_data.nWidth != d.vi.width || d.mvbw_data.nHeight != d.vi.height) {
        vsapi->setError(out, "FlowFPS: inconsistent source and vector frame size.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }


    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.mvbw_data.nHeight != nHeightS || d.mvbw_data.nWidth != nSuperWidth - d.nSuperHPad * 2) {
        vsapi->setError(out, "FlowFPS: wrong source or super clip frame size.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }

    if (!((d.mvbw_data.nWidth + d.mvbw_data.nHPadding * 2) == supervi->width && (d.mvbw_data.nHeight + d.mvbw_data.nVPadding * 2) <= supervi->height)) {
        vsapi->setError(out, "FlowFPS: inconsistent clips frame size! Incomprehensible error messages are the best, right?");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }

    if (!isConstantFormat(&d.vi) || d.vi.format->bitsPerSample > 16 || d.vi.format->sampleType != stInteger || d.vi.format->subSamplingW > 1 || d.vi.format->subSamplingH > 1 || (d.vi.format->colorFamily != cmYUV && d.vi.format->colorFamily != cmGray)) {
        vsapi->setError(out, "FlowFPS: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }

    if (d.vi.format->bitsPerSample > 8)
        d.isse = 0;


    d.nBlkXP = (d.mvbw_data.nBlkX * (d.mvbw_data.nBlkSizeX - d.mvbw_data.nOverlapX) + d.mvbw_data.nOverlapX < d.mvbw_data.nWidth) ? d.mvbw_data.nBlkX + 1 : d.mvbw_data.nBlkX;
    d.nBlkYP = (d.mvbw_data.nBlkY * (d.mvbw_data.nBlkSizeY - d.mvbw_data.nOverlapY) + d.mvbw_data.nOverlapY < d.mvbw_data.nHeight) ? d.mvbw_data.nBlkY + 1 : d.mvbw_data.nBlkY;
    d.nWidthP = d.nBlkXP * (d.mvbw_data.nBlkSizeX - d.mvbw_data.nOverlapX) + d.mvbw_data.nOverlapX;
    d.nHeightP = d.nBlkYP * (d.mvbw_data.nBlkSizeY - d.mvbw_data.nOverlapY) + d.mvbw_data.nOverlapY;

    d.nWidthPUV = d.nWidthP / d.mvbw_data.xRatioUV;
    d.nHeightPUV = d.nHeightP / d.mvbw_data.yRatioUV;
    d.nHeightUV = d.mvbw_data.nHeight / d.mvbw_data.yRatioUV;
    d.nWidthUV = d.mvbw_data.nWidth / d.mvbw_data.xRatioUV;

    d.nHPaddingUV = d.mvbw_data.nHPadding / d.mvbw_data.xRatioUV;
    d.nVPaddingUV = d.mvbw_data.nVPadding / d.mvbw_data.yRatioUV;

    d.VPitchY = (d.nWidthP + 15) & (~15);
    d.VPitchUV = (d.nWidthPUV + 15) & (~15);


    d.VXFullYB = (uint8_t *)malloc(d.nHeightP * d.VPitchY);
    d.VYFullYB = (uint8_t *)malloc(d.nHeightP * d.VPitchY);

    d.VXFullYF = (uint8_t *)malloc(d.nHeightP * d.VPitchY);
    d.VYFullYF = (uint8_t *)malloc(d.nHeightP * d.VPitchY);

    d.VXSmallYB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
    d.VYSmallYB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);

    d.VXSmallYF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
    d.VYSmallYF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);

    if (d.maskmode == 2) {
        d.VXFullYBB = (uint8_t *)malloc(d.nHeightP * d.VPitchY);
        d.VYFullYBB = (uint8_t *)malloc(d.nHeightP * d.VPitchY);

        d.VXFullYFF = (uint8_t *)malloc(d.nHeightP * d.VPitchY);
        d.VYFullYFF = (uint8_t *)malloc(d.nHeightP * d.VPitchY);

        d.VXSmallYBB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
        d.VYSmallYBB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);

        d.VXSmallYFF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
        d.VYSmallYFF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
    }

    d.MaskSmallB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
    d.MaskFullYB = (uint8_t *)malloc(d.nHeightP * d.VPitchY);

    d.MaskSmallF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
    d.MaskFullYF = (uint8_t *)malloc(d.nHeightP * d.VPitchY);

    simpleInit(&d.upsizer, d.nWidthP, d.nHeightP, d.nBlkXP, d.nBlkYP);

    if (d.vi.format->colorFamily != cmGray) {
        d.VXFullUVB = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
        d.VYFullUVB = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
        d.VXFullUVF = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
        d.VYFullUVF = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
        d.VXSmallUVB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
        d.VYSmallUVB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
        d.VXSmallUVF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
        d.VYSmallUVF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);

        if (d.maskmode == 2) {
            d.VXFullUVBB = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
            d.VYFullUVBB = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
            d.VXFullUVFF = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
            d.VYFullUVFF = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
            d.VXSmallUVBB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
            d.VYSmallUVBB = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
            d.VXSmallUVFF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
            d.VYSmallUVFF = (uint8_t *)malloc(d.nBlkXP * d.nBlkYP);
        }

        d.MaskFullUVB = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);
        d.MaskFullUVF = (uint8_t *)malloc(d.nHeightPUV * d.VPitchUV);

        simpleInit(&d.upsizerUV, d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP);
    }


    d.LUTVB = (int *)malloc(256 * sizeof(int));
    d.LUTVF = (int *)malloc(256 * sizeof(int));

    d.nleftLast = -1000;
    d.nrightLast = -1000;


    data = (MVFlowFPSData *)malloc(sizeof(d));
    *data = d;

    // Can't use fmParallel because of nleftLast/nrightLast.
    vsapi->createFilter(in, out, "FlowFPS", mvflowfpsInit, mvflowfpsGetFrame, mvflowfpsFree, fmParallelRequests, 0, data, core);

    // AssumeFPS sets the _DurationNum and _DurationDen properties.
    VSNodeRef *node = vsapi->propGetNode(out, "clip", 0, NULL);
    VSMap *args = vsapi->createMap();
    vsapi->propSetNode(args, "clip", node, paReplace);
    vsapi->freeNode(node);
    vsapi->propSetInt(args, "fpsnum", d.vi.fpsNum, paReplace);
    vsapi->propSetInt(args, "fpsden", d.vi.fpsDen, paReplace);
    VSPlugin *stdPlugin = vsapi->getPluginById("com.vapoursynth.std", core);
    VSMap *ret = vsapi->invoke(stdPlugin, "AssumeFPS", args);
    if (vsapi->getError(ret)) {
#define ERROR_SIZE 512
        char error_msg[ERROR_SIZE + 1] = { 0 };
        snprintf(error_msg, ERROR_SIZE, "FlowFPS: Failed to invoke AssumeFPS. Error message: %s", vsapi->getError(ret));
#undef ERROR_SIZE
        vsapi->setError(out, error_msg);

        vsapi->freeMap(args);
        vsapi->freeMap(ret);
        return;
    }
    node = vsapi->propGetNode(ret, "clip", 0, NULL);
    vsapi->freeMap(ret);
    vsapi->clearMap(args);
    vsapi->propSetNode(args, "clip", node, paReplace);
    vsapi->freeNode(node);
    ret = vsapi->invoke(stdPlugin, "Cache", args);
    vsapi->freeMap(args);
    if (vsapi->getError(ret)) {
#define ERROR_SIZE 512
        char error_msg[ERROR_SIZE + 1] = { 0 };
        snprintf(error_msg, ERROR_SIZE, "FlowFPS: Failed to invoke Cache. Error message: %s", vsapi->getError(ret));
#undef ERROR_SIZE
        vsapi->setError(out, error_msg);

        vsapi->freeMap(ret);
        return;
    }
    node = vsapi->propGetNode(ret, "clip", 0, NULL);
    vsapi->freeMap(ret);
    vsapi->propSetNode(out, "clip", node, paReplace);
    vsapi->freeNode(node);
}


void mvflowfpsRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("FlowFPS",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "num:int:opt;"
                 "den:int:opt;"
                 "mask:int:opt;"
                 "ml:float:opt;"
                 "blend:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "isse:int:opt;",
                 mvflowfpsCreate, 0, plugin);
}
