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

#include "MVFlowFPSHelper.h"


typedef struct MVFlowFPSData {
    VSNodeRef *node;
    VSVideoInfo vi;
    const VSVideoInfo *oldvi;

    VSNodeRef *finest;
    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    int64_t num, den;
    int maskmode;
    double ml;
    int blend;
    int64_t thscd1;
    int thscd2;
    int opt;

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

    SimpleResize upsizer;
    SimpleResize upsizerUV;

    int64_t fa, fb;

    FlowInterSimpleFunction FlowInterSimple;
    FlowInterFunction FlowInter;
    FlowInterExtraFunction FlowInterExtra;
} MVFlowFPSData;


static void VS_CC mvflowfpsInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVFlowFPSData *d = (MVFlowFPSData *)*instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvflowfpsGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    const MVFlowFPSData *d = (const MVFlowFPSData *)*instanceData;

    if (activationReason == arInitial) {
        int off = d->mvbw_data.nDeltaFrame; // integer offset of reference frame

        int nleft = (int)(n * d->fa / d->fb);
        int nright = nleft + off;

        int time256 = (int)(((double)n * d->fa / d->fb - nleft) * 256 + 0.5);
        if (off > 1)
            time256 = time256 / off;

        if (time256 == 0) {
            vsapi->requestFrameFilter(VSMIN(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);
            return 0;
        } else if (time256 == 256) {
            vsapi->requestFrameFilter(VSMIN(nright, d->oldvi->numFrames - 1), d->node, frameCtx);
            return 0;
        }

        if (nleft < d->oldvi->numFrames && nright < d->oldvi->numFrames) { // for the good estimation case
            if (d->maskmode == 2)
                vsapi->requestFrameFilter(nleft, d->mvfw, frameCtx); // requests nleft - off, nleft
            vsapi->requestFrameFilter(nright, d->mvfw, frameCtx);    // requests nleft, nleft + off
            vsapi->requestFrameFilter(nleft, d->mvbw, frameCtx);     // requests nleft, nleft + off
            if (d->maskmode == 2)
                vsapi->requestFrameFilter(nright, d->mvbw, frameCtx); // requests nleft + off, nleft + off + off

            vsapi->requestFrameFilter(nleft, d->finest, frameCtx);
            vsapi->requestFrameFilter(nright, d->finest, frameCtx);
        }

        vsapi->requestFrameFilter(VSMIN(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);

        if (d->blend)
            vsapi->requestFrameFilter(VSMIN(nright, d->oldvi->numFrames - 1), d->node, frameCtx);

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
            return vsapi->getFrameFilter(VSMIN(nleft, d->oldvi->numFrames - 1), d->node, frameCtx); // simply left
        } else if (time256 == 256) {
            return vsapi->getFrameFilter(VSMIN(nright, d->oldvi->numFrames - 1), d->node, frameCtx); // simply right
        }

        FakeGroupOfPlanes fgopF, fgopB;

        fgopInit(&fgopF, &d->mvfw_data);
        fgopInit(&fgopB, &d->mvbw_data);

        int isUsableF = 0;
        int isUsableB = 0;

        const VSFrameRef *mvF = NULL, *mvB = NULL;

        if (nleft < d->oldvi->numFrames && nright < d->oldvi->numFrames) {
            // forward from current to next
            mvF = vsapi->getFrameFilter(nright, d->mvfw, frameCtx);
            const VSMap *mvprops = vsapi->getFramePropsRO(mvF);
            fgopUpdate(&fgopF, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);

            // backward from next to current
            mvB = vsapi->getFrameFilter(nleft, d->mvbw, frameCtx);
            mvprops = vsapi->getFramePropsRO(mvB);
            fgopUpdate(&fgopB, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
        }

        const int nWidth = d->mvbw_data.nWidth;
        const int nHeight = d->mvbw_data.nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int nHeightP = d->nHeightP;
        const int nHeightPUV = d->nHeightPUV;
        const int maskmode = d->maskmode;
        const int blend = d->blend;
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
        const SimpleResize *upsizer = &d->upsizer;
        const SimpleResize *upsizerUV = &d->upsizerUV;

        int bitsPerSample = d->vi.format->bitsPerSample;
        int bytesPerSample = d->vi.format->bytesPerSample;

        if (isUsableB && isUsableF) {
            uint8_t *pDst[3] = { NULL };
            const uint8_t *pRef[3] = { NULL };
            const uint8_t *pSrc[3] = { NULL };
            int nDstPitches[3] = { 0 };
            int nRefPitches[3] = { 0 };

            // Put this before any allocations so we don't have to free much in case of error.
            const VSMap *props = vsapi->getFramePropsRO(mvB);
            int err[8] = { 0 };
            const int16_t *VXFullYB = (const int16_t *)vsapi->propGetData(props, prop_VXFullY, 0, &err[0]);
            const int16_t *VYFullYB = (const int16_t *)vsapi->propGetData(props, prop_VYFullY, 0, &err[1]);
            const int16_t *VXFullUVB = NULL;
            const int16_t *VYFullUVB = NULL;
            if (d->vi.format->colorFamily != cmGray) {
                VXFullUVB = (const int16_t *)vsapi->propGetData(props, prop_VXFullUV, 0, &err[2]);
                VYFullUVB = (const int16_t *)vsapi->propGetData(props, prop_VYFullUV, 0, &err[3]);
            }

            props = vsapi->getFramePropsRO(mvF);
            const int16_t *VXFullYF = (const int16_t *)vsapi->propGetData(props, prop_VXFullY, 0, &err[4]);
            const int16_t *VYFullYF = (const int16_t *)vsapi->propGetData(props, prop_VYFullY, 0, &err[5]);
            const int16_t *VXFullUVF = NULL;
            const int16_t *VYFullUVF = NULL;
            if (d->vi.format->colorFamily != cmGray) {
                VXFullUVF = (const int16_t *)vsapi->propGetData(props, prop_VXFullUV, 0, &err[6]);
                VYFullUVF = (const int16_t *)vsapi->propGetData(props, prop_VYFullUV, 0, &err[7]);
            }
            for (int i = 0; i < 8; i++) {
                if (err[i]) {
                    vsapi->freeFrame(mvB);
                    vsapi->freeFrame(mvF);

                    fgopDeinit(&fgopB);
                    fgopDeinit(&fgopF);

                    vsapi->setFilterError("FlowFPS: helper filter did not set the expected frame properties.", frameCtx);

                    return NULL;
                }
            }

            // If both are usable, that means both nleft and nright are less than oldvi->numFrames. Thus there is no need to check nleft and nright here.
            const VSFrameRef *src = vsapi->getFrameFilter(nleft, d->finest, frameCtx);
            const VSFrameRef *ref = vsapi->getFrameFilter(nright, d->finest, frameCtx); //  right frame for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

            for (int i = 0; i < d->vi.format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
            }

            int16_t *VXFullYBB = NULL;
            int16_t *VXFullUVBB = NULL;
            int16_t *VYFullYBB = NULL;
            int16_t *VYFullUVBB = NULL;
            int16_t *VXSmallYBB = NULL;
            int16_t *VYSmallYBB = NULL;
            int16_t *VXSmallUVBB = NULL;
            int16_t *VYSmallUVBB = NULL;
            int16_t *VXFullYFF = NULL;
            int16_t *VXFullUVFF = NULL;
            int16_t *VYFullYFF = NULL;
            int16_t *VYFullUVFF = NULL;
            int16_t *VXSmallYFF = NULL;
            int16_t *VYSmallYFF = NULL;
            int16_t *VXSmallUVFF = NULL;
            int16_t *VYSmallUVFF = NULL;

            uint8_t *MaskFullUVB = NULL;
            uint8_t *MaskFullUVF = NULL;

            if (maskmode == 2) {
                VXFullYBB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                VYFullYBB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));

                VXFullYFF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                VYFullYFF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));

                VXSmallYBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VYSmallYBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));

                VXSmallYFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VYSmallYFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            }

            uint8_t *MaskSmallB = (uint8_t *)malloc(nBlkXP * nBlkYP);
            uint8_t *MaskFullYB = (uint8_t *)malloc(nHeightP * VPitchY);

            uint8_t *MaskSmallF = (uint8_t *)malloc(nBlkXP * nBlkYP);
            uint8_t *MaskFullYF = (uint8_t *)malloc(nHeightP * VPitchY);

            if (d->vi.format->colorFamily != cmGray) {
                if (maskmode == 2) {
                    VXFullUVBB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    VYFullUVBB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    VXFullUVFF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    VYFullUVFF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    VXSmallUVBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    VYSmallUVBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    VXSmallUVFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    VYSmallUVFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                }

                MaskFullUVB = (uint8_t *)malloc(nHeightPUV * VPitchUV);
                MaskFullUVF = (uint8_t *)malloc(nHeightPUV * VPitchUV);
            }

            // analyse vectors field to detect occlusion
            //        double occNormB = (256-time256)/(256*ml);
            //        MakeVectorOcclusionMask(mvClipB, nBlkX, nBlkY, occNormB, 1.0, nPel, MaskSmallB, nBlkXP);
            MakeVectorOcclusionMaskTime(&fgopB, 1, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallB, nBlkXP, (256 - time256), nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);

            CheckAndPadMaskSmall(MaskSmallB, nBlkXP, nBlkYP, nBlkX, nBlkY);

            upsizer->simpleResize_uint8_t(upsizer, MaskFullYB, VPitchY, MaskSmallB, nBlkXP, 0);
            if (d->vi.format->colorFamily != cmGray)
                upsizer->simpleResize_uint8_t(upsizerUV, MaskFullUVB, VPitchUV, MaskSmallB, nBlkXP, 0);

            // analyse vectors field to detect occlusion
            //        double occNormF = time256/(256*ml);
            //        MakeVectorOcclusionMask(mvClipF, nBlkX, nBlkY, occNormF, 1.0, nPel, MaskSmallF, nBlkXP);
            MakeVectorOcclusionMaskTime(&fgopF, 0, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallF, nBlkXP, time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);

            CheckAndPadMaskSmall(MaskSmallF, nBlkXP, nBlkYP, nBlkX, nBlkY);

            upsizer->simpleResize_uint8_t(upsizer, MaskFullYF, VPitchY, MaskSmallF, nBlkXP, 0);
            if (d->vi.format->colorFamily != cmGray)
                upsizerUV->simpleResize_uint8_t(upsizerUV, MaskFullUVF, VPitchUV, MaskSmallF, nBlkXP, 0);

            if (maskmode == 2) { // These motion vectors should only be needed with maskmode 2. Why was the Avisynth plugin requesting them for all mask modes?
                // Get motion info from more frames for occlusion areas

                // forward from previous to current
                const VSFrameRef *mvFF = vsapi->getFrameFilter(nleft, d->mvfw, frameCtx);
                const VSMap *mvprops = vsapi->getFramePropsRO(mvFF);
                fgopUpdate(&fgopF, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
                isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvFF);

                // backward from next next to next
                const VSFrameRef *mvBB = vsapi->getFrameFilter(nright, d->mvbw, frameCtx);
                mvprops = vsapi->getFramePropsRO(mvBB);
                fgopUpdate(&fgopB, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
                isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvBB);
            }

            int nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * bytesPerSample * nPel;
            int nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * bytesPerSample * nPel;

            if (maskmode == 2 && isUsableB && isUsableF) { // slow method with extra frames
                // get vector mask from extra frames
                MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYBB, nBlkXP, VYSmallYBB, nBlkXP);
                MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYFF, nBlkXP, VYSmallYFF, nBlkXP);

                CheckAndPadSmallY(VXSmallYBB, VYSmallYBB, nBlkXP, nBlkYP, nBlkX, nBlkY);
                CheckAndPadSmallY(VXSmallYFF, VYSmallYFF, nBlkXP, nBlkYP, nBlkX, nBlkY);

                upsizer->simpleResize_int16_t(upsizer, VXFullYBB, VPitchY, VXSmallYBB, nBlkXP, 1);
                upsizer->simpleResize_int16_t(upsizer, VYFullYBB, VPitchY, VYSmallYBB, nBlkXP, 0);

                upsizer->simpleResize_int16_t(upsizer, VXFullYFF, VPitchY, VXSmallYFF, nBlkXP, 1);
                upsizer->simpleResize_int16_t(upsizer, VYFullYFF, VPitchY, VYSmallYFF, nBlkXP, 0);

                d->FlowInterExtra(pDst[0], nDstPitches[0],
                                  pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                                  VXFullYB, VXFullYF, VYFullYB, VYFullYF,
                                  MaskFullYB, MaskFullYF, VPitchY,
                                  nWidth, nHeight, time256, nPel,
                                  VXFullYBB, VXFullYFF, VYFullYBB, VYFullYFF);
                if (d->vi.format->colorFamily != cmGray) {
                    VectorSmallMaskYToHalfUV(VXSmallYBB, nBlkXP, nBlkYP, VXSmallUVBB, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYBB, nBlkXP, nBlkYP, VYSmallUVBB, yRatioUV);
                    VectorSmallMaskYToHalfUV(VXSmallYFF, nBlkXP, nBlkYP, VXSmallUVFF, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYFF, nBlkXP, nBlkYP, VYSmallUVFF, yRatioUV);

                    upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVBB, VPitchUV, VXSmallUVBB, nBlkXP, 1);
                    upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVBB, VPitchUV, VYSmallUVBB, nBlkXP, 0);

                    upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVFF, VPitchUV, VXSmallUVFF, nBlkXP, 1);
                    upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVFF, VPitchUV, VYSmallUVFF, nBlkXP, 0);

                    d->FlowInterExtra(pDst[1], nDstPitches[1],
                                      pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                                      VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                      MaskFullUVB, MaskFullUVF, VPitchUV,
                                      nWidthUV, nHeightUV, time256, nPel,
                                      VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF);
                    d->FlowInterExtra(pDst[2], nDstPitches[2],
                                      pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                                      VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                      MaskFullUVB, MaskFullUVF, VPitchUV,
                                      nWidthUV, nHeightUV, time256, nPel,
                                      VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF);
                }
            } else if (maskmode == 1) { // old method without extra frames
                d->FlowInter(pDst[0], nDstPitches[0],
                             pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                             VXFullYB, VXFullYF, VYFullYB, VYFullYF,
                             MaskFullYB, MaskFullYF, VPitchY,
                             nWidth, nHeight, time256, nPel);
                if (d->vi.format->colorFamily != cmGray) {
                    d->FlowInter(pDst[1], nDstPitches[1],
                                 pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                                 VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                 MaskFullUVB, MaskFullUVF, VPitchUV,
                                 nWidthUV, nHeightUV, time256, nPel);
                    d->FlowInter(pDst[2], nDstPitches[2],
                                 pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                                 VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                 MaskFullUVB, MaskFullUVF, VPitchUV,
                                 nWidthUV, nHeightUV, time256, nPel);
                }
            } else { // mode=0, faster simple method
                d->FlowInterSimple(pDst[0], nDstPitches[0],
                                   pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                                   VXFullYB, VXFullYF, VYFullYB, VYFullYF,
                                   MaskFullYB, MaskFullYF, VPitchY,
                                   nWidth, nHeight, time256, nPel);
                if (d->vi.format->colorFamily != cmGray) {
                    d->FlowInterSimple(pDst[1], nDstPitches[1],
                                       pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                                       VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                       MaskFullUVB, MaskFullUVF, VPitchUV,
                                       nWidthUV, nHeightUV, time256, nPel);
                    d->FlowInterSimple(pDst[2], nDstPitches[2],
                                       pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                                       VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                       MaskFullUVB, MaskFullUVF, VPitchUV,
                                       nWidthUV, nHeightUV, time256, nPel);
                }
            }

            if (maskmode == 2) {
                free(VXFullYBB);
                free(VYFullYBB);
                free(VXSmallYBB);
                free(VYSmallYBB);
                free(VXFullYFF);
                free(VYFullYFF);
                free(VXSmallYFF);
                free(VYSmallYFF);
            }

            free(MaskSmallB);
            free(MaskFullYB);
            free(MaskSmallF);
            free(MaskFullYF);

            if (d->vi.format->colorFamily != cmGray) {
                if (maskmode == 2) {
                    free(VXFullUVBB);
                    free(VYFullUVBB);
                    free(VXSmallUVBB);
                    free(VYSmallUVBB);
                    free(VXFullUVFF);
                    free(VYFullUVFF);
                    free(VXSmallUVFF);
                    free(VYSmallUVFF);
                }

                free(MaskFullUVB);
                free(MaskFullUVF);
            }

            vsapi->freeFrame(src);
            vsapi->freeFrame(ref);

            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            vsapi->freeFrame(mvB);
            vsapi->freeFrame(mvF);

            return dst;
        } else { // poor estimation
            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            vsapi->freeFrame(mvB);
            vsapi->freeFrame(mvF);

            const VSFrameRef *src = vsapi->getFrameFilter(VSMIN(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);

            if (blend) { //let's blend src with ref frames like ConvertFPS
                uint8_t *pDst[3];
                const uint8_t *pRef[3], *pSrc[3];
                int nDstPitches[3], nRefPitches[3], nSrcPitches[3];

                const VSFrameRef *ref = vsapi->getFrameFilter(VSMIN(nright, d->oldvi->numFrames - 1), d->node, frameCtx);

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
                Blend(pDst[0], pSrc[0], pRef[0], nHeight, nWidth, nDstPitches[0], nSrcPitches[0], nRefPitches[0], time256, bitsPerSample);
                if (d->vi.format->colorFamily != cmGray) {
                    Blend(pDst[1], pSrc[1], pRef[1], nHeightUV, nWidthUV, nDstPitches[1], nSrcPitches[1], nRefPitches[1], time256, bitsPerSample);
                    Blend(pDst[2], pSrc[2], pRef[2], nHeightUV, nWidthUV, nDstPitches[2], nSrcPitches[2], nRefPitches[2], time256, bitsPerSample);
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
    (void)core;

    MVFlowFPSData *d = (MVFlowFPSData *)instanceData;


    if (d->vi.format->colorFamily != cmGray)
        simpleDeinit(&d->upsizerUV);

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
    (void)userData;

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

    d.thscd1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.opt = !!vsapi->propGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 2;


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
    int evil_err[3];
    int nHeightS = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    int nSuperPel = int64ToIntS(vsapi->propGetInt(props, "Super_pel", 0, &evil_err[2]));
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
        vsapi->propSetInt(args, "opt", d.opt, paReplace);
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
    d.oldvi = vsapi->getVideoInfo(d.node);
    d.vi = *d.oldvi;


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

    if (d.mvbw_data.nHeight != nHeightS || d.mvbw_data.nWidth != nSuperWidth - d.nSuperHPad * 2 || d.mvbw_data.nPel != nSuperPel) {
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


    d.nBlkXP = d.mvbw_data.nBlkX;
    d.nBlkYP = d.mvbw_data.nBlkY;
    while (d.nBlkXP * (d.mvbw_data.nBlkSizeX - d.mvbw_data.nOverlapX) + d.mvbw_data.nOverlapX < d.mvbw_data.nWidth)
        d.nBlkXP++;
    while (d.nBlkYP * (d.mvbw_data.nBlkSizeY - d.mvbw_data.nOverlapY) + d.mvbw_data.nOverlapY < d.mvbw_data.nHeight)
        d.nBlkYP++;

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

    simpleInit(&d.upsizer, d.nWidthP, d.nHeightP, d.nBlkXP, d.nBlkYP, d.mvbw_data.nWidth, d.mvbw_data.nHeight, d.mvbw_data.nPel, d.opt);
    if (d.vi.format->colorFamily != cmGray)
        simpleInit(&d.upsizerUV, d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP, d.nWidthUV, d.nHeightUV, d.mvbw_data.nPel, d.opt);

    selectFlowInterFunctions(&d.FlowInterSimple, &d.FlowInter, &d.FlowInterExtra, d.vi.format->bitsPerSample, d.opt);


    MVFlowFPSHelperData *hb = (MVFlowFPSHelperData *)malloc(sizeof(MVFlowFPSHelperData));
    MVFlowFPSHelperData *hf = (MVFlowFPSHelperData *)malloc(sizeof(MVFlowFPSHelperData));

    hb->vectors = d.mvbw;
    hb->vectors_data = d.mvbw_data;
    hb->vi = vsapi->getVideoInfo(hb->vectors);
    hf->vectors = d.mvfw;
    hf->vectors_data = d.mvfw_data;
    hf->vi = vsapi->getVideoInfo(hf->vectors);

    hb->supervi = hf->supervi = vsapi->getVideoInfo(d.super);
    hb->thscd1 = hf->thscd1 = d.thscd1;
    hb->thscd2 = hf->thscd2 = d.thscd2;
    hb->nHeightP = hf->nHeightP = d.nHeightP;
    hb->nHeightPUV = hf->nHeightPUV = d.nHeightPUV;
    hb->VPitchY = hf->VPitchY = d.VPitchY;
    hb->VPitchUV = hf->VPitchUV = d.VPitchUV;
    hb->nBlkXP = hf->nBlkXP = d.nBlkXP;
    hb->nBlkYP = hf->nBlkYP = d.nBlkYP;
    hb->upsizer = hf->upsizer = d.upsizer;
    hb->upsizerUV = hf->upsizerUV = d.upsizerUV;

    vsapi->createFilter(in, out, "FlowFPSHelper", mvflowfpshelperInit, mvflowfpshelperGetFrame, mvflowfpshelperFree, fmParallel, 0, hb, core);

    VSPlugin *std_plugin = vsapi->getPluginById("com.vapoursynth.std", core);

    d.mvbw = vsapi->propGetNode(out, "clip", 0, NULL);
    vsapi->clearMap(out);
    VSMap *args = vsapi->createMap();
    vsapi->propSetNode(args, "clip", d.mvbw, paReplace);
    vsapi->freeNode(d.mvbw);
    VSMap *ret = vsapi->invoke(std_plugin, "Cache", args);
    d.mvbw = vsapi->propGetNode(ret, "clip", 0, NULL);
    vsapi->freeMap(ret);

    vsapi->createFilter(in, out, "FlowFPSHelper", mvflowfpshelperInit, mvflowfpshelperGetFrame, mvflowfpshelperFree, fmParallel, 0, hf, core);

    d.mvfw = vsapi->propGetNode(out, "clip", 0, NULL);
    vsapi->clearMap(out);
    vsapi->clearMap(args);
    vsapi->propSetNode(args, "clip", d.mvfw, paReplace);
    vsapi->freeNode(d.mvfw);
    ret = vsapi->invoke(std_plugin, "Cache", args);
    d.mvfw = vsapi->propGetNode(ret, "clip", 0, NULL);
    vsapi->freeMap(ret);


    data = (MVFlowFPSData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "FlowFPS", mvflowfpsInit, mvflowfpsGetFrame, mvflowfpsFree, fmParallel, 0, data, core);

    // AssumeFPS sets the _DurationNum and _DurationDen properties.
    VSNodeRef *node = vsapi->propGetNode(out, "clip", 0, NULL);
    vsapi->clearMap(args);
    vsapi->propSetNode(args, "clip", node, paReplace);
    vsapi->freeNode(node);
    vsapi->propSetInt(args, "fpsnum", d.vi.fpsNum, paReplace);
    vsapi->propSetInt(args, "fpsden", d.vi.fpsDen, paReplace);
    ret = vsapi->invoke(std_plugin, "AssumeFPS", args);
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
    ret = vsapi->invoke(std_plugin, "Cache", args);
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
                 "opt:int:opt;",
                 mvflowfpsCreate, 0, plugin);
}
