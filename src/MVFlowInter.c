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
#include "Fakery.h"
#include "MaskFun.h"
#include "MVAnalysisData.h"
#include "SimpleResize.h"


typedef struct MVFlowInterData {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *finest;
    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    float time;
    float ml;
    int blend;
    int64_t thscd1;
    int thscd2;
    int opt;

    MVAnalysisData mvbw_data;
    MVAnalysisData mvfw_data;

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

    SimpleResize upsizer;
    SimpleResize upsizerUV;

    FlowInterSimpleFunction FlowInterSimple;
    FlowInterFunction FlowInter;
    FlowInterExtraFunction FlowInterExtra;
} MVFlowInterData;


static void VS_CC mvflowinterInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVFlowInterData *d = (MVFlowInterData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvflowinterGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVFlowInterData *d = (MVFlowInterData *)*instanceData;

    if (activationReason == arInitial) {
        int off = d->mvbw_data.nDeltaFrame; // integer offset of reference frame

        if (n + off < d->vi->numFrames) {
            vsapi->requestFrameFilter(n, d->mvfw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvfw, frameCtx);

            vsapi->requestFrameFilter(n, d->mvbw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvbw, frameCtx);

            vsapi->requestFrameFilter(n, d->finest, frameCtx);
            vsapi->requestFrameFilter(n + off, d->finest, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->node, frameCtx);
        vsapi->requestFrameFilter(VSMIN(n + off, d->vi->numFrames - 1), d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        uint8_t *pDst[3];
        const uint8_t *pRef[3], *pSrc[3];
        int nDstPitches[3];
        int nRefPitches[3];
        int nSrcPitches[3];

        FakeGroupOfPlanes fgopF, fgopB;

        fgopInit(&fgopF, &d->mvfw_data);
        fgopInit(&fgopB, &d->mvbw_data);

        int isUsableB = 0;
        int isUsableF = 0;

        int off = d->mvbw_data.nDeltaFrame; // integer offset of reference frame

        if (n + off < d->vi->numFrames) {
            const VSFrameRef *mvF = vsapi->getFrameFilter(n + off, d->mvfw, frameCtx);
            const VSMap *mvprops = vsapi->getFramePropsRO(mvF);
            fgopUpdate(&fgopF, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            vsapi->freeFrame(mvF);
            isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);

            const VSFrameRef *mvB = vsapi->getFrameFilter(n, d->mvbw, frameCtx);
            mvprops = vsapi->getFramePropsRO(mvB);
            fgopUpdate(&fgopB, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            vsapi->freeFrame(mvB);
            isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
        }

        const int nWidth = d->mvbw_data.nWidth;
        const int nHeight = d->mvbw_data.nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int time256 = d->time256;
        const int blend = d->blend;

        int bitsPerSample = d->vi->format->bitsPerSample;
        int bytesPerSample = d->vi->format->bytesPerSample;

        if (isUsableB && isUsableF) {
            const VSFrameRef *src = vsapi->getFrameFilter(n, d->finest, frameCtx);
            const VSFrameRef *ref = vsapi->getFrameFilter(n + off, d->finest, frameCtx); //  ref for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

            for (int i = 0; i < d->vi->format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
            }

            const float ml = d->ml;
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
            const int nHeightP = d->nHeightP;
            const int nHeightPUV = d->nHeightPUV;
            const int nBlkXP = d->nBlkXP;
            const int nBlkYP = d->nBlkYP;
            SimpleResize *upsizer = &d->upsizer;
            SimpleResize *upsizerUV = &d->upsizerUV;

            int nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * bytesPerSample * nPel;
            int nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * bytesPerSample * nPel;


            int16_t *VXFullYB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VYFullYB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VXFullYF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VYFullYF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VXSmallYB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            int16_t *VYSmallYB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            int16_t *VXSmallYF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            int16_t *VYSmallYF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            uint8_t *MaskSmallB = (uint8_t *)malloc(nBlkXP * nBlkYP);
            uint8_t *MaskFullYB = (uint8_t *)malloc(nHeightP * VPitchY);
            uint8_t *MaskSmallF = (uint8_t *)malloc(nBlkXP * nBlkYP);
            uint8_t *MaskFullYF = (uint8_t *)malloc(nHeightP * VPitchY);
            int16_t *VXFullUVB = NULL;
            int16_t *VYFullUVB = NULL;
            int16_t *VXFullUVF = NULL;
            int16_t *VYFullUVF = NULL;
            int16_t *VXSmallUVB = NULL;
            int16_t *VYSmallUVB = NULL;
            int16_t *VXSmallUVF = NULL;
            int16_t *VYSmallUVF = NULL;
            uint8_t *MaskFullUVB = NULL;
            uint8_t *MaskFullUVF = NULL;


            // make  vector vx and vy small masks
            MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYB, nBlkXP, VYSmallYB, nBlkXP);
            MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYF, nBlkXP, VYSmallYF, nBlkXP);

            CheckAndPadSmallY(VXSmallYB, VYSmallYB, nBlkXP, nBlkYP, nBlkX, nBlkY);
            CheckAndPadSmallY(VXSmallYF, VYSmallYF, nBlkXP, nBlkYP, nBlkX, nBlkY);

            // analyse vectors field to detect occlusion
            //      double occNormB = (256-time256)/(256*ml);
            MakeVectorOcclusionMaskTime(&fgopB, 1, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallB, nBlkXP, (256 - time256), nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            //      double occNormF = time256/(256*ml);
            MakeVectorOcclusionMaskTime(&fgopF, 0, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallF, nBlkXP, time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);

            CheckAndPadMaskSmall(MaskSmallB, nBlkXP, nBlkYP, nBlkX, nBlkY);
            CheckAndPadMaskSmall(MaskSmallF, nBlkXP, nBlkYP, nBlkX, nBlkY);

            // upsize (bilinear interpolate) vector masks to fullframe size


            upsizer->simpleResize_int16_t(upsizer, VXFullYB, VPitchY, VXSmallYB, nBlkXP, 1);
            upsizer->simpleResize_int16_t(upsizer, VYFullYB, VPitchY, VYSmallYB, nBlkXP, 0);
            upsizer->simpleResize_int16_t(upsizer, VXFullYF, VPitchY, VXSmallYF, nBlkXP, 1);
            upsizer->simpleResize_int16_t(upsizer, VYFullYF, VPitchY, VYSmallYF, nBlkXP, 0);
            upsizer->simpleResize_uint8_t(upsizer, MaskFullYB, VPitchY, MaskSmallB, nBlkXP, 0);
            upsizer->simpleResize_uint8_t(upsizer, MaskFullYF, VPitchY, MaskSmallF, nBlkXP, 0);

            if (d->vi->format->colorFamily != cmGray) {
                VXFullUVB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VYFullUVB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VXFullUVF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VYFullUVF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VXSmallUVB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VYSmallUVB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VXSmallUVF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VYSmallUVF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                MaskFullUVB = (uint8_t *)malloc(nHeightPUV * VPitchUV);
                MaskFullUVF = (uint8_t *)malloc(nHeightPUV * VPitchUV);

                VectorSmallMaskYToHalfUV(VXSmallYB, nBlkXP, nBlkYP, VXSmallUVB, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYB, nBlkXP, nBlkYP, VYSmallUVB, yRatioUV);
                VectorSmallMaskYToHalfUV(VXSmallYF, nBlkXP, nBlkYP, VXSmallUVF, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYF, nBlkXP, nBlkYP, VYSmallUVF, yRatioUV);

                upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVB, VPitchUV, VXSmallUVB, nBlkXP, 1);
                upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVB, VPitchUV, VYSmallUVB, nBlkXP, 0);
                upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVF, VPitchUV, VXSmallUVF, nBlkXP, 1);
                upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVF, VPitchUV, VYSmallUVF, nBlkXP, 0);
                upsizerUV->simpleResize_uint8_t(upsizerUV, MaskFullUVB, VPitchUV, MaskSmallB, nBlkXP, 0);
                upsizerUV->simpleResize_uint8_t(upsizerUV, MaskFullUVF, VPitchUV, MaskSmallF, nBlkXP, 0);
            }


            {
                const VSFrameRef *mvFF = vsapi->getFrameFilter(n, d->mvfw, frameCtx);
                const VSMap *mvprops = vsapi->getFramePropsRO(mvFF);
                fgopUpdate(&fgopF, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
                isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvFF);

                const VSFrameRef *mvBB = vsapi->getFrameFilter(n + off, d->mvbw, frameCtx);
                mvprops = vsapi->getFramePropsRO(mvBB);
                fgopUpdate(&fgopB, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
                isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvBB);
            }


            if (isUsableF && isUsableB) {
                int16_t *VXFullYBB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VYFullYBB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VXFullYFF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VYFullYFF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VXSmallYBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                int16_t *VYSmallYBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                int16_t *VXSmallYFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                int16_t *VYSmallYFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));

                // get vector mask from extra frames
                MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYBB, nBlkXP, VYSmallYBB, nBlkXP);
                MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYFF, nBlkXP, VYSmallYFF, nBlkXP);

                CheckAndPadSmallY(VXSmallYBB, VYSmallYBB, nBlkXP, nBlkYP, nBlkX, nBlkY);
                CheckAndPadSmallY(VXSmallYFF, VYSmallYFF, nBlkXP, nBlkYP, nBlkX, nBlkY);

                // upsize vectors to full frame
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

                if (d->vi->format->colorFamily != cmGray) {
                    int16_t *VXFullUVFF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VXFullUVBB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VYFullUVBB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VYFullUVFF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VXSmallUVBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    int16_t *VYSmallUVBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    int16_t *VXSmallUVFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    int16_t *VYSmallUVFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));

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

                    free(VXFullUVBB);
                    free(VYFullUVBB);
                    free(VXSmallUVBB);
                    free(VYSmallUVBB);
                    free(VXFullUVFF);
                    free(VYFullUVFF);
                    free(VXSmallUVFF);
                    free(VYSmallUVFF);
                }

                free(VXFullYBB);
                free(VYFullYBB);
                free(VXSmallYBB);
                free(VYSmallYBB);
                free(VXFullYFF);
                free(VYFullYFF);
                free(VXSmallYFF);
                free(VYSmallYFF);
            } else { // bad extra frames, use old method without extra frames
                d->FlowInter(pDst[0], nDstPitches[0],
                             pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                             VXFullYB, VXFullYF, VYFullYB, VYFullYF,
                             MaskFullYB, MaskFullYF, VPitchY,
                             nWidth, nHeight, time256, nPel);
                if (d->vi->format->colorFamily != cmGray) {
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
            }


            free(VXFullYB);
            free(VYFullYB);
            free(VXSmallYB);
            free(VYSmallYB);
            free(VXFullYF);
            free(VYFullYF);
            free(VXSmallYF);
            free(VYSmallYF);
            free(MaskSmallB);
            free(MaskFullYB);
            free(MaskSmallF);
            free(MaskFullYF);

            if (d->vi->format->colorFamily != cmGray) {
                free(VXFullUVB);
                free(VYFullUVB);
                free(VXSmallUVB);
                free(VYSmallUVB);
                free(VXFullUVF);
                free(VYFullUVF);
                free(VXSmallUVF);
                free(VYSmallUVF);
                free(MaskFullUVB);
                free(MaskFullUVF);
            }

            vsapi->freeFrame(src);
            vsapi->freeFrame(ref);

            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            return dst;
        } else { // not usable
            // poor estimation

            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);

            if (blend) //let's blend src with ref frames like ConvertFPS
            {
                const VSFrameRef *ref = vsapi->getFrameFilter(VSMIN(n + off, d->vi->numFrames - 1), d->node, frameCtx);

                VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

                for (int i = 0; i < d->vi->format->numPlanes; i++) {
                    pDst[i] = vsapi->getWritePtr(dst, i);
                    pRef[i] = vsapi->getReadPtr(ref, i);
                    pSrc[i] = vsapi->getReadPtr(src, i);
                    nDstPitches[i] = vsapi->getStride(dst, i);
                    nRefPitches[i] = vsapi->getStride(ref, i);
                    nSrcPitches[i] = vsapi->getStride(src, i);
                }

                // blend with time weight
                Blend(pDst[0], pSrc[0], pRef[0], nHeight, nWidth, nDstPitches[0], nSrcPitches[0], nRefPitches[0], time256, bitsPerSample);
                if (d->vi->format->colorFamily != cmGray) {
                    Blend(pDst[1], pSrc[1], pRef[1], nHeightUV, nWidthUV, nDstPitches[1], nSrcPitches[1], nRefPitches[1], time256, bitsPerSample);
                    Blend(pDst[2], pSrc[2], pRef[2], nHeightUV, nWidthUV, nDstPitches[2], nSrcPitches[2], nRefPitches[2], time256, bitsPerSample);
                }

                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);

                return dst;
            } else { // no blend
                return src; // like ChangeFPS
            }
        }
    }

    return 0;
}


static void VS_CC mvflowinterFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVFlowInterData *d = (MVFlowInterData *)instanceData;

    simpleDeinit(&d->upsizer);
    if (d->vi->format->colorFamily != cmGray)
        simpleDeinit(&d->upsizerUV);

    vsapi->freeNode(d->finest);
    vsapi->freeNode(d->super);
    vsapi->freeNode(d->mvfw);
    vsapi->freeNode(d->mvbw);
    vsapi->freeNode(d->node);
    free(d);
}


static void VS_CC mvflowinterCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

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

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.opt = !!vsapi->propGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 2;


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

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "FlowInter: failed to retrieve first frame from super clip. Error message: ";
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
            vsapi->setError(out, "FlowInter: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);

#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };
    const char *filter_name = "FlowInter";

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


    if (d.mvbw_data.nDeltaFrame <= 0 || d.mvfw_data.nDeltaFrame <= 0) {
        vsapi->setError(out, "FlowInter: cannot use motion vectors with absolute frame references.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    // XXX Alternatively, use both clips' delta as offsets in GetFrame.
    if (d.mvfw_data.nDeltaFrame != d.mvbw_data.nDeltaFrame) {
        vsapi->setError(out, "FlowInter: mvbw and mvfw must be generated with the same delta.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    // Make sure the motion vector clips are correct.
    if (!d.mvbw_data.isBackward || d.mvfw_data.isBackward) {
        if (!d.mvbw_data.isBackward)
            vsapi->setError(out, "FlowInter: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "FlowInter: mvfw must be generated with isb=False.");
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
            snprintf(error_msg, ERROR_SIZE, "FlowInter: %s", vsapi->getError(ret));
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
            snprintf(error_msg, ERROR_SIZE, "FlowInter: %s", vsapi->getError(ret));
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
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.mvbw_data.nHeight != nHeightS || d.mvbw_data.nWidth != nSuperWidth - d.nSuperHPad * 2 || d.mvbw_data.nPel != nSuperPel) {
        vsapi->setError(out, "FlowInter: wrong source or super clip frame size.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, "FlowInter: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
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
    if (d.vi->format->colorFamily != cmGray)
        simpleInit(&d.upsizerUV, d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP, d.nWidthUV, d.nHeightUV, d.mvbw_data.nPel, d.opt);

    selectFlowInterFunctions(&d.FlowInterSimple, &d.FlowInter, &d.FlowInterExtra, d.vi->format->bitsPerSample, d.opt);


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
                 "opt:int:opt;",
                 mvflowinterCreate, 0, plugin);
}
