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
#include "CopyCode.h"
#include "CommonFunctions.h"
#include "MaskFun.h"
#include "MVAnalysisData.h"
#include "Overlap.h"
#include "SimpleResize.h"


typedef struct MVBlockFPSData {
    VSNodeRef *node;
    VSVideoInfo vi;
    const VSVideoInfo *oldvi;
    const VSVideoInfo *supervi;

    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    int64_t num, den;
    int mode;
    double ml;
    int blend;
    int thscd1, thscd2;
    int opt;

    MVAnalysisData mvbw_data;
    MVAnalysisData mvfw_data;

    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;

    int nWidthUV;
    int nHeightUV;
    int nPitchY;
    int nPitchUV;
    int nWidthP;
    int nHeightP;
    int nWidthPUV;
    int nHeightPUV;
    int nBlkXP;
    int nBlkYP;

    SimpleResize upsizer;
    SimpleResize upsizerUV;

    int64_t fa, fb;

    int dstTempPitch;
    int dstTempPitchUV;
    int nBlkPitch;

    OverlapWindows *OverWins;
    OverlapWindows *OverWinsUV;

    OverlapsFunction OVERS[3];
    ToPixelsFunction ToPixels;
} MVBlockFPSData;


static void VS_CC mvblockfpsInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVBlockFPSData *d = (MVBlockFPSData *)*instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static void MultMasks(uint8_t *smallmaskF, uint8_t *smallmaskB, uint8_t *smallmaskO, int nBlkX, int nBlkY) {
    for (int j = 0; j < nBlkY; j++) {
        for (int i = 0; i < nBlkX; i++)
            smallmaskO[i] = (smallmaskF[i] * smallmaskB[i]) / 255;
        smallmaskF += nBlkX;
        smallmaskB += nBlkX;
        smallmaskO += nBlkX;
    }
}


#define MEDIAN(PixelType) \
static inline PixelType MEDIAN_##PixelType(PixelType a, PixelType b, PixelType c) { \
    PixelType mn = VSMIN(a, b); \
    PixelType mx = VSMAX(a, b); \
    PixelType m = VSMIN(mx, c); \
    m = VSMAX(mn, m); \
    return m; \
}

MEDIAN(uint8_t)
MEDIAN(uint16_t)


#define RealResultBlock(PixelType) \
static void RealResultBlock_##PixelType(uint8_t *pDst, int dst_pitch, const uint8_t *pMCB, int MCB_pitch, const uint8_t *pMCF, int MCF_pitch, \
                            const uint8_t *pRef, int ref_pitch, const uint8_t *pSrc, int src_pitch, uint8_t *maskB, int mask_pitch, uint8_t *maskF, \
                            uint8_t *pOcc, int nBlkSizeX, int nBlkSizeY, int time256, int mode, int bitsPerSample) { \
    if (mode == 0) { \
        for (int h = 0; h < nBlkSizeY; h++) { \
            for (int w = 0; w < nBlkSizeX; w++) { \
                const PixelType *pMCB_ = (const PixelType *)pMCB; \
                const PixelType *pMCF_ = (const PixelType *)pMCF; \
                PixelType *pDst_ = (PixelType *)pDst; \
 \
                int mca = (pMCB_[w] * time256 + pMCF_[w] * (256 - time256)) >> 8; /* MC fetched average */ \
                pDst_[w] = mca; \
            } \
            pDst += dst_pitch; \
            pMCB += MCB_pitch; \
            pMCF += MCF_pitch; \
        } \
    } else if (mode == 1) { \
        for (int h = 0; h < nBlkSizeY; h++) { \
            for (int w = 0; w < nBlkSizeX; w++) { \
                const PixelType *pMCB_ = (const PixelType *)pMCB; \
                const PixelType *pMCF_ = (const PixelType *)pMCF; \
                const PixelType *pRef_ = (const PixelType *)pRef; \
                const PixelType *pSrc_ = (const PixelType *)pSrc; \
                PixelType *pDst_ = (PixelType *)pDst; \
 \
                int mca = (pMCB_[w] * time256 + pMCF_[w] * (256 - time256)) >> 8; /* MC fetched average */ \
                int sta = MEDIAN_##PixelType(pRef_[w], pSrc_[w], mca);             /* static median */ \
                pDst_[w] = sta; \
            } \
            pDst += dst_pitch; \
            pMCB += MCB_pitch; \
            pMCF += MCF_pitch; \
            pRef += ref_pitch; \
            pSrc += src_pitch; \
        } \
    } else if (mode == 2) { \
        for (int h = 0; h < nBlkSizeY; h++) { \
            for (int w = 0; w < nBlkSizeX; w++) { \
                const PixelType *pMCB_ = (const PixelType *)pMCB; \
                const PixelType *pMCF_ = (const PixelType *)pMCF; \
                const PixelType *pRef_ = (const PixelType *)pRef; \
                const PixelType *pSrc_ = (const PixelType *)pSrc; \
                PixelType *pDst_ = (PixelType *)pDst; \
 \
                int avg = (pRef_[w] * time256 + pSrc_[w] * (256 - time256)) >> 8; /* simple temporal non-MC average */ \
                int dyn = MEDIAN_##PixelType(avg, pMCB_[w], pMCF_[w]);             /* dynamic median */ \
                pDst_[w] = dyn; \
            } \
            pDst += dst_pitch; \
            pMCB += MCB_pitch; \
            pMCF += MCF_pitch; \
            pRef += ref_pitch; \
            pSrc += src_pitch; \
        } \
    } else if (mode == 3 || mode == 6) { \
        for (int h = 0; h < nBlkSizeY; h++) { \
            for (int w = 0; w < nBlkSizeX; w++) { \
                const PixelType *pMCB_ = (const PixelType *)pMCB; \
                const PixelType *pMCF_ = (const PixelType *)pMCF; \
                PixelType *pDst_ = (PixelType *)pDst; \
 \
                pDst_[w] = (((maskB[w] * pMCF_[w] + (255 - maskB[w]) * pMCB_[w] + 255) >> 8) * time256 + \
                            ((maskF[w] * pMCB_[w] + (255 - maskF[w]) * pMCF_[w] + 255) >> 8) * (256 - time256)) >> \
                           8; \
            } \
            pDst += dst_pitch; \
            pMCB += MCB_pitch; \
            pMCF += MCF_pitch; \
            maskB += mask_pitch; \
            maskF += mask_pitch; \
        } \
    } else if (mode == 4 || mode == 7) { \
        for (int h = 0; h < nBlkSizeY; h++) { \
            for (int w = 0; w < nBlkSizeX; w++) { \
                const PixelType *pMCB_ = (const PixelType *)pMCB; \
                const PixelType *pMCF_ = (const PixelType *)pMCF; \
                const PixelType *pRef_ = (const PixelType *)pRef; \
                const PixelType *pSrc_ = (const PixelType *)pSrc; \
                PixelType *pDst_ = (PixelType *)pDst; \
 \
                int f = (maskF[w] * pMCB_[w] + (255 - maskF[w]) * pMCF_[w] + 255) >> 8; \
                int b = (maskB[w] * pMCF_[w] + (255 - maskB[w]) * pMCB_[w] + 255) >> 8; \
                int avg = (pRef_[w] * time256 + pSrc_[w] * (256 - time256) + 255) >> 8; /* simple temporal non-MC average */ \
                int m = (b * time256 + f * (256 - time256)) >> 8; \
                pDst_[w] = (avg * pOcc[w] + m * (255 - pOcc[w]) + 255) >> 8; \
            } \
            pDst += dst_pitch; \
            pMCB += MCB_pitch; \
            pMCF += MCF_pitch; \
            pRef += ref_pitch; \
            pSrc += src_pitch; \
            maskB += mask_pitch; \
            maskF += mask_pitch; \
            pOcc += mask_pitch; \
        } \
    } else if (mode == 5 || mode == 8) { \
        for (int h = 0; h < nBlkSizeY; h++) { \
            for (int w = 0; w < nBlkSizeX; w++) { \
                PixelType *pDst_ = (PixelType *)pDst; \
 \
                pDst_[w] = pOcc[w] << (bitsPerSample - 8); \
            } \
            pDst += dst_pitch; \
            pOcc += mask_pitch; \
        } \
    } \
}

RealResultBlock(uint8_t)
RealResultBlock(uint16_t)


static void ResultBlock(uint8_t *pDst, int dst_pitch, const uint8_t *pMCB, int MCB_pitch, const uint8_t *pMCF, int MCF_pitch,
                        const uint8_t *pRef, int ref_pitch, const uint8_t *pSrc, int src_pitch, uint8_t *maskB, int mask_pitch, uint8_t *maskF,
                        uint8_t *pOcc, int nBlkSizeX, int nBlkSizeY, int time256, int mode, int bitsPerSample) {
    if (bitsPerSample == 8)
        RealResultBlock_uint8_t(pDst, dst_pitch, pMCB, MCB_pitch, pMCF, MCF_pitch, pRef, ref_pitch, pSrc, src_pitch, maskB, mask_pitch, maskF, pOcc, nBlkSizeX, nBlkSizeY, time256, mode, bitsPerSample);
    else
        RealResultBlock_uint16_t(pDst, dst_pitch, pMCB, MCB_pitch, pMCF, MCF_pitch, pRef, ref_pitch, pSrc, src_pitch, maskB, mask_pitch, maskF, pOcc, nBlkSizeX, nBlkSizeY, time256, mode, bitsPerSample);
}


static const VSFrameRef *VS_CC mvblockfpsGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVBlockFPSData *d = (MVBlockFPSData *)*instanceData;

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
            vsapi->requestFrameFilter(nright, d->mvfw, frameCtx);  // requests nleft, nleft + off
            vsapi->requestFrameFilter(nleft, d->mvbw, frameCtx);   // requests nleft, nleft + off

            vsapi->requestFrameFilter(nleft, d->super, frameCtx);
            vsapi->requestFrameFilter(nright, d->super, frameCtx);
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

        if (time256 == 0)
            return vsapi->getFrameFilter(VSMIN(nleft, d->oldvi->numFrames - 1), d->node, frameCtx); // simply left
        else if (time256 == 256)
            return vsapi->getFrameFilter(VSMIN(nright, d->oldvi->numFrames - 1), d->node, frameCtx); // simply right

        FakeGroupOfPlanes fgopF, fgopB;

        fgopInit(&fgopF, &d->mvfw_data);
        fgopInit(&fgopB, &d->mvbw_data);

        int isUsableF = 0;
        int isUsableB = 0;

        if (nleft < d->oldvi->numFrames && nright < d->oldvi->numFrames) {
            // forward from current to next
            const VSFrameRef *mvF = vsapi->getFrameFilter(nright, d->mvfw, frameCtx);
            const VSMap *mvprops = vsapi->getFramePropsRO(mvF);
            fgopUpdate(&fgopF, (const int *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);
            vsapi->freeFrame(mvF);

            // backward from next to current
            const VSFrameRef *mvB = vsapi->getFrameFilter(nleft, d->mvbw, frameCtx);
            mvprops = vsapi->getFramePropsRO(mvB);
            fgopUpdate(&fgopB, (const int *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
            vsapi->freeFrame(mvB);
        }

        const int nWidth[3] = { d->mvbw_data.nWidth, d->nWidthUV, d->nWidthUV };
        const int nHeight[3] = { d->mvbw_data.nHeight, d->nHeightUV, d->nHeightUV };
        const int nHeightP = d->nHeightP;
        const int nHeightPUV = d->nHeightPUV;
        const int mode = d->mode;
        const double ml = d->ml;
        const int blend = d->blend;
        const int opt = d->opt;
        const int xRatioUV = d->mvbw_data.xRatioUV;
        const int yRatioUV = d->mvbw_data.yRatioUV;
        const int nBlkX = d->mvbw_data.nBlkX;
        const int nBlkY = d->mvbw_data.nBlkY;
        const int nBlkSizeX[3] = { d->mvbw_data.nBlkSizeX, nBlkSizeX[0] / xRatioUV, nBlkSizeX[1] };
        const int nBlkSizeY[3] = { d->mvbw_data.nBlkSizeY, nBlkSizeY[0] / yRatioUV, nBlkSizeY[1] };
        const int nOverlapX[3] = { d->mvbw_data.nOverlapX, nOverlapX[0] / xRatioUV, nOverlapX[1] };
        const int nOverlapY[3] = { d->mvbw_data.nOverlapY, nOverlapY[0] / yRatioUV, nOverlapY[1] };
        const int nPel = d->mvbw_data.nPel;
        const int nPitch[3] = { d->nPitchY, d->nPitchUV, d->nPitchUV };
        const int nBlkXP = d->nBlkXP;
        const int nBlkYP = d->nBlkYP;
        const int nWidth_B[3] = { nBlkX * (nBlkSizeX[0] - nOverlapX[0]) + nOverlapX[0], nWidth_B[0] / xRatioUV, nWidth_B[1] };
        const int nHeight_B[3] = { nBlkY * (nBlkSizeY[0] - nOverlapY[0]) + nOverlapY[0], nHeight_B[0] / yRatioUV, nHeight_B[1] };
        SimpleResize *upsizer = &d->upsizer;
        SimpleResize *upsizerUV = &d->upsizerUV;

        const int nSuperHPad = d->nSuperHPad;
        const int nSuperVPad = d->nSuperVPad;
        const int nSuperModeYUV = d->nSuperModeYUV;
        const int nSuperLevels = d->nSuperLevels;
        const int nSuperPel = d->nSuperPel;

        const int bitsPerSample = d->supervi->format->bitsPerSample;
        const int bytesPerSample = d->supervi->format->bytesPerSample;

        int planes = 1;
        if (nSuperModeYUV & UVPLANES)
            planes = 3;


        if (isUsableB && isUsableF) {
            uint8_t *pDst[3] = { NULL };
            const uint8_t *pRef[3] = { NULL };
            const uint8_t *pSrc[3] = { NULL };
            int nDstPitches[3] = { 0 };
            int nRefPitches[3] = { 0 };
            int nSrcPitches[3] = { 0 };

            // If both are usable, that means both nleft and nright are less than oldvi->numFrames. Thus there is no need to check nleft and nright here.
            const VSFrameRef *src = vsapi->getFrameFilter(nleft, d->super, frameCtx);
            const VSFrameRef *ref = vsapi->getFrameFilter(nright, d->super, frameCtx); //  right frame for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

            for (int i = 0; i < d->supervi->format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
            }

            MVGroupOfFrames pRefBGOF, pRefFGOF;

            mvgofInit(&pRefBGOF, nSuperLevels, nWidth[0], nHeight[0], nSuperPel, nSuperHPad, nSuperVPad, nSuperModeYUV, opt, xRatioUV, yRatioUV, d->supervi->format->bitsPerSample);
            mvgofInit(&pRefFGOF, nSuperLevels, nWidth[0], nHeight[0], nSuperPel, nSuperHPad, nSuperVPad, nSuperModeYUV, opt, xRatioUV, yRatioUV, d->supervi->format->bitsPerSample);

            mvgofUpdate(&pRefBGOF, (uint8_t **)pRef, nRefPitches);
            mvgofUpdate(&pRefFGOF, (uint8_t **)pSrc, nSrcPitches);

            MVPlane **pPlanesB = pRefBGOF.frames[0]->planes;
            MVPlane **pPlanesF = pRefFGOF.frames[0]->planes;


            uint8_t *MaskFullYB = (uint8_t *)malloc(nHeightP * nPitch[0]);
            uint8_t *MaskFullYF = (uint8_t *)malloc(nHeightP * nPitch[0]);
            uint8_t *MaskOccY = (uint8_t *)malloc(nHeightP * nPitch[0]);

            uint8_t *MaskFullUVB = NULL;
            uint8_t *MaskFullUVF = NULL;
            uint8_t *MaskOccUV = NULL;
            if (nSuperModeYUV & UVPLANES) {
                MaskFullUVB = (uint8_t *)malloc(nHeightPUV * nPitch[1]);
                MaskFullUVF = (uint8_t *)malloc(nHeightPUV * nPitch[1]);
                MaskOccUV = (uint8_t *)malloc(nHeightPUV * nPitch[1]);
            }


            uint8_t *smallMaskB = NULL;
            uint8_t *smallMaskF = NULL;
            uint8_t *smallMaskO = NULL;

            memset(MaskFullYB, 0, nHeightP * nPitch[0]);
            memset(MaskFullYF, 0, nHeightP * nPitch[0]);

            int blocks = nBlkX * nBlkY;

            if (mode >= 3 && mode <= 8) {
                smallMaskB = (uint8_t *)malloc(nBlkXP * nBlkYP);
                smallMaskF = (uint8_t *)malloc(nBlkXP * nBlkYP);
                smallMaskO = (uint8_t *)malloc(nBlkXP * nBlkYP);


                if (mode <= 5) {
                    MakeVectorOcclusionMaskTime(&fgopF, 0, nBlkX, nBlkY, ml, 1.0, nPel, smallMaskF, nBlkXP, time256, nBlkSizeX[0] - nOverlapX[0], nBlkSizeY[0] - nOverlapY[0]);
                    MakeVectorOcclusionMaskTime(&fgopB, 1, nBlkX, nBlkY, ml, 1.0, nPel, smallMaskB, nBlkXP, 256 - time256, nBlkSizeX[0] - nOverlapX[0], nBlkSizeY[0] - nOverlapY[0]);
                } else { // 6 to 8
                    MakeSADMaskTime(&fgopF, nBlkX, nBlkY, 4.0 / (ml * nBlkSizeX[0] * nBlkSizeY[0]), 1.0, nPel, smallMaskF, nBlkXP, time256, nBlkSizeX[0] - nOverlapX[0], nBlkSizeY[0] - nOverlapY[0], bitsPerSample);
                    MakeSADMaskTime(&fgopB, nBlkX, nBlkY, 4.0 / (ml * nBlkSizeX[0] * nBlkSizeY[0]), 1.0, nPel, smallMaskB, nBlkXP, 256 - time256, nBlkSizeX[0] - nOverlapX[0], nBlkSizeY[0] - nOverlapY[0], bitsPerSample);
                }

                if (nBlkXP > nBlkX) // fill right
                    for (int j = 0; j < nBlkY; j++) {
                        smallMaskF[j * nBlkXP + nBlkX] = smallMaskF[j * nBlkXP + nBlkX - 1];
                        smallMaskB[j * nBlkXP + nBlkX] = smallMaskB[j * nBlkXP + nBlkX - 1];
                    }
                if (nBlkYP > nBlkY) // fill bottom
                    for (int i = 0; i < nBlkXP; i++) {
                        smallMaskF[nBlkXP * nBlkY + i] = smallMaskF[nBlkXP * (nBlkY - 1) + i];
                        smallMaskB[nBlkXP * nBlkY + i] = smallMaskB[nBlkXP * (nBlkY - 1) + i];
                    }


                // upsize small mask to full frame size
                upsizer->simpleResize_uint8_t(upsizer, MaskFullYF, nPitch[0], smallMaskF, nBlkXP);
                upsizer->simpleResize_uint8_t(upsizer, MaskFullYB, nPitch[0], smallMaskB, nBlkXP);

                if (nSuperModeYUV & UVPLANES) {
                    upsizerUV->simpleResize_uint8_t(upsizerUV, MaskFullUVF, nPitch[1], smallMaskF, nBlkXP);
                    upsizerUV->simpleResize_uint8_t(upsizerUV, MaskFullUVB, nPitch[1], smallMaskB, nBlkXP);
                }
            }

            if (mode == 4 || mode == 5 || mode == 7 || mode == 8) {
                // make final (both directions) occlusion mask
                MultMasks(smallMaskF, smallMaskB, smallMaskO, nBlkXP, nBlkYP);
                // upsize small mask to full frame size
                upsizer->simpleResize_uint8_t(upsizer, MaskOccY, nPitch[0], smallMaskO, nBlkXP);
                if (nSuperModeYUV & UVPLANES)
                    upsizerUV->simpleResize_uint8_t(upsizerUV, MaskOccUV, nPitch[1], smallMaskO, nBlkXP);
            }

            pSrc[0] += nSuperHPad * bytesPerSample + nSrcPitches[0] * nSuperVPad; // add offset source in super
            pRef[0] += nSuperHPad * bytesPerSample + nRefPitches[0] * nSuperVPad;
            if (nSuperModeYUV & UVPLANES) {
                // XXX Seriously, how can this be right for anything that isn't 4:2:0?
                pSrc[1] += (nSuperHPad >> 1) * bytesPerSample + nSrcPitches[1] * (nSuperVPad >> 1);
                pSrc[2] += (nSuperHPad >> 1) * bytesPerSample + nSrcPitches[2] * (nSuperVPad >> 1);
                pRef[1] += (nSuperHPad >> 1) * bytesPerSample + nRefPitches[1] * (nSuperVPad >> 1);
                pRef[2] += (nSuperHPad >> 1) * bytesPerSample + nRefPitches[2] * (nSuperVPad >> 1);
            }


            int xRatio[3] = { 1, xRatioUV, xRatioUV };
            int yRatio[3] = { 1, yRatioUV, yRatioUV };

            uint8_t *pMaskFullB[3] = { MaskFullYB, MaskFullUVB, MaskFullUVB };
            uint8_t *pMaskFullF[3] = { MaskFullYF, MaskFullUVF, MaskFullUVF };
            uint8_t *pMaskOcc[3] = { MaskOccY, MaskOccUV, MaskOccUV };

            if (nOverlapX[0] == 0 && nOverlapY[0] == 0) {
                // fetch image blocks
                for (int i = 0; i < blocks; i++) {
                    const FakeBlockData *blockB = fgopGetBlock(&fgopB, 0, i);
                    const FakeBlockData *blockF = fgopGetBlock(&fgopF, 0, i);

                    for (int plane = 0; plane < planes; plane++) {
                        ResultBlock(pDst[plane], nDstPitches[plane],
                                    mvpGetPointer(pPlanesB[plane],
                                                  (blockB->x * nPel + ((blockB->vector.x * (256 - time256)) >> 8)) / xRatio[plane],
                                                  (blockB->y * nPel + ((blockB->vector.y * (256 - time256)) >> 8)) / yRatio[plane]),
                                    pPlanesB[plane]->nPitch,
                                    mvpGetPointer(pPlanesF[plane],
                                                  (blockF->x * nPel + ((blockF->vector.x * time256) >> 8)) / xRatio[plane],
                                                  (blockF->y * nPel + ((blockF->vector.y * time256) >> 8)) / yRatio[plane]),
                                    pPlanesF[plane]->nPitch,
                                    pRef[plane], nRefPitches[plane],
                                    pSrc[plane], nSrcPitches[plane],
                                    pMaskFullB[plane], nPitch[plane],
                                    pMaskFullF[plane], pMaskOcc[plane],
                                    nBlkSizeX[plane], nBlkSizeY[plane],
                                    time256, mode, bitsPerSample);

                        // update pDsts
                        pDst[plane] += nBlkSizeX[plane] * bytesPerSample;
                        pRef[plane] += nBlkSizeX[plane] * bytesPerSample;
                        pSrc[plane] += nBlkSizeX[plane] * bytesPerSample;
                        pMaskFullB[plane] += nBlkSizeX[plane];
                        pMaskFullF[plane] += nBlkSizeX[plane];
                        pMaskOcc[plane] += nBlkSizeX[plane];


                        if (!((i + 1) % nBlkX)) {
                            // blend rest right with time weight
                            Blend(pDst[plane], pSrc[plane], pRef[plane], nBlkSizeY[plane], nWidth[plane] - nBlkSizeX[plane] * nBlkX, nDstPitches[plane], nSrcPitches[plane], nRefPitches[plane], time256, bitsPerSample);

                            pDst[plane] += nBlkSizeY[plane] * nDstPitches[plane] - nBlkSizeX[plane] * nBlkX * bytesPerSample;
                            pRef[plane] += nBlkSizeY[plane] * nRefPitches[plane] - nBlkSizeX[plane] * nBlkX * bytesPerSample;
                            pSrc[plane] += nBlkSizeY[plane] * nSrcPitches[plane] - nBlkSizeX[plane] * nBlkX * bytesPerSample;
                            pMaskFullB[plane] += nBlkSizeY[plane] * nPitch[plane] - nBlkSizeX[plane] * nBlkX;
                            pMaskFullF[plane] += nBlkSizeY[plane] * nPitch[plane] - nBlkSizeX[plane] * nBlkX;
                            pMaskOcc[plane] += nBlkSizeY[plane] * nPitch[plane] - nBlkSizeX[plane] * nBlkX;
                        }
                    }
                }

                // blend rest bottom with time weight
                for (int plane = 0; plane < planes; plane++)
                    Blend(pDst[plane], pSrc[plane], pRef[plane], nHeight[plane] - nBlkSizeY[plane] * nBlkY, nWidth[plane], nDstPitches[plane], nSrcPitches[plane], nRefPitches[plane], time256, bitsPerSample);
            } else { // overlap
                for (int plane = 0; plane < planes; plane++) {
                    // blend rest right with time weight
                    Blend(pDst[plane] + nWidth_B[plane] * bytesPerSample,
                          pSrc[plane] + nWidth_B[plane] * bytesPerSample,
                          pRef[plane] + nWidth_B[plane] * bytesPerSample,
                          nHeight_B[plane],
                          nWidth[plane] - nWidth_B[plane],
                          nDstPitches[plane], nSrcPitches[plane], nRefPitches[plane], time256, bitsPerSample);

                    // blend rest bottom with time weight
                    Blend(pDst[plane] + nDstPitches[plane] * nHeight_B[plane],
                          pSrc[plane] + nSrcPitches[plane] * nHeight_B[plane],
                          pRef[plane] + nRefPitches[plane] * nHeight_B[plane],
                          nHeight[plane] - nHeight_B[plane],
                          nWidth[plane],
                          nDstPitches[plane], nSrcPitches[plane], nRefPitches[plane], time256, bitsPerSample);
                }

                const int dstTempPitch[3] = { d->dstTempPitch, d->dstTempPitchUV, d->dstTempPitchUV };
                int nBlkPitch = d->nBlkPitch;

                uint8_t *DstTemp[3] = { NULL };
                uint8_t *pDstTemp[3] = { NULL };
                for (int plane = 0; plane < planes; plane++) {
                    pDstTemp[plane] = DstTemp[plane] = (uint8_t *)malloc(dstTempPitch[plane] * nHeight[plane]);
                    memset(DstTemp[plane], 0, nHeight_B[plane] * dstTempPitch[plane]);
                }

                uint8_t *TmpBlock = (uint8_t *)malloc(nBlkSizeY[0] * nBlkPitch);

                for (int by = 0; by < nBlkY; by++) {
                    int wby = ((by + nBlkY - 3) / (nBlkY - 2)) * 3;
                    int xx[3] = { 0 };
                    for (int bx = 0; bx < nBlkX; bx++) {
                        // select window
                        int wbx = (bx + nBlkX - 3) / (nBlkX - 2);
                        int16_t *winOver[3] = { overGetWindow(d->OverWins, wby + wbx) };
                        if (planes > 1)
                            winOver[1] = winOver[2] = overGetWindow(d->OverWinsUV, wby + wbx);

                        int i = by * nBlkX + bx;

                        const FakeBlockData *blockB = fgopGetBlock(&fgopB, 0, i);
                        const FakeBlockData *blockF = fgopGetBlock(&fgopF, 0, i);

                        // firstly calculate result block and write it to temporary place, not to dst
                        for (int plane = 0; plane < planes; plane++) {
                            ResultBlock(TmpBlock, nBlkPitch,
                                        mvpGetPointer(pPlanesB[plane],
                                                      (blockB->x * nPel + ((blockB->vector.x * (256 - time256)) >> 8)) / xRatio[plane],
                                                      (blockB->y * nPel + ((blockB->vector.y * (256 - time256)) >> 8)) / yRatio[plane]),
                                        pPlanesB[plane]->nPitch,
                                        mvpGetPointer(pPlanesF[plane],
                                                      (blockF->x * nPel + ((blockF->vector.x * time256) >> 8)) / xRatio[plane],
                                                      (blockF->y * nPel + ((blockF->vector.y * time256) >> 8)) / yRatio[plane]),
                                        pPlanesF[plane]->nPitch,
                                        pRef[plane] + xx[plane] * bytesPerSample, nRefPitches[plane],
                                        pSrc[plane] + xx[plane] * bytesPerSample, nSrcPitches[plane],
                                        pMaskFullB[plane] + xx[plane], nPitch[plane],
                                        pMaskFullF[plane] + xx[plane], pMaskOcc[plane] + xx[plane],
                                        nBlkSizeX[plane], nBlkSizeY[plane],
                                        time256, mode, bitsPerSample);

                            d->OVERS[plane](pDstTemp[plane] + xx[plane] * bytesPerSample * 2, dstTempPitch[plane], TmpBlock, nBlkPitch, winOver[plane], nBlkSizeX[plane]);

                            xx[plane] += nBlkSizeX[plane] - nOverlapX[plane];
                        }
                    }

                    for (int plane = 0; plane < planes; plane++) {
                        pDstTemp[plane] += dstTempPitch[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                        pDst[plane] += nDstPitches[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                        pRef[plane] += nRefPitches[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                        pSrc[plane] += nSrcPitches[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                        pMaskFullB[plane] += nPitch[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                        pMaskFullF[plane] += nPitch[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                        pMaskOcc[plane] += nPitch[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                    }
                }

                for (int plane = 0; plane < planes; plane++) {
                    pDst[plane] = vsapi->getWritePtr(dst, plane);

                    d->ToPixels(pDst[plane], nDstPitches[plane], DstTemp[plane], dstTempPitch[plane], nWidth_B[plane], nHeight_B[plane], bitsPerSample);

                    free(DstTemp[plane]);
                }

                free(TmpBlock);
            }


            free(MaskFullYB);
            free(MaskFullYF);
            free(MaskOccY);
            if (nSuperModeYUV & UVPLANES) {
                free(MaskFullUVB);
                free(MaskFullUVF);
                free(MaskOccUV);
            }
            if (smallMaskB) {
                free(smallMaskB);
                free(smallMaskF);
                free(smallMaskO);
            }

            mvgofDeinit(&pRefBGOF);
            mvgofDeinit(&pRefFGOF);

            vsapi->freeFrame(src);
            vsapi->freeFrame(ref);

            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            return dst;
        } else { // poor estimation
            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            const VSFrameRef *src = vsapi->getFrameFilter(VSMIN(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);

            if (blend) { //let's blend src with ref frames like ConvertFPS
                uint8_t *pDst[3];
                const uint8_t *pRef[3], *pSrc[3];
                int nDstPitches[3], nRefPitches[3], nSrcPitches[3];

                const VSFrameRef *ref = vsapi->getFrameFilter(VSMIN(nright, d->oldvi->numFrames - 1), d->node, frameCtx);

                VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

                for (int plane = 0; plane < planes; plane++) {
                    pDst[plane] = vsapi->getWritePtr(dst, plane);
                    pRef[plane] = vsapi->getReadPtr(ref, plane);
                    pSrc[plane] = vsapi->getReadPtr(src, plane);
                    nDstPitches[plane] = vsapi->getStride(dst, plane);
                    nRefPitches[plane] = vsapi->getStride(ref, plane);
                    nSrcPitches[plane] = vsapi->getStride(src, plane);

                    Blend(pDst[plane], pSrc[plane], pRef[plane], nHeight[plane], nWidth[plane], nDstPitches[plane], nSrcPitches[plane], nRefPitches[plane], time256, bitsPerSample);
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


static void VS_CC mvblockfpsFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVBlockFPSData *d = (MVBlockFPSData *)instanceData;

    simpleDeinit(&d->upsizer);
    if (d->nSuperModeYUV & UVPLANES)
        simpleDeinit(&d->upsizerUV);

    if (d->mvbw_data.nOverlapX || d->mvbw_data.nOverlapY) {
        overDeinit(d->OverWins);
        free(d->OverWins);
        if (d->nSuperModeYUV & UVPLANES) {
            overDeinit(d->OverWinsUV);
            free(d->OverWinsUV);
        }
    }

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


static void selectFunctions(MVBlockFPSData *d) {
    const unsigned xRatioUV = d->mvbw_data.xRatioUV;
    const unsigned yRatioUV = d->mvbw_data.yRatioUV;
    const unsigned nBlkSizeX = d->mvbw_data.nBlkSizeX;
    const unsigned nBlkSizeY = d->mvbw_data.nBlkSizeY;
    const unsigned bits = d->vi.format->bytesPerSample * 8;

    if (d->vi.format->bitsPerSample == 8) {
        d->ToPixels = ToPixels_uint16_t_uint8_t;
    } else {
        d->ToPixels = ToPixels_uint32_t_uint16_t;
    }

    d->OVERS[0] = selectOverlapsFunction(nBlkSizeX, nBlkSizeY, bits, d->opt);
    d->OVERS[1] = d->OVERS[2] = selectOverlapsFunction(nBlkSizeX / xRatioUV, nBlkSizeY / yRatioUV, bits, d->opt);
}


static void VS_CC mvblockfpsCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVBlockFPSData d;
    MVBlockFPSData *data;

    int err;

    d.num = vsapi->propGetInt(in, "num", 0, &err);
    if (err)
        d.num = 25;

    d.den = vsapi->propGetInt(in, "den", 0, &err);
    if (err)
        d.den = 1;

    d.mode = int64ToIntS(vsapi->propGetInt(in, "mode", 0, &err));
    if (err)
        d.mode = 3;

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

    d.opt = !!vsapi->propGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;


    if (d.mode < 0 || d.mode > 8) {
        vsapi->setError(out, "BlockFPS: mode must be between 0 and 8 (inclusive).");
        return;
    }


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "BlockFPS: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg + errorLen, ERROR_SIZE - errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->setError(out, errorMsg);
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[6];
    int nHeightS = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    d.nSuperVPad = int64ToIntS(vsapi->propGetInt(props, "Super_vpad", 0, &evil_err[2]));
    d.nSuperPel = int64ToIntS(vsapi->propGetInt(props, "Super_pel", 0, &evil_err[3]));
    d.nSuperModeYUV = int64ToIntS(vsapi->propGetInt(props, "Super_modeyuv", 0, &evil_err[4]));
    d.nSuperLevels = int64ToIntS(vsapi->propGetInt(props, "Super_levels", 0, &evil_err[5]));
    vsapi->freeFrame(evil);

    for (int i = 0; i < 6; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "BlockFPS: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);

    // There is another variable called "error" a bit lower.
    {
#define ERROR_SIZE 512
        char error[ERROR_SIZE + 1] = { 0 };
        const char *filter_name = "BlockFPS";

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


    if (d.mvbw_data.nDeltaFrame <= 0 || d.mvfw_data.nDeltaFrame <= 0) {
        vsapi->setError(out, "BlockFPS: cannot use motion vectors with absolute frame references.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    // XXX Alternatively, use both clips' delta as offsets in GetFrame.
    if (d.mvbw_data.nDeltaFrame != d.mvfw_data.nDeltaFrame) {
        vsapi->setError(out, "BlockFPS: mvbw and mvfw must be generated with the same delta.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    // Make sure the motion vector clips are correct.
    if (!d.mvbw_data.isBackward || d.mvfw_data.isBackward) {
        if (!d.mvbw_data.isBackward)
            vsapi->setError(out, "BlockFPS: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "BlockFPS: mvfw must be generated with isb=False.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.oldvi = vsapi->getVideoInfo(d.node);
    d.vi = *d.oldvi;


    if (d.vi.fpsNum == 0 || d.vi.fpsDen == 0) {
        vsapi->setError(out, "BlockFPS: The input clip must have a frame rate. Invoke AssumeFPS if necessary.");
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


    d.supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = d.supervi->width;

    if (d.mvbw_data.nHeight != nHeightS ||
        d.mvbw_data.nWidth != nSuperWidth - d.nSuperHPad * 2 ||
        d.mvbw_data.nWidth != d.vi.width ||
        d.mvbw_data.nHeight != d.vi.height ||
        d.mvbw_data.nPel != d.nSuperPel) {
        vsapi->setError(out, "BlockFPS: wrong source or super clip frame size.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }

    if (!isConstantFormat(&d.vi) || d.vi.format->bitsPerSample > 16 || d.vi.format->sampleType != stInteger || d.vi.format->subSamplingW > 1 || d.vi.format->subSamplingH > 1 || (d.vi.format->colorFamily != cmYUV && d.vi.format->colorFamily != cmGray)) {
        vsapi->setError(out, "BlockFPS: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }


    d.nBlkXP = (d.mvbw_data.nBlkX * (d.mvbw_data.nBlkSizeX - d.mvbw_data.nOverlapX) + d.mvbw_data.nOverlapX < d.mvbw_data.nWidth) ? d.mvbw_data.nBlkX + 1 : d.mvbw_data.nBlkX;
    d.nBlkYP = (d.mvbw_data.nBlkY * (d.mvbw_data.nBlkSizeY - d.mvbw_data.nOverlapY) + d.mvbw_data.nOverlapY < d.mvbw_data.nHeight) ? d.mvbw_data.nBlkY + 1 : d.mvbw_data.nBlkY;
    d.nWidthP = d.nBlkXP * (d.mvbw_data.nBlkSizeX - d.mvbw_data.nOverlapX) + d.mvbw_data.nOverlapX;
    d.nHeightP = d.nBlkYP * (d.mvbw_data.nBlkSizeY - d.mvbw_data.nOverlapY) + d.mvbw_data.nOverlapY;

    d.nWidthPUV = d.nWidthP / d.mvbw_data.xRatioUV;
    d.nHeightPUV = d.nHeightP / d.mvbw_data.yRatioUV;
    d.nHeightUV = d.mvbw_data.nHeight / d.mvbw_data.yRatioUV;
    d.nWidthUV = d.mvbw_data.nWidth / d.mvbw_data.xRatioUV;

    d.nPitchY = (d.nWidthP + 15) & (~15);
    d.nPitchUV = (d.nWidthPUV + 15) & (~15);


    simpleInit(&d.upsizer, d.nWidthP, d.nHeightP, d.nBlkXP, d.nBlkYP, d.opt);
    if (d.nSuperModeYUV & UVPLANES)
        simpleInit(&d.upsizerUV, d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP, d.opt);

    if (d.mvbw_data.nOverlapX || d.mvbw_data.nOverlapY) {
        d.OverWins = (OverlapWindows *)malloc(sizeof(OverlapWindows));
        overInit(d.OverWins, d.mvbw_data.nBlkSizeX, d.mvbw_data.nBlkSizeY, d.mvbw_data.nOverlapX, d.mvbw_data.nOverlapY);
        if (d.nSuperModeYUV & UVPLANES) {
            d.OverWinsUV = (OverlapWindows *)malloc(sizeof(OverlapWindows));
            overInit(d.OverWinsUV, d.mvbw_data.nBlkSizeX / d.mvbw_data.xRatioUV, d.mvbw_data.nBlkSizeY / d.mvbw_data.yRatioUV, d.mvbw_data.nOverlapX / d.mvbw_data.xRatioUV, d.mvbw_data.nOverlapY / d.mvbw_data.yRatioUV);
        }
    }

    d.dstTempPitch = ((d.mvbw_data.nWidth + 15) / 16) * 16 * d.vi.format->bytesPerSample * 2;
    d.dstTempPitchUV = (((d.mvbw_data.nWidth / d.mvbw_data.xRatioUV) + 15) / 16) * 16 * d.vi.format->bytesPerSample * 2;
    d.nBlkPitch = ((d.mvbw_data.nBlkSizeX + 15) & (~15)) * d.vi.format->bytesPerSample;


    selectFunctions(&d);

    if (d.vi.format->bitsPerSample > 8)
        d.opt = 0;


    data = (MVBlockFPSData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "BlockFPS", mvblockfpsInit, mvblockfpsGetFrame, mvblockfpsFree, fmParallel, 0, data, core);

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
        snprintf(error_msg, ERROR_SIZE, "BlockFPS: Failed to invoke AssumeFPS. Error message: %s", vsapi->getError(ret));
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
        snprintf(error_msg, ERROR_SIZE, "BlockFPS: Failed to invoke Cache. Error message: %s", vsapi->getError(ret));
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


void mvblockfpsRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("BlockFPS",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "num:int:opt;"
                 "den:int:opt;"
                 "mode:int:opt;"
                 "ml:float:opt;"
                 "blend:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "opt:int:opt;",
                 mvblockfpsCreate, 0, plugin);
}
