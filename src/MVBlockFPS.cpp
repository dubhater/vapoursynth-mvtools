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

#include "CopyCode.h"
#include "CommonFunctions.h"
#include "MaskFun.h"
#include "MVInterface.h"
#include "SimpleResize.h"


typedef struct {
    VSNodeRef *node;
    VSVideoInfo vi;

    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    int64_t num, den;
    int mode;
    int thres;
    int blend;
    int thscd1, thscd2;
    int isse;

    MVClipDicks *mvClipB;
    MVClipDicks *mvClipF;

    MVFilter *bleh;

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

    uint8_t *OnesBlock;

    SimpleResize *upsizer;
    SimpleResize *upsizerUV;

    int64_t fa, fb;

    COPYFunction BLITLUMA;
} MVBlockFPSData;


static void VS_CC mvblockfpsInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVBlockFPSData *d = (MVBlockFPSData *) * instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static void MakeSmallMask(uint8_t *image, int imagePitch, uint8_t *smallmask, int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int threshold) {
    // it can be MMX

    // count occlusions in blocks
    uint8_t *psmallmask = smallmask;

    for (int ny = 0; ny < nBlkY; ny++) {
        for (int nx = 0; nx < nBlkX; nx++) {
            psmallmask[nx] = 0;
            for (int j = 0; j < nBlkSizeY; j++) {
                for (int i = 0; i < nBlkSizeX; i++)
                    if (image[i] == 0) // 0 is mark of occlusion
                        psmallmask[nx]++; // count
                image += imagePitch;
            }
            image += -imagePitch * nBlkSizeY + nBlkSizeX;
        }
        image += imagePitch * nBlkSizeY - nBlkX * nBlkSizeX;
        psmallmask += nBlkX;
    }

    // make small binary mask
    psmallmask = smallmask;

    for (int ny = 0; ny < nBlkY; ny++) {
        for (int nx = 0; nx < nBlkX; nx++) {
            if (psmallmask[nx] >= threshold)
                psmallmask[nx] = 255;
            else
                psmallmask[nx] = 0;

        }
        psmallmask += nBlkX;
    }
}


static void InflateMask(uint8_t *smallmask, int nBlkX, int nBlkY) {

    // inflate mask
    uint8_t *psmallmask = smallmask + nBlkX;

    for (int ny = 1; ny < nBlkY - 1; ny++) {// skip edges
        for (int nx = 1; nx < nBlkX - 1; nx++) {// skip edges
            if (psmallmask[nx] == 255) {
                psmallmask[nx - 1] = 192;
                psmallmask[nx + 1] = 192;
                psmallmask[nx - nBlkX - 1] = 144;
                psmallmask[nx - nBlkX] = 192;
                psmallmask[nx - nBlkX + 1] = 144;
                psmallmask[nx + nBlkX - 1] = 144;
                psmallmask[nx + nBlkX] = 192;
                psmallmask[nx + nBlkX + 1] = 144;
            }
        }
        psmallmask += nBlkX;
    }
}


static void MultMasks(uint8_t *smallmaskF, uint8_t *smallmaskB, uint8_t *smallmaskO,  int nBlkX, int nBlkY) {
    for (int j = 0; j < nBlkY; j++) {
        for (int i = 0; i < nBlkX; i++)
            smallmaskO[i] = (smallmaskF[i] * smallmaskB[i]) / 255;
        smallmaskF += nBlkX;
        smallmaskB += nBlkX;
        smallmaskO += nBlkX;
    }
}


static inline uint8_t MEDIAN(uint8_t a, uint8_t b, uint8_t c) {
    uint8_t mn = VSMIN(a, b);
    uint8_t mx = VSMAX(a, b);
    uint8_t m = VSMIN(mx, c);
    m = VSMAX(mn, m);
    return m;
}


static void ResultBlock(uint8_t *pDst, int dst_pitch, const uint8_t * pMCB, int MCB_pitch, const uint8_t * pMCF, int MCF_pitch,
        const uint8_t * pRef, int ref_pitch, const uint8_t * pSrc, int src_pitch, uint8_t *maskB, int mask_pitch, uint8_t *maskF,
        uint8_t *pOcc, int nBlkSizeX, int nBlkSizeY, int time256, int mode) {
    if (mode == 0) {
        for (int h = 0; h < nBlkSizeY; h++) {
            for (int w = 0; w < nBlkSizeX; w++) {
                int mca = (pMCB[w] * time256 + pMCF[w] * (256 - time256)) >> 8; // MC fetched average
                pDst[w] = mca;
            }
            pDst += dst_pitch;
            pMCB += MCB_pitch;
            pMCF += MCF_pitch;
        }
    } else if (mode == 1) {
        for (int h = 0; h < nBlkSizeY; h++) {
            for (int w = 0; w < nBlkSizeX; w++) {
                int mca = (pMCB[w] * time256 + pMCF[w] * (256 - time256)) >> 8; // MC fetched average
                int sta = MEDIAN(pRef[w], pSrc[w], mca); // static median
                pDst[w] = sta;

            }
            pDst += dst_pitch;
            pMCB += MCB_pitch;
            pMCF += MCF_pitch;
            pRef += ref_pitch;
            pSrc += src_pitch;
        }
    } else if (mode == 2) {
        for (int h = 0; h < nBlkSizeY; h++) {
            for (int w = 0; w < nBlkSizeX; w++) {
                int avg = (pRef[w] * time256 + pSrc[w] * (256 - time256)) >> 8; // simple temporal non-MC average
                int dyn = MEDIAN(avg, pMCB[w], pMCF[w]); // dynamic median
                pDst[w] = dyn;
            }
            pDst += dst_pitch;
            pMCB += MCB_pitch;
            pMCF += MCF_pitch;
            pRef += ref_pitch;
            pSrc += src_pitch;
        }
    } else if (mode == 3) {
        for (int h = 0; h < nBlkSizeY; h++) {
            for (int w = 0; w < nBlkSizeX; w++)
                pDst[w] = ( ( (maskB[w] * pMCF[w] + (255 - maskB[w]) * pMCB[w] + 255) >> 8 ) * time256 +
                            ( (maskF[w] * pMCB[w] + (255 - maskF[w]) * pMCF[w] + 255) >> 8 ) * (256 - time256) ) >> 8;
            pDst += dst_pitch;
            pMCB += MCB_pitch;
            pMCF += MCF_pitch;
            pRef += ref_pitch;
            pSrc += src_pitch;
            maskB += mask_pitch;
            maskF += mask_pitch;
        }
    } else if (mode == 4) {
        for (int h = 0; h < nBlkSizeY; h++) {
            for (int w = 0; w < nBlkSizeX; w++) {
                int f = (maskF[w] * pMCB[w] + (255 - maskF[w]) * pMCF[w] + 255 ) >> 8;
                int b = (maskB[w] * pMCF[w] + (255 - maskB[w]) * pMCB[w] + 255) >> 8;
                int avg = (pRef[w] * time256 + pSrc[w] * (256 - time256) + 255) >> 8; // simple temporal non-MC average
                int m = ( b * time256 + f * (256 - time256) ) >> 8;
                pDst[w]= ( avg * pOcc[w] + m * (255 - pOcc[w]) + 255 ) >> 8;
            }
            pDst += dst_pitch;
            pMCB += MCB_pitch;
            pMCF += MCF_pitch;
            pRef += ref_pitch;
            pSrc += src_pitch;
            maskB += mask_pitch;
            maskF += mask_pitch;
            pOcc += mask_pitch;
        }
    } else if (mode == 5) {
        for (int h = 0; h < nBlkSizeY; h++) {
            for (int w = 0; w < nBlkSizeX; w++) {
                pDst[w] = pOcc[w];
            }
            pDst += dst_pitch;
            pOcc += mask_pitch;
        }
    }
}


static const VSFrameRef *VS_CC mvblockfpsGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVBlockFPSData *d = (MVBlockFPSData *) * instanceData;

    if (activationReason == arInitial) {
        int off = d->mvClipB->GetDeltaFrame(); // integer offset of reference frame

        int nleft = (int)(n * d->fa / d->fb);
        int nright = nleft + off;

        int time256 = int( (double(n) * double(d->fa) / double(d->fb) - nleft) * 256 + 0.5);
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
            vsapi->requestFrameFilter(nright, d->mvfw, frameCtx); // requests nleft, nleft + off
            vsapi->requestFrameFilter(nleft, d->mvbw, frameCtx); // requests nleft, nleft + off

            vsapi->requestFrameFilter(nleft, d->super, frameCtx);
            vsapi->requestFrameFilter(nright, d->super, frameCtx);
        } 

        vsapi->requestFrameFilter(d->vi.numFrames ? VSMIN(nleft, d->vi.numFrames - 1) : nleft, d->node, frameCtx);

        if (d->blend)
            vsapi->requestFrameFilter(d->vi.numFrames ? VSMIN(nright, d->vi.numFrames - 1) : nright, d->node, frameCtx);

    } else if (activationReason == arAllFramesReady) {
        int nleft = (int)(n * d->fa / d->fb);
        // intermediate product may be very large! Now I know how to multiply int64
        int time256 = int( (double(n)*double(d->fa)/double(d->fb) - nleft) * 256 + 0.5);

        int off = d->mvClipB->GetDeltaFrame(); // integer offset of reference frame
        // usually off must be = 1
        if (off > 1)
            time256 = time256 / off;

        int nright = nleft + off;

        if (time256 == 0)
            return vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nleft, d->vi.numFrames - 1) : nleft, d->node, frameCtx); // simply left
        else if (time256 == 256)
            return vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nright, d->vi.numFrames - 1) : nright, d->node, frameCtx); // simply right

        MVClipBalls ballsF(d->mvClipF, vsapi);
        MVClipBalls ballsB(d->mvClipB, vsapi);

        bool isUsableF = false;
        bool isUsableB = false;

        if ((nleft < d->vi.numFrames && nright < d->vi.numFrames) || !d->vi.numFrames) {
            const VSFrameRef *mvF = vsapi->getFrameFilter(nright, d->mvfw, frameCtx);
            ballsF.Update(mvF);// forward from current to next
            isUsableF = ballsF.IsUsable();
            vsapi->freeFrame(mvF);

            const VSFrameRef *mvB = vsapi->getFrameFilter(nleft, d->mvbw, frameCtx);
            ballsB.Update(mvB);// backward from next to current
            isUsableB = ballsB.IsUsable();
            vsapi->freeFrame(mvB);
        }

        const int nWidth = d->bleh->nWidth;
        const int nHeight = d->bleh->nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int nHeightP = d->nHeightP;
        const int nHeightPUV = d->nHeightPUV;
        const int mode = d->mode;
        const int thres = d->thres;
        const int blend = d->blend;
        const int isse = d->isse;
        const int yRatioUV = d->bleh->yRatioUV;
        const int nBlkX = d->bleh->nBlkX;
        const int nBlkY = d->bleh->nBlkY;
        const int nBlkSizeX = d->bleh->nBlkSizeX;
        const int nBlkSizeY = d->bleh->nBlkSizeY;
        const int nPel = d->bleh->nPel;
        const int nPitchY = d->nPitchY;
        const int nPitchUV = d->nPitchUV;
        const int nBlkXP = d->nBlkXP;
        const int nBlkYP = d->nBlkYP;
        SimpleResize *upsizer = d->upsizer;
        SimpleResize *upsizerUV = d->upsizerUV;

        const uint8_t *OnesBlock = d->OnesBlock;

        const int nSuperHPad = d->nSuperHPad;
        const int nSuperVPad = d->nSuperVPad;
        const int nSuperModeYUV = d->nSuperModeYUV;
        const int nSuperLevels = d->nSuperLevels;
        const int nSuperPel = d->nSuperPel;


        if (isUsableB && isUsableF) {
            uint8_t *pDst[3];
            const uint8_t *pRef[3], *pSrc[3];
            int nDstPitches[3], nRefPitches[3], nSrcPitches[3];

            // If both are usable, that means both nleft and nright are less than numFrames, or that we don't have numFrames. Thus there is no need to check nleft and nright here.
            const VSFrameRef *src = vsapi->getFrameFilter(nleft, d->super, frameCtx);
            const VSFrameRef *ref = vsapi->getFrameFilter(nright, d->super, frameCtx);//  right frame for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

            for (int i = 0; i < 3; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
            }

            MVGroupOfFrames *pRefBGOF = new MVGroupOfFrames(nSuperLevels, nWidth, nHeight, nSuperPel, nSuperHPad, nSuperVPad, nSuperModeYUV, isse, yRatioUV);
            MVGroupOfFrames *pRefFGOF = new MVGroupOfFrames(nSuperLevels, nWidth, nHeight, nSuperPel, nSuperHPad, nSuperVPad, nSuperModeYUV, isse, yRatioUV);

            pRefBGOF->Update(YUVPLANES, (uint8_t*)pRef[0], nRefPitches[0], (uint8_t*)pRef[1], nRefPitches[1], (uint8_t*)pRef[2], nRefPitches[2]);// v2.0
            pRefFGOF->Update(YUVPLANES, (uint8_t*)pSrc[0], nSrcPitches[0], (uint8_t*)pSrc[1], nSrcPitches[1], (uint8_t*)pSrc[2], nSrcPitches[2]);

            MVPlane *pPlanesB[3];
            MVPlane *pPlanesF[3];

            pPlanesB[0] = pRefBGOF->GetFrame(0)->GetPlane(YPLANE);
            pPlanesB[1] = pRefBGOF->GetFrame(0)->GetPlane(UPLANE);
            pPlanesB[2] = pRefBGOF->GetFrame(0)->GetPlane(VPLANE);

            pPlanesF[0] = pRefFGOF->GetFrame(0)->GetPlane(YPLANE);
            pPlanesF[1] = pRefFGOF->GetFrame(0)->GetPlane(UPLANE);
            pPlanesF[2] = pRefFGOF->GetFrame(0)->GetPlane(VPLANE);

            uint8_t *MaskFullYB = new uint8_t [nHeightP * nPitchY];
            uint8_t *MaskFullUVB = new uint8_t [nHeightPUV * nPitchUV];
            uint8_t *MaskFullYF = new uint8_t [nHeightP * nPitchY];
            uint8_t *MaskFullUVF = new uint8_t [nHeightPUV * nPitchUV];

            uint8_t *MaskOccY = new uint8_t [nHeightP * nPitchY];
            uint8_t *MaskOccUV = new uint8_t [nHeightPUV * nPitchUV];

            uint8_t *smallMaskB = NULL;
            uint8_t *smallMaskF = NULL;
            uint8_t *smallMaskO = NULL;

            memset(MaskFullYB, 0, nHeightP * nPitchY);
            memset(MaskFullYF, 0, nHeightP * nPitchY);

            int blocks = d->mvClipB->GetBlkCount();

            int maxoffset = nPitchY * (nHeightP - nBlkSizeY) - nBlkSizeX;

            if (mode == 3 || mode == 4 || mode == 5) {
                smallMaskB = new uint8_t [nBlkXP * nBlkYP];
                smallMaskF = new uint8_t [nBlkXP * nBlkYP];
                smallMaskO = new uint8_t [nBlkXP * nBlkYP];

                // make forward shifted images by projection to build occlusion mask
                for ( int i = 0; i < blocks; i++ ) {
                    const FakeBlockData &blockF = ballsF.GetBlock(0, i);
                    int offset = blockF.GetX() - ((blockF.GetMV().x * time256) >> 8) / nPel + (blockF.GetY() - ((blockF.GetMV().y * time256) >> 8) / nPel) * nPitchY;
                    if (offset >= 0 && offset < maxoffset)
                        d->BLITLUMA(MaskFullYF + offset, nPitchY, OnesBlock, nBlkSizeX); // fill by ones
                }
                //  same mask for backward
                for ( int i = 0; i < blocks; i++ ) {
                    const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                    int offset = blockB.GetX() - ((blockB.GetMV().x * (256 - time256)) >> 8) / nPel + (blockB.GetY() - ((blockB.GetMV().y * (256 - time256)) >> 8) / nPel) * nPitchY;
                    if (offset >= 0 && offset < maxoffset)
                        d->BLITLUMA(MaskFullYB + offset, nPitchY, OnesBlock, nBlkSizeX); // fill by ones
                }

                // make small binary mask from  occlusion  regions
                MakeSmallMask(MaskFullYF, nPitchY, smallMaskF, nBlkXP, nBlkYP, nBlkSizeX, nBlkSizeY, thres);
                InflateMask(smallMaskF, nBlkXP, nBlkYP);
                // upsize small mask to full frame size
                upsizer->Resize(MaskFullYF, nPitchY, smallMaskF, nBlkXP);
                upsizerUV->Resize(MaskFullUVF, nPitchUV, smallMaskF, nBlkXP);
                // now we have forward fullframe blured occlusion mask in maskF arrays

                // make small binary mask from  occlusion  regions
                MakeSmallMask(MaskFullYB, nPitchY, smallMaskB, nBlkXP, nBlkYP, nBlkSizeX, nBlkSizeY, thres);
                InflateMask(smallMaskB, nBlkXP, nBlkYP);
                // upsize small mask to full frame size
                upsizer->Resize(MaskFullYB, nPitchY, smallMaskB, nBlkXP);
                upsizerUV->Resize(MaskFullUVB, nPitchUV, smallMaskB, nBlkXP);
            }

            if (mode == 4 || mode == 5) {
                // make final (both directions) occlusion mask
                MultMasks(smallMaskF, smallMaskB, smallMaskO,  nBlkXP, nBlkYP);
                InflateMask(smallMaskO, nBlkXP, nBlkYP);
                // upsize small mask to full frame size
                upsizer->Resize(MaskOccY, nPitchY, smallMaskO, nBlkXP);
                upsizerUV->Resize(MaskOccUV, nPitchUV, smallMaskO, nBlkXP);
            }

            // pointers
            uint8_t * pMaskFullYB = MaskFullYB;
            uint8_t * pMaskFullYF = MaskFullYF;
            uint8_t * pMaskFullUVB = MaskFullUVB;
            uint8_t * pMaskFullUVF = MaskFullUVF;
            uint8_t * pMaskOccY = MaskOccY;
            uint8_t * pMaskOccUV = MaskOccUV;

            pSrc[0] += nSuperHPad + nSrcPitches[0] * nSuperVPad; // add offset source in super
            pSrc[1] += (nSuperHPad >> 1) + nSrcPitches[1] * (nSuperVPad >> 1);
            pSrc[2] += (nSuperHPad >> 1) + nSrcPitches[2] * (nSuperVPad >> 1);
            pRef[0] += nSuperHPad + nRefPitches[0] * nSuperVPad;
            pRef[1] += (nSuperHPad >> 1) + nRefPitches[1] * (nSuperVPad >> 1);
            pRef[2] += (nSuperHPad >> 1) + nRefPitches[2] * (nSuperVPad >> 1);

            // fetch image blocks
            for ( int i = 0; i < blocks; i++ ) {
                const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                const FakeBlockData &blockF = ballsF.GetBlock(0, i);

                // luma
                ResultBlock(pDst[0], nDstPitches[0],
                        pPlanesB[0]->GetPointer(blockB.GetX() * nPel + ((blockB.GetMV().x * (256 - time256)) >> 8), blockB.GetY() * nPel + ((blockB.GetMV().y * (256 - time256)) >> 8)),
                        pPlanesB[0]->GetPitch(),
                        pPlanesF[0]->GetPointer(blockF.GetX() * nPel + ((blockF.GetMV().x * time256) >> 8), blockF.GetY() * nPel + ((blockF.GetMV().y * time256) >> 8)),
                        pPlanesF[0]->GetPitch(),
                        pRef[0], nRefPitches[0],
                        pSrc[0], nSrcPitches[0],
                        pMaskFullYB, nPitchY,
                        pMaskFullYF, pMaskOccY,
                        nBlkSizeX, nBlkSizeY, time256, mode);
                // chroma u
                if (nSuperModeYUV & UPLANE)
                    ResultBlock(pDst[1], nDstPitches[1],
                            pPlanesB[1]->GetPointer((blockB.GetX() * nPel + ((blockB.GetMV().x * (256 - time256)) >> 8)) >> 1, (blockB.GetY() * nPel + ((blockB.GetMV().y * (256 - time256)) >> 8)) / yRatioUV),
                            pPlanesB[1]->GetPitch(),
                            pPlanesF[1]->GetPointer((blockF.GetX() * nPel + ((blockF.GetMV().x * time256) >> 8)) >> 1, (blockF.GetY() * nPel + ((blockF.GetMV().y * time256) >> 8)) / yRatioUV),
                            pPlanesF[1]->GetPitch(),
                            pRef[1], nRefPitches[1],
                            pSrc[1], nSrcPitches[1],
                            pMaskFullUVB, nPitchUV,
                            pMaskFullUVF, pMaskOccUV,
                            nBlkSizeX >> 1, nBlkSizeY / yRatioUV, time256, mode);
                // chroma v
                if (nSuperModeYUV & VPLANE)
                    ResultBlock(pDst[2], nDstPitches[2],
                            pPlanesB[2]->GetPointer((blockB.GetX() * nPel + ((blockB.GetMV().x * (256 - time256)) >> 8)) >> 1, (blockB.GetY() * nPel + ((blockB.GetMV().y * (256 - time256)) >> 8)) / yRatioUV),
                            pPlanesB[2]->GetPitch(),
                            pPlanesF[2]->GetPointer((blockF.GetX() * nPel + ((blockF.GetMV().x * time256) >> 8)) >> 1, (blockF.GetY() * nPel + ((blockF.GetMV().y * time256) >> 8)) / yRatioUV),
                            pPlanesF[2]->GetPitch(),
                            pRef[2], nRefPitches[2],
                            pSrc[2], nSrcPitches[2],
                            pMaskFullUVB, nPitchUV,
                            pMaskFullUVF, pMaskOccUV,
                            nBlkSizeX >> 1, nBlkSizeY / yRatioUV, time256, mode);


                // update pDsts
                pDst[0] += nBlkSizeX;
                pDst[1] += nBlkSizeX >> 1;
                pDst[2] += nBlkSizeX >> 1;
                pRef[0] += nBlkSizeX;
                pRef[1] += nBlkSizeX >> 1;
                pRef[2] += nBlkSizeX >> 1;
                pSrc[0] += nBlkSizeX;
                pSrc[1] += nBlkSizeX >> 1;
                pSrc[2] += nBlkSizeX >> 1;
                pMaskFullYB += nBlkSizeX;
                pMaskFullUVB += nBlkSizeX >> 1;
                pMaskFullYF += nBlkSizeX;
                pMaskFullUVF += nBlkSizeX >> 1;
                pMaskOccY += nBlkSizeX;
                pMaskOccUV += nBlkSizeX >> 1;


                if ( !((i + 1) % nBlkX)  ) {
                    // blend rest right with time weight
                    Blend(pDst[0], pSrc[0], pRef[0], nBlkSizeY, nWidth - nBlkSizeX * nBlkX, nDstPitches[0], nSrcPitches[0], nRefPitches[0], time256, isse);
                    if (nSuperModeYUV & UPLANE)
                        Blend(pDst[1], pSrc[1], pRef[1], nBlkSizeY / yRatioUV, nWidthUV - (nBlkSizeX >> 1) * nBlkX, nDstPitches[1], nSrcPitches[1], nRefPitches[1], time256, isse);
                    if (nSuperModeYUV & VPLANE)
                        Blend(pDst[2], pSrc[2], pRef[2], nBlkSizeY / yRatioUV, nWidthUV - (nBlkSizeX >> 1) * nBlkX, nDstPitches[2], nSrcPitches[2], nRefPitches[2], time256, isse);

                    pDst[0] += nBlkSizeY * nDstPitches[0] - nBlkSizeX * nBlkX;
                    pDst[1] += ( nBlkSizeY / yRatioUV ) * nDstPitches[1] - (nBlkSizeX >> 1) * nBlkX;
                    pDst[2] += ( nBlkSizeY / yRatioUV ) * nDstPitches[2] - (nBlkSizeX >> 1) * nBlkX;
                    pRef[0] += nBlkSizeY * nRefPitches[0] - nBlkSizeX * nBlkX;
                    pRef[1] += ( nBlkSizeY / yRatioUV ) * nRefPitches[1] - (nBlkSizeX >> 1) * nBlkX;
                    pRef[2] += ( nBlkSizeY / yRatioUV ) * nRefPitches[2] - (nBlkSizeX >> 1) * nBlkX;
                    pSrc[0] += nBlkSizeY * nSrcPitches[0] - nBlkSizeX * nBlkX;
                    pSrc[1] += ( nBlkSizeY / yRatioUV ) * nSrcPitches[1] - (nBlkSizeX >> 1) * nBlkX;
                    pSrc[2] += ( nBlkSizeY / yRatioUV ) * nSrcPitches[2] - (nBlkSizeX >> 1) * nBlkX;
                    pMaskFullYB += nBlkSizeY * nPitchY - nBlkSizeX * nBlkX;
                    pMaskFullUVB += (nBlkSizeY / yRatioUV) * nPitchUV - (nBlkSizeX >> 1) * nBlkX;
                    pMaskFullYF += nBlkSizeY * nPitchY - nBlkSizeX * nBlkX;
                    pMaskFullUVF += (nBlkSizeY / yRatioUV) * nPitchUV - (nBlkSizeX >> 1) * nBlkX;
                    pMaskOccY += nBlkSizeY * nPitchY - nBlkSizeX * nBlkX;
                    pMaskOccUV += (nBlkSizeY / yRatioUV) * nPitchUV - (nBlkSizeX >> 1) * nBlkX;
                }
            }
            // blend rest bottom with time weight
            Blend(pDst[0], pSrc[0], pRef[0], nHeight - nBlkSizeY * nBlkY, nWidth, nDstPitches[0], nSrcPitches[0], nRefPitches[0], time256, isse);
            if (nSuperModeYUV & UPLANE)
                Blend(pDst[1], pSrc[1], pRef[1], nHeightUV - (nBlkSizeY / yRatioUV) * nBlkY, nWidthUV, nDstPitches[1], nSrcPitches[1], nRefPitches[1], time256, isse);
            if (nSuperModeYUV & VPLANE)
                Blend(pDst[2], pSrc[2], pRef[2], nHeightUV - (nBlkSizeY / yRatioUV) * nBlkY, nWidthUV, nDstPitches[2], nSrcPitches[2], nRefPitches[2], time256, isse);


            delete [] MaskFullYB;
            delete [] MaskFullUVB;
            delete [] MaskFullYF;
            delete [] MaskFullUVF;
            delete [] MaskOccY;
            delete [] MaskOccUV;
            if (smallMaskB) {
                delete [] smallMaskB;
                delete [] smallMaskF;
                delete [] smallMaskO;
            }

            delete pRefBGOF;
            delete pRefFGOF;

            vsapi->freeFrame(src);
            vsapi->freeFrame(ref);

            return dst;
        } else { // poor estimation
            const VSFrameRef *src = vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nleft, d->vi.numFrames - 1) : nleft, d->node, frameCtx);

            if (blend) {//let's blend src with ref frames like ConvertFPS
                uint8_t *pDst[3];
                const uint8_t *pRef[3], *pSrc[3];
                int nDstPitches[3], nRefPitches[3], nSrcPitches[3];

                const VSFrameRef *ref = vsapi->getFrameFilter(d->vi.numFrames ? VSMIN(nright, d->vi.numFrames - 1) : nright, d->node, frameCtx);

                VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

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
            } else {
                return src; // like ChangeFPS
            }
        }
    }

    return 0;
}


static void VS_CC mvblockfpsFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVBlockFPSData *d = (MVBlockFPSData *)instanceData;

    delete d->mvClipB;
    delete d->mvClipF;

    delete d->bleh;

    delete d->upsizer;
    delete d->upsizerUV;

    delete [] d->OnesBlock;

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
    const int nBlkSizeX = d->bleh->nBlkSizeX;
    const int nBlkSizeY = d->bleh->nBlkSizeY;

    COPYFunction copys[33][33];

    copys[2][2] = d->isse ? mvtools_Copy2x2_sse2 : Copy_C<2,2>;
    copys[2][4] = d->isse ? mvtools_Copy2x4_sse2 : Copy_C<2,4>;
    copys[4][2] = d->isse ? mvtools_Copy4x2_sse2 : Copy_C<4,2>;
    copys[4][4] = d->isse ? mvtools_Copy4x4_sse2 : Copy_C<4,4>;
    copys[4][8] = d->isse ? mvtools_Copy4x8_sse2 : Copy_C<4,8>;
    copys[8][1] = d->isse ? mvtools_Copy8x1_sse2 : Copy_C<8,1>;
    copys[8][2] = d->isse ? mvtools_Copy8x2_sse2 : Copy_C<8,2>;
    copys[8][4] = d->isse ? mvtools_Copy8x4_sse2 : Copy_C<8,4>;
    copys[8][8] = d->isse ? mvtools_Copy8x8_sse2 : Copy_C<8,8>;
    copys[8][16] = d->isse ? mvtools_Copy8x16_sse2 : Copy_C<8,16>;
    copys[16][2] = d->isse ? mvtools_Copy16x2_sse2 : Copy_C<16,2>;
    copys[16][8] = d->isse ? mvtools_Copy16x8_sse2 : Copy_C<16,8>;
    copys[16][16] = d->isse ? mvtools_Copy16x16_sse2 : Copy_C<16,16>;
    copys[16][32] = d->isse ? mvtools_Copy16x32_sse2 : Copy_C<16,32>;
    copys[32][16] = d->isse ? mvtools_Copy32x16_sse2 : Copy_C<32,16>;
    copys[32][32] = d->isse ? mvtools_Copy32x32_sse2 : Copy_C<32,32>;

    d->BLITLUMA = copys[nBlkSizeX][nBlkSizeY];
}


static void VS_CC mvblockfpsCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVBlockFPSData d;
    MVBlockFPSData *data;

    int err, thres_err;

    d.num = vsapi->propGetInt(in, "num", 0, &err);
    if (err)
        d.num = 25;

    d.den = vsapi->propGetInt(in, "den", 0, &err);
    if (err)
        d.den = 1;

    d.mode = vsapi->propGetInt(in, "mode", 0, &err);

    d.thres = vsapi->propGetInt(in, "thres", 0, &thres_err);
    // The default value is computed later.

    d.blend = !!vsapi->propGetInt(in, "blend", 0, &err);
    if (err)
        d.blend = 1;

    d.thscd1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = vsapi->propGetInt(in, "thscd2", 0, &err);
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.isse = vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;


    if (d.mode < 0 || d.mode > 5) {
        vsapi->setError(out, "BlockFPS: mode must be between 0 and 5 (inclusive).");
        return;
    }


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("BlockFPS: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[6];
    int nHeightS = vsapi->propGetInt(props, "Super height", 0, &evil_err[0]);
    d.nSuperHPad = vsapi->propGetInt(props, "Super hpad", 0, &evil_err[1]);
    d.nSuperVPad = vsapi->propGetInt(props, "Super vpad", 0, &evil_err[2]);
    d.nSuperPel = vsapi->propGetInt(props, "Super pel", 0, &evil_err[3]);
    d.nSuperModeYUV = vsapi->propGetInt(props, "Super modeyuv", 0, &evil_err[4]);
    d.nSuperLevels = vsapi->propGetInt(props, "Super levels", 0, &evil_err[5]);
    vsapi->freeFrame(evil);

    for (int i = 0; i < 6; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "BlockFPS: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
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
        vsapi->setError(out, "BlockFPS: mvbw and mvfw must be generated with the same delta.");
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
            vsapi->setError(out, "BlockFPS: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "BlockFPS: mvfw must be generated with isb=False.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    try {
        d.bleh = new MVFilter(d.mvfw, "BlockFPS", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    if (thres_err)
        d.thres = d.bleh->nBlkSizeX * d.bleh->nBlkSizeY / 4;

    if (d.bleh->nOverlapX || d.bleh->nOverlapY) {
        vsapi->setError(out, "BlockFPS: Overlap must be 0.");
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
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

    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = *vsapi->getVideoInfo(d.node);


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


    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.bleh->nHeight != nHeightS ||
            d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2 ||
            d.bleh->nWidth != d.vi.width ||
            d.bleh->nHeight != d.vi.height) {
        vsapi->setError(out, "BlockFPS: wrong source or super clip frame size.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    int id = d.vi.format->id;
    if (!isConstantFormat(&d.vi) || (id != pfYUV420P8 && id != pfYUV422P8)) {
        vsapi->setError(out, "BlockFPS: input clip must be YUV420P8 or YUV422P8, with constant dimensions.");
        vsapi->freeNode(d.super);
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
    d.nWidthUV = d.bleh->nWidth / 2; // for YV12

    d.nPitchY = (d.nWidthP + 15) & (~15);
    d.nPitchUV = (d.nWidthPUV + 15) & (~15);



    d.OnesBlock = new uint8_t [d.bleh->nBlkSizeX * d.bleh->nBlkSizeY + 32]; // padding seems unnecessary
    memset(d.OnesBlock, 255, d.bleh->nBlkSizeX * d.bleh->nBlkSizeY);


    d.upsizer = new SimpleResize(d.nWidthP, d.nHeightP, d.nBlkXP, d.nBlkYP);
    d.upsizerUV = new SimpleResize(d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP);

    selectFunctions(&d);


    data = (MVBlockFPSData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "BlockFPS", mvblockfpsInit, mvblockfpsGetFrame, mvblockfpsFree, fmParallel, 0, data, core);
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
            "thres:int:opt;"
            "blend:int:opt;"
            "thscd1:int:opt;"
            "thscd2:int:opt;"
            "isse:int:opt;"
            , mvblockfpsCreate, 0, plugin);
}
