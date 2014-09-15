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

#include <VapourSynth.h>
#include <VSHelper.h>

#include "MVDegrain3.h"
#include "MVInterface.h"
#include "CopyCode.h"
#include "overlap.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;
    VSNodeRef *mvbw2;
    VSNodeRef *mvfw2;
    VSNodeRef *mvbw3;
    VSNodeRef *mvfw3;

    int thSAD;
    int thSADC;
    int YUVplanes;
    int nLimit;
    int nLimitC;
    int nSCD1;
    int nSCD2;
    int isse;

    MVClipDicks *mvClipB;
    MVClipDicks *mvClipF;
    MVClipDicks *mvClipB2;
    MVClipDicks *mvClipF2;
    MVClipDicks *mvClipB3;
    MVClipDicks *mvClipF3;

    MVFilter *bleh;

    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;

    int dstShortPitch;

    OverlapsFunction *OVERSLUMA;
    OverlapsFunction *OVERSCHROMA;
    Denoise3Function *DEGRAINLUMA;
    Denoise3Function *DEGRAINCHROMA;
} MVDegrain3Data;


static void VS_CC mvdegrain3Init(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVDegrain3Data *d = (MVDegrain3Data *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvdegrain3GetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVDegrain3Data *d = (MVDegrain3Data *) * instanceData;

    if (activationReason == arInitial) {
        int offF3 = -1 * d->mvClipF3->GetDeltaFrame();
        int offF2 = -1 * d->mvClipF2->GetDeltaFrame();
        int offF = -1 * d->mvClipF->GetDeltaFrame();
        int offB = d->mvClipB->GetDeltaFrame();
        int offB2 = d->mvClipB2->GetDeltaFrame();
        int offB3 = d->mvClipB3->GetDeltaFrame();

        vsapi->requestFrameFilter(n, d->mvfw3, frameCtx);
        vsapi->requestFrameFilter(n, d->mvfw2, frameCtx);
        vsapi->requestFrameFilter(n, d->mvfw, frameCtx);
        vsapi->requestFrameFilter(n, d->mvbw, frameCtx);
        vsapi->requestFrameFilter(n, d->mvbw2, frameCtx);
        vsapi->requestFrameFilter(n, d->mvbw3, frameCtx);

        if (n + offF3 >= 0)
            vsapi->requestFrameFilter(n + offF3, d->super, frameCtx);
        if (n + offF2 >= 0)
            vsapi->requestFrameFilter(n + offF2, d->super, frameCtx);
        if (n + offF >= 0)
            vsapi->requestFrameFilter(n + offF, d->super, frameCtx);
        if (n + offB < d->vi->numFrames)
            vsapi->requestFrameFilter(n + offB, d->super, frameCtx);
        if (n + offB2 < d->vi->numFrames)
            vsapi->requestFrameFilter(n + offB2, d->super, frameCtx);
        if (n + offB3 < d->vi->numFrames)
            vsapi->requestFrameFilter(n + offB3, d->super, frameCtx);

        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);


        uint8_t *pDst[3], *pDstCur[3];
        const uint8_t *pSrcCur[3];
        const uint8_t *pSrc[3];
        const uint8_t *pRefB[3];
        const uint8_t *pRefF[3];
        const uint8_t *pRefB2[3];
        const uint8_t *pRefF2[3];
        const uint8_t *pRefB3[3];
        const uint8_t *pRefF3[3];
        int nDstPitches[3], nSrcPitches[3];
        int nRefBPitches[3], nRefFPitches[3];
        int nRefB2Pitches[3], nRefF2Pitches[3];
        int nRefB3Pitches[3], nRefF3Pitches[3];
        bool isUsableB, isUsableF, isUsableB2, isUsableF2, isUsableB3, isUsableF3;
        int tmpPitch = d->bleh->nBlkSizeX;
        int WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3;
        unsigned short *pDstShort;

        const VSFrameRef *mvF3 = vsapi->getFrameFilter(n, d->mvfw3, frameCtx);
        MVClipBalls ballsF3(d->mvClipF3, vsapi);
        ballsF3.Update(mvF3);
        isUsableF3 = ballsF3.IsUsable();
        vsapi->freeFrame(mvF3);

        const VSFrameRef *mvF2 = vsapi->getFrameFilter(n, d->mvfw2, frameCtx);
        MVClipBalls ballsF2(d->mvClipF2, vsapi);
        ballsF2.Update(mvF2);
        isUsableF2 = ballsF2.IsUsable();
        vsapi->freeFrame(mvF2);

        const VSFrameRef *mvF = vsapi->getFrameFilter(n, d->mvfw, frameCtx);
        MVClipBalls ballsF(d->mvClipF, vsapi);
        ballsF.Update(mvF);
        isUsableF = ballsF.IsUsable();
        vsapi->freeFrame(mvF);

        const VSFrameRef *mvB = vsapi->getFrameFilter(n, d->mvbw, frameCtx);
        MVClipBalls ballsB(d->mvClipB, vsapi);
        ballsB.Update(mvB);
        isUsableB = ballsB.IsUsable();
        vsapi->freeFrame(mvB);

        const VSFrameRef *mvB2 = vsapi->getFrameFilter(n, d->mvbw2, frameCtx);
        MVClipBalls ballsB2(d->mvClipB2, vsapi);
        ballsB2.Update(mvB2);
        isUsableB2 = ballsB2.IsUsable();
        vsapi->freeFrame(mvB2);

        const VSFrameRef *mvB3 = vsapi->getFrameFilter(n, d->mvbw3, frameCtx);
        MVClipBalls ballsB3(d->mvClipB3, vsapi);
        ballsB3.Update(mvB3);
        isUsableB3 = ballsB3.IsUsable();
        vsapi->freeFrame(mvB3);

        const VSFrameRef *refB = NULL;
        const VSFrameRef *refF = NULL;
        const VSFrameRef *refB2 = NULL;
        const VSFrameRef *refF2 = NULL;
        const VSFrameRef *refB3 = NULL;
        const VSFrameRef *refF3 = NULL;

        if (isUsableF3)
        {
            int offF3 = -1 * d->mvClipF3->GetDeltaFrame();
            refF3 = vsapi->getFrameFilter(n + offF3, d->super, frameCtx);
        }
        if (isUsableF2)
        {
            int offF2 = -1 * d->mvClipF2->GetDeltaFrame();
            refF2 = vsapi->getFrameFilter(n + offF2, d->super, frameCtx);
        }
        if (isUsableF)
        {
            int offF = -1 * d->mvClipF->GetDeltaFrame();
            refF = vsapi->getFrameFilter(n + offF, d->super, frameCtx);
        }
        if (isUsableB)
        {
            int offB = d->mvClipB->GetDeltaFrame();
            refB = vsapi->getFrameFilter(n + offB, d->super, frameCtx);
        }
        if (isUsableB2)
        {
            int offB2 = d->mvClipB2->GetDeltaFrame();
            refB2 = vsapi->getFrameFilter(n + offB2, d->super, frameCtx);
        }
        if (isUsableB3)
        {
            int offB3 = d->mvClipB3->GetDeltaFrame();
            refB3 = vsapi->getFrameFilter(n + offB3, d->super, frameCtx);
        }

        for (int i = 0; i < 3; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
            pSrc[i] = vsapi->getReadPtr(src, i);
            nSrcPitches[i] = vsapi->getStride(src, i);
            if (isUsableF3) {
                pRefF3[i] = vsapi->getReadPtr(refF3, i);
                nRefF3Pitches[i] = vsapi->getStride(refF3, i);
            }
            if (isUsableF2) {
                pRefF2[i] = vsapi->getReadPtr(refF2, i);
                nRefF2Pitches[i] = vsapi->getStride(refF2, i);
            }
            if (isUsableF) {
                pRefF[i] = vsapi->getReadPtr(refF, i);
                nRefFPitches[i] = vsapi->getStride(refF, i);
            }
            if (isUsableB) {
                pRefB[i] = vsapi->getReadPtr(refB, i);
                nRefBPitches[i] = vsapi->getStride(refB, i);
            }
            if (isUsableB2) {
                pRefB2[i] = vsapi->getReadPtr(refB2, i);
                nRefB2Pitches[i] = vsapi->getStride(refB2, i);
            }
            if (isUsableB3) {
                pRefB3[i] = vsapi->getReadPtr(refB3, i);
                nRefB3Pitches[i] = vsapi->getStride(refB3, i);
            }
        }

        const int nWidth = d->bleh->nWidth;
        const int nHeight = d->bleh->nHeight;
        const int yRatioUV = d->bleh->yRatioUV;
        const int nOverlapX = d->bleh->nOverlapX;
        const int nOverlapY = d->bleh->nOverlapY;
        const int nBlkSizeX = d->bleh->nBlkSizeX;
        const int nBlkSizeY = d->bleh->nBlkSizeY;
        const int nBlkX = d->bleh->nBlkX;
        const int nBlkY = d->bleh->nBlkY;
        const int isse = d->isse;
        const int thSAD = d->thSAD;
        const int thSADC = d->thSADC;
        const int YUVplanes = d->YUVplanes;
        const int dstShortPitch = d->dstShortPitch;
        const int nLimit = d->nLimit;
        const int nLimitC = d->nLimitC;
        const int nSuperModeYUV = d->nSuperModeYUV;
        const int nPel = d->bleh->nPel;


        MVGroupOfFrames *pRefBGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefFGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefB2GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefF2GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefB3GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefF3GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);


        short *winOver;
        short *winOverUV;

        OverlapWindows *OverWins = NULL;
        OverlapWindows *OverWinsUV = NULL;
        unsigned short *DstShort = NULL;
        if (nOverlapX > 0 || nOverlapY > 0)
        {
            OverWins = new OverlapWindows(nBlkSizeX, nBlkSizeY, nOverlapX, nOverlapY);
            OverWinsUV = new OverlapWindows(nBlkSizeX / 2, nBlkSizeY / yRatioUV, nOverlapX / 2, nOverlapY / yRatioUV);
            DstShort = new unsigned short[d->dstShortPitch * nHeight];
        }

        uint8_t *tmpBlock = new uint8_t[32*32];

        int nWidth_B = nBlkX*(nBlkSizeX - nOverlapX) + nOverlapX;
        int nHeight_B = nBlkY*(nBlkSizeY - nOverlapY) + nOverlapY;

        MVPlane *pPlanesB[3] = { };
        MVPlane *pPlanesF[3] = { };
        MVPlane *pPlanesB2[3] = { };
        MVPlane *pPlanesF2[3] = { };
        MVPlane *pPlanesB3[3] = { };
        MVPlane *pPlanesF3[3] = { };

        if (isUsableF3)
        {
            pRefF3GOF->Update(YUVplanes, (uint8_t*)pRefF3[0], nRefF3Pitches[0], (uint8_t*)pRefF3[1], nRefF3Pitches[1], (uint8_t*)pRefF3[2], nRefF3Pitches[2]);
            if (YUVplanes & YPLANE)
                pPlanesF3[0] = pRefF3GOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesF3[1] = pRefF3GOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesF3[2] = pRefF3GOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableF2)
        {
            pRefF2GOF->Update(YUVplanes, (uint8_t*)pRefF2[0], nRefF2Pitches[0], (uint8_t*)pRefF2[1], nRefF2Pitches[1], (uint8_t*)pRefF2[2], nRefF2Pitches[2]);
            if (YUVplanes & YPLANE)
                pPlanesF2[0] = pRefF2GOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesF2[1] = pRefF2GOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesF2[2] = pRefF2GOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableF)
        {
            pRefFGOF->Update(YUVplanes, (uint8_t*)pRefF[0], nRefFPitches[0], (uint8_t*)pRefF[1], nRefFPitches[1], (uint8_t*)pRefF[2], nRefFPitches[2]);
            if (YUVplanes & YPLANE)
                pPlanesF[0] = pRefFGOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesF[1] = pRefFGOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesF[2] = pRefFGOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableB)
        {
            pRefBGOF->Update(YUVplanes, (uint8_t*)pRefB[0], nRefBPitches[0], (uint8_t*)pRefB[1], nRefBPitches[1], (uint8_t*)pRefB[2], nRefBPitches[2]);// v2.0
            if (YUVplanes & YPLANE)
                pPlanesB[0] = pRefBGOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesB[1] = pRefBGOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesB[2] = pRefBGOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableB2)
        {
            pRefB2GOF->Update(YUVplanes, (uint8_t*)pRefB2[0], nRefB2Pitches[0], (uint8_t*)pRefB2[1], nRefB2Pitches[1], (uint8_t*)pRefB2[2], nRefB2Pitches[2]);// v2.0
            if (YUVplanes & YPLANE)
                pPlanesB2[0] = pRefB2GOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesB2[1] = pRefB2GOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesB2[2] = pRefB2GOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableB3)
        {
            pRefB3GOF->Update(YUVplanes, (uint8_t*)pRefB3[0], nRefB3Pitches[0], (uint8_t*)pRefB3[1], nRefB3Pitches[1], (uint8_t*)pRefB3[2], nRefB3Pitches[2]);// v2.0
            if (YUVplanes & YPLANE)
                pPlanesB3[0] = pRefB3GOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesB3[1] = pRefB3GOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesB3[2] = pRefB3GOF->GetFrame(0)->GetPlane(VPLANE);
        }


        pDstCur[0] = pDst[0];
        pDstCur[1] = pDst[1];
        pDstCur[2] = pDst[2];
        pSrcCur[0] = pSrc[0];
        pSrcCur[1] = pSrc[1];
        pSrcCur[2] = pSrc[2];
        // -----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------

        // LUMA plane Y

        if (!(YUVplanes & YPLANE))
            BitBlt(pDstCur[0], nDstPitches[0], pSrcCur[0], nSrcPitches[0], nWidth, nHeight, isse);
        else
        {
            if (nOverlapX==0 && nOverlapY==0)
            {
                for (int by=0; by<nBlkY; by++)
                {
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        int i = by*nBlkX + bx;
                        const uint8_t * pB, *pF, *pB2, *pF2, *pB3, *pF3;
                        int npB, npF, npB2, npF2, npB3, npF3;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = blockB.GetX() * nPel + blockB.GetMV().x;
                            int blyB = blockB.GetY() * nPel + blockB.GetMV().y;
                            pB = pPlanesB[0]->GetPointer(blxB, blyB);
                            npB = pPlanesB[0]->GetPitch();
                            int blockSAD = blockB.GetSAD();
                            WRefB = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pB = pSrcCur[0]+ xx;
                            npB = nSrcPitches[0];
                            WRefB = 0;
                        }
                        if (isUsableF)
                        {
                            const FakeBlockData &blockF = ballsF.GetBlock(0, i);
                            int blxF = blockF.GetX() * nPel + blockF.GetMV().x;
                            int blyF = blockF.GetY() * nPel + blockF.GetMV().y;
                            pF = pPlanesF[0]->GetPointer(blxF, blyF);
                            npF = pPlanesF[0]->GetPitch();
                            int blockSAD = blockF.GetSAD();
                            WRefF = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pF = pSrcCur[0]+ xx;
                            npF = nSrcPitches[0];
                            WRefF = 0;
                        }
                        if (isUsableB2)
                        {
                            const FakeBlockData &blockB2 = ballsB2.GetBlock(0, i);
                            int blxB2 = blockB2.GetX() * nPel + blockB2.GetMV().x;
                            int blyB2 = blockB2.GetY() * nPel + blockB2.GetMV().y;
                            pB2 = pPlanesB2[0]->GetPointer(blxB2, blyB2);
                            npB2 = pPlanesB2[0]->GetPitch();
                            int blockSAD = blockB2.GetSAD();
                            WRefB2 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pB2 = pSrcCur[0]+ xx;
                            npB2 = nSrcPitches[0];
                            WRefB2 = 0;
                        }
                        if (isUsableF2)
                        {
                            const FakeBlockData &blockF2 = ballsF2.GetBlock(0, i);
                            int blxF2 = blockF2.GetX() * nPel + blockF2.GetMV().x;
                            int blyF2 = blockF2.GetY() * nPel + blockF2.GetMV().y;
                            pF2 = pPlanesF2[0]->GetPointer(blxF2, blyF2);
                            npF2 = pPlanesF2[0]->GetPitch();
                            int blockSAD = blockF2.GetSAD();
                            WRefF2 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pF2 = pSrcCur[0]+ xx;
                            npF2 = nSrcPitches[0];
                            WRefF2 = 0;
                        }
                        if (isUsableB3)
                        {
                            const FakeBlockData &blockB3 = ballsB3.GetBlock(0, i);
                            int blxB3 = blockB3.GetX() * nPel + blockB3.GetMV().x;
                            int blyB3 = blockB3.GetY() * nPel + blockB3.GetMV().y;
                            pB3 = pPlanesB3[0]->GetPointer(blxB3, blyB3);
                            npB3 = pPlanesB3[0]->GetPitch();
                            int blockSAD = blockB3.GetSAD();
                            WRefB3 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pB3 = pSrcCur[0]+ xx;
                            npB3 = nSrcPitches[0];
                            WRefB3 = 0;
                        }
                        if (isUsableF3)
                        {
                            const FakeBlockData &blockF3 = ballsF3.GetBlock(0, i);
                            int blxF3 = blockF3.GetX() * nPel + blockF3.GetMV().x;
                            int blyF3 = blockF3.GetY() * nPel + blockF3.GetMV().y;
                            pF3 = pPlanesF3[0]->GetPointer(blxF3, blyF3);
                            npF3 = pPlanesF3[0]->GetPitch();
                            int blockSAD = blockF3.GetSAD();
                            WRefF3 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pF3 = pSrcCur[0]+ xx;
                            npF3 = nSrcPitches[0];
                            WRefF3 = 0;
                        }
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + WRefB2 + WRefF2  + WRefB3 + WRefF3 + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WRefB2 = WRefB2*256/WSum;
                        WRefF2 = WRefF2*256/WSum;
                        WRefB3 = WRefB3*256/WSum;
                        WRefF3 = WRefF3*256/WSum;
                        WSrc = 256 - WRefB - WRefF - WRefB2 - WRefF2 - WRefB3 - WRefF3;
                        // luma
                        d->DEGRAINLUMA(pDstCur[0] + xx, nDstPitches[0], pSrcCur[0]+ xx, nSrcPitches[0],
                                pB, npB, pF, npF, pB2, npB2, pF2, npF2, pB3, npB3, pF3, npF3,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);

                        xx += (nBlkSizeX);

                        if (bx == nBlkX-1 && nWidth_B < nWidth) // right non-covered region
                        {
                            // luma
                            BitBlt(pDstCur[0] + nWidth_B, nDstPitches[0],
                                    pSrcCur[0] + nWidth_B, nSrcPitches[0], nWidth-nWidth_B, nBlkSizeY, isse);
                        }
                    }
                    pDstCur[0] += ( nBlkSizeY ) * (nDstPitches[0]);
                    pSrcCur[0] += ( nBlkSizeY ) * (nSrcPitches[0]);

                    if (by == nBlkY-1 && nHeight_B < nHeight) // bottom uncovered region
                    {
                        // luma
                        BitBlt(pDstCur[0], nDstPitches[0], pSrcCur[0], nSrcPitches[0], nWidth, nHeight-nHeight_B, isse);
                    }
                }
            }
            // -----------------------------------------------------------------
            else // overlap
            {
                pDstShort = DstShort;
                MemZoneSet(reinterpret_cast<unsigned char*>(pDstShort), 0, nWidth_B*2, nHeight_B, 0, 0, dstShortPitch*2);

                for (int by=0; by<nBlkY; by++)
                {
                    int wby = ((by + nBlkY - 3)/(nBlkY - 2))*3;
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        // select window
                        int wbx = (bx + nBlkX - 3)/(nBlkX - 2);
                        winOver = OverWins->GetWindow(wby + wbx);

                        int i = by*nBlkX + bx;
                        const uint8_t * pB, *pF, *pB2, *pF2, *pB3, *pF3;
                        int npB, npF, npB2, npF2, npB3, npF3;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = blockB.GetX() * nPel + blockB.GetMV().x;
                            int blyB = blockB.GetY() * nPel + blockB.GetMV().y;
                            pB = pPlanesB[0]->GetPointer(blxB, blyB);
                            npB = pPlanesB[0]->GetPitch();
                            int blockSAD = blockB.GetSAD();
                            WRefB = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pB = pSrcCur[0]+ xx;
                            npB = nSrcPitches[0];
                            WRefB = 0;
                        }
                        if (isUsableF)
                        {
                            const FakeBlockData &blockF = ballsF.GetBlock(0, i);
                            int blxF = blockF.GetX() * nPel + blockF.GetMV().x;
                            int blyF = blockF.GetY() * nPel + blockF.GetMV().y;
                            pF = pPlanesF[0]->GetPointer(blxF, blyF);
                            npF = pPlanesF[0]->GetPitch();
                            int blockSAD = blockF.GetSAD();
                            WRefF = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pF = pSrcCur[0]+ xx;
                            npF = nSrcPitches[0];
                            WRefF = 0;
                        }
                        if (isUsableB2)
                        {
                            const FakeBlockData &blockB2 = ballsB2.GetBlock(0, i);
                            int blxB2 = blockB2.GetX() * nPel + blockB2.GetMV().x;
                            int blyB2 = blockB2.GetY() * nPel + blockB2.GetMV().y;
                            pB2 = pPlanesB2[0]->GetPointer(blxB2, blyB2);
                            npB2 = pPlanesB2[0]->GetPitch();
                            int blockSAD = blockB2.GetSAD();
                            WRefB2 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pB2 = pSrcCur[0]+ xx;
                            npB2 = nSrcPitches[0];
                            WRefB2 = 0;
                        }
                        if (isUsableF2)
                        {
                            const FakeBlockData &blockF2 = ballsF2.GetBlock(0, i);
                            int blxF2 = blockF2.GetX() * nPel + blockF2.GetMV().x;
                            int blyF2 = blockF2.GetY() * nPel + blockF2.GetMV().y;
                            pF2 = pPlanesF2[0]->GetPointer(blxF2, blyF2);
                            npF2 = pPlanesF2[0]->GetPitch();
                            int blockSAD = blockF2.GetSAD();
                            WRefF2 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pF2 = pSrcCur[0]+ xx;
                            npF2 = nSrcPitches[0];
                            WRefF2 = 0;
                        }
                        if (isUsableB3)
                        {
                            const FakeBlockData &blockB3 = ballsB3.GetBlock(0, i);
                            int blxB3 = blockB3.GetX() * nPel + blockB3.GetMV().x;
                            int blyB3 = blockB3.GetY() * nPel + blockB3.GetMV().y;
                            pB3 = pPlanesB3[0]->GetPointer(blxB3, blyB3);
                            npB3 = pPlanesB3[0]->GetPitch();
                            int blockSAD = blockB3.GetSAD();
                            WRefB3 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pB3 = pSrcCur[0]+ xx;
                            npB3 = nSrcPitches[0];
                            WRefB3 = 0;
                        }
                        if (isUsableF3)
                        {
                            const FakeBlockData &blockF3 = ballsF3.GetBlock(0, i);
                            int blxF3 = blockF3.GetX() * nPel + blockF3.GetMV().x;
                            int blyF3 = blockF3.GetY() * nPel + blockF3.GetMV().y;
                            pF3 = pPlanesF3[0]->GetPointer(blxF3, blyF3);
                            npF3 = pPlanesF3[0]->GetPitch();
                            int blockSAD = blockF3.GetSAD();
                            WRefF3 = DegrainWeight(thSAD, blockSAD);
                        }
                        else
                        {
                            pF3 = pSrcCur[0]+ xx;
                            npF3 = nSrcPitches[0];
                            WRefF3 = 0;
                        }
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + WRefB2 + WRefF2 + WRefB3 + WRefF3 + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WRefB2 = WRefB2*256/WSum;
                        WRefF2 = WRefF2*256/WSum;
                        WRefB3 = WRefB3*256/WSum;
                        WRefF3 = WRefF3*256/WSum;
                        WSrc = 256 - WRefB - WRefF - WRefB2 - WRefF2 - WRefB3 - WRefF3;
                        // luma
                        d->DEGRAINLUMA(tmpBlock, tmpPitch, pSrcCur[0]+ xx, nSrcPitches[0],
                                pB, npB, pF, npF, pB2, npB2, pF2, npF2, pB3, npB3, pF3, npF3,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);
                        d->OVERSLUMA(pDstShort + xx, dstShortPitch, tmpBlock, tmpPitch, winOver, nBlkSizeX);

                        xx += (nBlkSizeX - nOverlapX);

                    }
                    pSrcCur[0] += (nBlkSizeY - nOverlapY) * (nSrcPitches[0]);
                    pDstShort += (nBlkSizeY - nOverlapY) * dstShortPitch;
                }
                Short2Bytes(pDst[0], nDstPitches[0], DstShort, dstShortPitch, nWidth_B, nHeight_B);
                if (nWidth_B < nWidth)
                    BitBlt(pDst[0] + nWidth_B, nDstPitches[0], pSrc[0] + nWidth_B, nSrcPitches[0], nWidth-nWidth_B, nHeight_B, isse);
                if (nHeight_B < nHeight) // bottom noncovered region
                    BitBlt(pDst[0] + nHeight_B*nDstPitches[0], nDstPitches[0], pSrc[0] + nHeight_B*nSrcPitches[0], nSrcPitches[0], nWidth, nHeight-nHeight_B, isse);
            }
            if (nLimit < 255) {
                if (isse)
                    LimitChanges_mmx(pDst[0], nDstPitches[0], pSrc[0], nSrcPitches[0], nWidth, nHeight, nLimit);
                else
                    LimitChanges_c(pDst[0], nDstPitches[0], pSrc[0], nSrcPitches[0], nWidth, nHeight, nLimit);
            }
        }
        //----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        // CHROMA plane U
        if (!(YUVplanes & UPLANE & nSuperModeYUV)) // v2.0
            BitBlt(pDstCur[1], nDstPitches[1], pSrcCur[1], nSrcPitches[1], nWidth>>1, nHeight/yRatioUV, isse);
        else
        {
            if (nOverlapX==0 && nOverlapY==0)
            {
                for (int by=0; by<nBlkY; by++)
                {
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        int i = by*nBlkX + bx;
                        const uint8_t * pBU, *pFU, *pB2U, *pF2U, *pB3U, *pF3U;
                        int npBU, npFU, npB2U, npF2U, npB3U, npF3U;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = blockB.GetX() * nPel + blockB.GetMV().x;
                            int blyB = blockB.GetY() * nPel + blockB.GetMV().y;
                            pBU = pPlanesB[1]->GetPointer(blxB>>1, blyB/yRatioUV);
                            npBU = pPlanesB[1]->GetPitch();
                            int blockSAD = blockB.GetSAD();
                            WRefB = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pBU = pSrcCur[1]+ (xx>>1);
                            npBU = nSrcPitches[1];
                            WRefB = 0;
                        }
                        if (isUsableF)
                        {
                            const FakeBlockData &blockF = ballsF.GetBlock(0, i);
                            int blxF = blockF.GetX() * nPel + blockF.GetMV().x;
                            int blyF = blockF.GetY() * nPel + blockF.GetMV().y;
                            pFU = pPlanesF[1]->GetPointer(blxF>>1, blyF/yRatioUV);
                            npFU = pPlanesF[1]->GetPitch();
                            int blockSAD = blockF.GetSAD();
                            WRefF = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pFU = pSrcCur[1]+ (xx>>1);
                            npFU = nSrcPitches[1];
                            WRefF = 0;
                        }
                        if (isUsableB2)
                        {
                            const FakeBlockData &blockB2 = ballsB2.GetBlock(0, i);
                            int blxB2 = blockB2.GetX() * nPel + blockB2.GetMV().x;
                            int blyB2 = blockB2.GetY() * nPel + blockB2.GetMV().y;
                            pB2U = pPlanesB2[1]->GetPointer(blxB2>>1, blyB2/yRatioUV);
                            npB2U = pPlanesB2[1]->GetPitch();
                            int blockSAD = blockB2.GetSAD();
                            WRefB2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB2U = pSrcCur[1]+ (xx>>1);
                            npB2U = nSrcPitches[1];
                            WRefB2 = 0;
                        }
                        if (isUsableF2)
                        {
                            const FakeBlockData &blockF2 = ballsF2.GetBlock(0, i);
                            int blxF2 = blockF2.GetX() * nPel + blockF2.GetMV().x;
                            int blyF2 = blockF2.GetY() * nPel + blockF2.GetMV().y;
                            pF2U = pPlanesF2[1]->GetPointer(blxF2>>1, blyF2/yRatioUV);
                            npF2U = pPlanesF2[1]->GetPitch();
                            int blockSAD = blockF2.GetSAD();
                            WRefF2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF2U = pSrcCur[1]+ (xx>>1);
                            npF2U = nSrcPitches[1];
                            WRefF2 = 0;
                        }
                        if (isUsableB3)
                        {
                            const FakeBlockData &blockB3 = ballsB3.GetBlock(0, i);
                            int blxB3 = blockB3.GetX() * nPel + blockB3.GetMV().x;
                            int blyB3 = blockB3.GetY() * nPel + blockB3.GetMV().y;
                            pB3U = pPlanesB3[1]->GetPointer(blxB3>>1, blyB3/yRatioUV);
                            npB3U = pPlanesB3[1]->GetPitch();
                            int blockSAD = blockB3.GetSAD();
                            WRefB3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB3U = pSrcCur[1]+ (xx>>1);
                            npB3U = nSrcPitches[1];
                            WRefB3 = 0;
                        }
                        if (isUsableF3)
                        {
                            const FakeBlockData &blockF3 = ballsF3.GetBlock(0, i);
                            int blxF3 = blockF3.GetX() * nPel + blockF3.GetMV().x;
                            int blyF3 = blockF3.GetY() * nPel + blockF3.GetMV().y;
                            pF3U = pPlanesF3[1]->GetPointer(blxF3>>1, blyF3/yRatioUV);
                            npF3U = pPlanesF3[1]->GetPitch();
                            int blockSAD = blockF3.GetSAD();
                            WRefF3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF3U = pSrcCur[1]+ (xx>>1);
                            npF3U = nSrcPitches[1];
                            WRefF3 = 0;
                        }
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + WRefB2 + WRefF2 + WRefB3 + WRefF3 + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WRefB2 = WRefB2*256/WSum;
                        WRefF2 = WRefF2*256/WSum;
                        WRefB3 = WRefB3*256/WSum;
                        WRefF3 = WRefF3*256/WSum;
                        WSrc = 256 - WRefB - WRefF - WRefB2 - WRefF2 - WRefB3 - WRefF3;
                        // chroma u
                        d->DEGRAINCHROMA(pDstCur[1] + (xx>>1), nDstPitches[1], pSrcCur[1]+ (xx>>1), nSrcPitches[1],
                                pBU, npBU, pFU, npFU, pB2U, npB2U, pF2U, npF2U, pB3U, npB3U, pF3U, npF3U,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);

                        xx += (nBlkSizeX);

                        if (bx == nBlkX-1 && nWidth_B < nWidth) // right non-covered region
                        {
                            // chroma u
                            BitBlt(pDstCur[1] + (nWidth_B>>1), nDstPitches[1],
                                    pSrcCur[1] + (nWidth_B>>1), nSrcPitches[1], (nWidth-nWidth_B)>>1, (nBlkSizeY)/yRatioUV, isse);
                        }
                    }
                    pDstCur[1] += ( nBlkSizeY )/yRatioUV * (nDstPitches[1]) ;
                    pSrcCur[1] += ( nBlkSizeY )/yRatioUV * (nSrcPitches[1]) ;

                    if (by == nBlkY-1 && nHeight_B < nHeight) // bottom uncovered region
                    {
                        // chroma u
                        BitBlt(pDstCur[1], nDstPitches[1], pSrcCur[1], nSrcPitches[1], nWidth>>1, (nHeight-nHeight_B)/yRatioUV, isse);
                    }
                }
            }
            // -----------------------------------------------------------------
            else // overlap
            {
                pDstShort = DstShort;
                MemZoneSet(reinterpret_cast<unsigned char*>(pDstShort), 0, nWidth_B, nHeight_B/yRatioUV, 0, 0, dstShortPitch*2);
                for (int by=0; by<nBlkY; by++)
                {
                    int wby = ((by + nBlkY - 3)/(nBlkY - 2))*3;
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        // select window
                        int wbx = (bx + nBlkX - 3)/(nBlkX - 2);
                        winOverUV = OverWinsUV->GetWindow(wby + wbx);

                        int i = by*nBlkX + bx;
                        const uint8_t * pBU, *pFU, *pB2U, *pF2U, *pB3U, *pF3U;
                        int npBU, npFU, npB2U, npF2U, npB3U, npF3U;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = blockB.GetX() * nPel + blockB.GetMV().x;
                            int blyB = blockB.GetY() * nPel + blockB.GetMV().y;
                            pBU = pPlanesB[1]->GetPointer(blxB>>1, blyB/yRatioUV);
                            npBU = pPlanesB[1]->GetPitch();
                            int blockSAD = blockB.GetSAD();
                            WRefB = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pBU = pSrcCur[1]+ (xx>>1);
                            npBU = nSrcPitches[1];
                            WRefB = 0;
                        }
                        if (isUsableF)
                        {
                            const FakeBlockData &blockF = ballsF.GetBlock(0, i);
                            int blxF = blockF.GetX() * nPel + blockF.GetMV().x;
                            int blyF = blockF.GetY() * nPel + blockF.GetMV().y;
                            pFU = pPlanesF[1]->GetPointer(blxF>>1, blyF/yRatioUV);
                            npFU = pPlanesF[1]->GetPitch();
                            int blockSAD = blockF.GetSAD();
                            WRefF = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pFU = pSrcCur[1]+ (xx>>1);
                            npFU = nSrcPitches[1];
                            WRefF = 0;
                        }
                        if (isUsableB2)
                        {
                            const FakeBlockData &blockB2 = ballsB2.GetBlock(0, i);
                            int blxB2 = blockB2.GetX() * nPel + blockB2.GetMV().x;
                            int blyB2 = blockB2.GetY() * nPel + blockB2.GetMV().y;
                            pB2U = pPlanesB2[1]->GetPointer(blxB2>>1, blyB2/yRatioUV);
                            npB2U = pPlanesB2[1]->GetPitch();
                            int blockSAD = blockB2.GetSAD();
                            WRefB2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB2U = pSrcCur[1]+ (xx>>1);
                            npB2U = nSrcPitches[1];
                            WRefB2 = 0;
                        }
                        if (isUsableF2)
                        {
                            const FakeBlockData &blockF2 = ballsF2.GetBlock(0, i);
                            int blxF2 = blockF2.GetX() * nPel + blockF2.GetMV().x;
                            int blyF2 = blockF2.GetY() * nPel + blockF2.GetMV().y;
                            pF2U = pPlanesF2[1]->GetPointer(blxF2>>1, blyF2/yRatioUV);
                            npF2U = pPlanesF2[1]->GetPitch();
                            int blockSAD = blockF2.GetSAD();
                            WRefF2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF2U = pSrcCur[1]+ (xx>>1);
                            npF2U = nSrcPitches[1];
                            WRefF2 = 0;
                        }
                        if (isUsableB3)
                        {
                            const FakeBlockData &blockB3 = ballsB3.GetBlock(0, i);
                            int blxB3 = blockB3.GetX() * nPel + blockB3.GetMV().x;
                            int blyB3 = blockB3.GetY() * nPel + blockB3.GetMV().y;
                            pB3U = pPlanesB3[1]->GetPointer(blxB3>>1, blyB3/yRatioUV);
                            npB3U = pPlanesB3[1]->GetPitch();
                            int blockSAD = blockB3.GetSAD();
                            WRefB3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB3U = pSrcCur[1]+ (xx>>1);
                            npB3U = nSrcPitches[1];
                            WRefB3 = 0;
                        }
                        if (isUsableF3)
                        {
                            const FakeBlockData &blockF3 = ballsF3.GetBlock(0, i);
                            int blxF3 = blockF3.GetX() * nPel + blockF3.GetMV().x;
                            int blyF3 = blockF3.GetY() * nPel + blockF3.GetMV().y;
                            pF3U = pPlanesF3[1]->GetPointer(blxF3>>1, blyF3/yRatioUV);
                            npF3U = pPlanesF3[1]->GetPitch();
                            int blockSAD = blockF3.GetSAD();
                            WRefF3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF3U = pSrcCur[1]+ (xx>>1);
                            npF3U = nSrcPitches[1];
                            WRefF3 = 0;
                        }
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + WRefB2 + WRefF2 + WRefB3 + WRefF3 + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WRefB2 = WRefB2*256/WSum;
                        WRefF2 = WRefF2*256/WSum;
                        WRefB3 = WRefB3*256/WSum;
                        WRefF3 = WRefF3*256/WSum;
                        WSrc = 256 - WRefB - WRefF - WRefB2 - WRefF2 - WRefB3 - WRefF3;
                        // chroma u
                        d->DEGRAINCHROMA(tmpBlock, tmpPitch, pSrcCur[1]+ (xx>>1), nSrcPitches[1],
                                pBU, npBU, pFU, npFU, pB2U, npB2U, pF2U, npF2U, pB3U, npB3U, pF3U, npF3U,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);
                        d->OVERSCHROMA(pDstShort + (xx>>1), dstShortPitch, tmpBlock, tmpPitch, winOverUV, nBlkSizeX/2);

                        xx += (nBlkSizeX - nOverlapX);

                    }
                    pSrcCur[1] += (nBlkSizeY - nOverlapY)/yRatioUV * (nSrcPitches[1]) ;
                    pDstShort += (nBlkSizeY - nOverlapY)/yRatioUV * dstShortPitch;
                }
                Short2Bytes(pDst[1], nDstPitches[1], DstShort, dstShortPitch, nWidth_B>>1, nHeight_B/yRatioUV);
                if (nWidth_B < nWidth)
                    BitBlt(pDst[1] + (nWidth_B>>1), nDstPitches[1], pSrc[1] + (nWidth_B>>1), nSrcPitches[1], (nWidth-nWidth_B)>>1, nHeight_B/yRatioUV, isse);
                if (nHeight_B < nHeight) // bottom noncovered region
                    BitBlt(pDst[1] + nDstPitches[1]*nHeight_B/yRatioUV, nDstPitches[1], pSrc[1] + nSrcPitches[1]*nHeight_B/yRatioUV, nSrcPitches[1], nWidth>>1, (nHeight-nHeight_B)/yRatioUV, isse);
            }
            if (nLimitC < 255) {
                if (isse)
                    LimitChanges_mmx(pDst[1], nDstPitches[1], pSrc[1], nSrcPitches[1], nWidth/2, nHeight/yRatioUV, nLimitC);
                else
                    LimitChanges_c(pDst[1], nDstPitches[1], pSrc[1], nSrcPitches[1], nWidth/2, nHeight/yRatioUV, nLimitC);
            }
        }
        //----------------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        // CHROMA plane V
        if (!(YUVplanes & VPLANE & nSuperModeYUV))
            BitBlt(pDstCur[2], nDstPitches[2], pSrcCur[2], nSrcPitches[2], nWidth>>1, nHeight/yRatioUV, isse);
        else
        {
            if (nOverlapX==0 && nOverlapY==0)
            {
                for (int by=0; by<nBlkY; by++)
                {
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        int i = by*nBlkX + bx;
                        const uint8_t * pBV, *pFV, *pB2V, *pF2V, *pB3V, *pF3V;
                        int npBV, npFV, npB2V, npF2V, npB3V, npF3V;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = blockB.GetX() * nPel + blockB.GetMV().x;
                            int blyB = blockB.GetY() * nPel + blockB.GetMV().y;
                            pBV = pPlanesB[2]->GetPointer(blxB>>1, blyB/yRatioUV);
                            npBV = pPlanesB[2]->GetPitch();
                            int blockSAD = blockB.GetSAD();
                            WRefB = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pBV = pSrcCur[2]+ (xx>>1);
                            npBV = nSrcPitches[2];
                            WRefB = 0;
                        }
                        if (isUsableF)
                        {
                            const FakeBlockData &blockF = ballsF.GetBlock(0, i);
                            int blxF = blockF.GetX() * nPel + blockF.GetMV().x;
                            int blyF = blockF.GetY() * nPel + blockF.GetMV().y;
                            pFV = pPlanesF[2]->GetPointer(blxF>>1, blyF/yRatioUV);
                            npFV = pPlanesF[2]->GetPitch();
                            int blockSAD = blockF.GetSAD();
                            WRefF = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pFV = pSrcCur[2]+ (xx>>1);
                            npFV = nSrcPitches[2];
                            WRefF = 0;
                        }
                        if (isUsableB2)
                        {
                            const FakeBlockData &blockB2 = ballsB2.GetBlock(0, i);
                            int blxB2 = blockB2.GetX() * nPel + blockB2.GetMV().x;
                            int blyB2 = blockB2.GetY() * nPel + blockB2.GetMV().y;
                            pB2V = pPlanesB2[2]->GetPointer(blxB2>>1, blyB2/yRatioUV);
                            npB2V = pPlanesB2[2]->GetPitch();
                            int blockSAD = blockB2.GetSAD();
                            WRefB2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB2V = pSrcCur[2]+ (xx>>1);
                            npB2V = nSrcPitches[2];
                            WRefB2 = 0;
                        }
                        if (isUsableF2)
                        {
                            const FakeBlockData &blockF2 = ballsF2.GetBlock(0, i);
                            int blxF2 = blockF2.GetX() * nPel + blockF2.GetMV().x;
                            int blyF2 = blockF2.GetY() * nPel + blockF2.GetMV().y;
                            pF2V = pPlanesF2[2]->GetPointer(blxF2>>1, blyF2/yRatioUV);
                            npF2V = pPlanesF2[2]->GetPitch();
                            int blockSAD = blockF2.GetSAD();
                            WRefF2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF2V = pSrcCur[2]+ (xx>>1);
                            npF2V = nSrcPitches[2];
                            WRefF2 = 0;
                        }
                        if (isUsableB3)
                        {
                            const FakeBlockData &blockB3 = ballsB3.GetBlock(0, i);
                            int blxB3 = blockB3.GetX() * nPel + blockB3.GetMV().x;
                            int blyB3 = blockB3.GetY() * nPel + blockB3.GetMV().y;
                            pB3V = pPlanesB3[2]->GetPointer(blxB3>>1, blyB3/yRatioUV);
                            npB3V = pPlanesB3[2]->GetPitch();
                            int blockSAD = blockB3.GetSAD();
                            WRefB3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB3V = pSrcCur[2]+ (xx>>1);
                            npB3V = nSrcPitches[2];
                            WRefB3 = 0;
                        }
                        if (isUsableF3)
                        {
                            const FakeBlockData &blockF3 = ballsF3.GetBlock(0, i);
                            int blxF3 = blockF3.GetX() * nPel + blockF3.GetMV().x;
                            int blyF3 = blockF3.GetY() * nPel + blockF3.GetMV().y;
                            pF3V = pPlanesF3[2]->GetPointer(blxF3>>1, blyF3/yRatioUV);
                            npF3V = pPlanesF3[2]->GetPitch();
                            int blockSAD = blockF3.GetSAD();
                            WRefF3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF3V = pSrcCur[2]+ (xx>>1);
                            npF3V = nSrcPitches[2];
                            WRefF3 = 0;
                        }
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + WRefB2 + WRefF2 + WRefB3 + WRefF3 + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WRefB2 = WRefB2*256/WSum;
                        WRefF2 = WRefF2*256/WSum;
                        WRefB3 = WRefB3*256/WSum;
                        WRefF3 = WRefF3*256/WSum;
                        WSrc = 256 - WRefB - WRefF - WRefB2 - WRefF2 - WRefB3 - WRefF3;
                        // chroma v
                        d->DEGRAINCHROMA(pDstCur[2] + (xx>>1), nDstPitches[2], pSrcCur[2]+ (xx>>1), nSrcPitches[2],
                                pBV, npBV, pFV, npFV, pB2V, npB2V, pF2V, npF2V, pB3V, npB3V, pF3V, npF3V,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);

                        xx += (nBlkSizeX);

                        if (bx == nBlkX-1 && nWidth_B < nWidth) // right non-covered region
                        {
                            // chroma v
                            BitBlt(pDstCur[2] + (nWidth_B>>1), nDstPitches[2],
                                    pSrcCur[2] + (nWidth_B>>1), nSrcPitches[2], (nWidth-nWidth_B)>>1, (nBlkSizeY)/yRatioUV, isse);
                        }
                    }
                    pDstCur[2] += ( nBlkSizeY )/yRatioUV * (nDstPitches[2]) ;
                    pSrcCur[2] += ( nBlkSizeY )/yRatioUV * (nSrcPitches[2]) ;

                    if (by == nBlkY-1 && nHeight_B < nHeight) // bottom uncovered region
                    {
                        // chroma v
                        BitBlt(pDstCur[2], nDstPitches[2], pSrcCur[2], nSrcPitches[2], nWidth>>1, (nHeight-nHeight_B)/yRatioUV, isse);
                    }
                }
            }
            // -----------------------------------------------------------------
            else // overlap
            {
                pDstShort = DstShort;
                MemZoneSet(reinterpret_cast<unsigned char*>(pDstShort), 0, nWidth_B, nHeight_B/yRatioUV, 0, 0, dstShortPitch*2);

                for (int by=0; by<nBlkY; by++)
                {
                    int wby = ((by + nBlkY - 3)/(nBlkY - 2))*3;
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        // select window
                        int wbx = (bx + nBlkX - 3)/(nBlkX - 2);
                        winOverUV = OverWinsUV->GetWindow(wby + wbx);

                        int i = by*nBlkX + bx;
                        const uint8_t * pBV, *pFV, *pB2V, *pF2V, *pB3V, *pF3V;
                        int npBV, npFV, npB2V, npF2V, npB3V, npF3V;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = blockB.GetX() * nPel + blockB.GetMV().x;
                            int blyB = blockB.GetY() * nPel + blockB.GetMV().y;
                            pBV = pPlanesB[2]->GetPointer(blxB>>1, blyB/yRatioUV);
                            npBV = pPlanesB[2]->GetPitch();
                            int blockSAD = blockB.GetSAD();
                            WRefB = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pBV = pSrcCur[2]+ (xx>>1);
                            npBV = nSrcPitches[2];
                            WRefB = 0;
                        }
                        if (isUsableF)
                        {
                            const FakeBlockData &blockF = ballsF.GetBlock(0, i);
                            int blxF = blockF.GetX() * nPel + blockF.GetMV().x;
                            int blyF = blockF.GetY() * nPel + blockF.GetMV().y;
                            pFV = pPlanesF[2]->GetPointer(blxF>>1, blyF/yRatioUV);
                            npFV = pPlanesF[2]->GetPitch();
                            int blockSAD = blockF.GetSAD();
                            WRefF = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pFV = pSrcCur[2]+ (xx>>1);
                            npFV = nSrcPitches[2];
                            WRefF = 0;
                        }
                        if (isUsableB2)
                        {
                            const FakeBlockData &blockB2 = ballsB2.GetBlock(0, i);
                            int blxB2 = blockB2.GetX() * nPel + blockB2.GetMV().x;
                            int blyB2 = blockB2.GetY() * nPel + blockB2.GetMV().y;
                            pB2V = pPlanesB2[2]->GetPointer(blxB2>>1, blyB2/yRatioUV);
                            npB2V = pPlanesB2[2]->GetPitch();
                            int blockSAD = blockB2.GetSAD();
                            WRefB2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB2V = pSrcCur[2]+ (xx>>1);
                            npB2V = nSrcPitches[2];
                            WRefB2 = 0;
                        }
                        if (isUsableF2)
                        {
                            const FakeBlockData &blockF2 = ballsF2.GetBlock(0, i);
                            int blxF2 = blockF2.GetX() * nPel + blockF2.GetMV().x;
                            int blyF2 = blockF2.GetY() * nPel + blockF2.GetMV().y;
                            pF2V = pPlanesF2[2]->GetPointer(blxF2>>1, blyF2/yRatioUV);
                            npF2V = pPlanesF2[2]->GetPitch();
                            int blockSAD = blockF2.GetSAD();
                            WRefF2 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF2V = pSrcCur[2]+ (xx>>1);
                            npF2V = nSrcPitches[2];
                            WRefF2 = 0;
                        }
                        if (isUsableB3)
                        {
                            const FakeBlockData &blockB3 = ballsB3.GetBlock(0, i);
                            int blxB3 = blockB3.GetX() * nPel + blockB3.GetMV().x;
                            int blyB3 = blockB3.GetY() * nPel + blockB3.GetMV().y;
                            pB3V = pPlanesB3[2]->GetPointer(blxB3>>1, blyB3/yRatioUV);
                            npB3V = pPlanesB3[2]->GetPitch();
                            int blockSAD = blockB3.GetSAD();
                            WRefB3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pB3V = pSrcCur[2]+ (xx>>1);
                            npB3V = nSrcPitches[2];
                            WRefB3 = 0;
                        }
                        if (isUsableF3)
                        {
                            const FakeBlockData &blockF3 = ballsF3.GetBlock(0, i);
                            int blxF3 = blockF3.GetX() * nPel + blockF3.GetMV().x;
                            int blyF3 = blockF3.GetY() * nPel + blockF3.GetMV().y;
                            pF3V = pPlanesF3[2]->GetPointer(blxF3>>1, blyF3/yRatioUV);
                            npF3V = pPlanesF3[2]->GetPitch();
                            int blockSAD = blockF3.GetSAD();
                            WRefF3 = DegrainWeight(thSADC, blockSAD);
                        }
                        else
                        {
                            pF3V = pSrcCur[2]+ (xx>>1);
                            npF3V = nSrcPitches[2];
                            WRefF3 = 0;
                        }
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + WRefB2 + WRefF2 + WRefB3 + WRefF3 + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WRefB2 = WRefB2*256/WSum;
                        WRefF2 = WRefF2*256/WSum;
                        WRefB3 = WRefB3*256/WSum;
                        WRefF3 = WRefF3*256/WSum;
                        WSrc = 256 - WRefB - WRefF - WRefB2 - WRefF2 - WRefB3 - WRefF3;
                        // chroma v
                        d->DEGRAINCHROMA(tmpBlock, tmpPitch, pSrcCur[2]+ (xx>>1), nSrcPitches[2],
                                pBV, npBV, pFV, npFV, pB2V, npB2V, pF2V, npF2V, pB3V, npB3V, pF3V, npF3V,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);
                        d->OVERSCHROMA(pDstShort + (xx>>1), dstShortPitch, tmpBlock, tmpPitch, winOverUV, nBlkSizeX/2);

                        xx += (nBlkSizeX - nOverlapX);

                    }
                    pSrcCur[2] += (nBlkSizeY - nOverlapY)/yRatioUV * (nSrcPitches[2]) ;
                    pDstShort += (nBlkSizeY - nOverlapY)/yRatioUV * dstShortPitch;
                }
                Short2Bytes(pDst[2], nDstPitches[2], DstShort, dstShortPitch, nWidth_B>>1, nHeight_B/yRatioUV);
                if (nWidth_B < nWidth)
                    BitBlt(pDst[2] + (nWidth_B>>1), nDstPitches[2], pSrc[2] + (nWidth_B>>1), nSrcPitches[2], (nWidth-nWidth_B)>>1, nHeight_B/yRatioUV, isse);
                if (nHeight_B < nHeight) // bottom noncovered region
                    BitBlt(pDst[2] + nDstPitches[2]*nHeight_B/yRatioUV, nDstPitches[2], pSrc[2] + nSrcPitches[2]*nHeight_B/yRatioUV, nSrcPitches[2], nWidth>>1, (nHeight-nHeight_B)/yRatioUV, isse);
            }
            if (nLimitC < 255) {
                if (isse)
                    LimitChanges_mmx(pDst[2], nDstPitches[2], pSrc[2], nSrcPitches[2], nWidth/2, nHeight/yRatioUV, nLimitC);
                else
                    LimitChanges_c(pDst[2], nDstPitches[2], pSrc[2], nSrcPitches[2], nWidth/2, nHeight/yRatioUV, nLimitC);
            }
        }
        //--------------------------------------------------------------------------------


        delete[] tmpBlock;

        if (OverWins)
            delete OverWins;
        if (OverWinsUV)
            delete OverWinsUV;
        if (DstShort)
            delete[] DstShort;

        delete pRefFGOF;
        delete pRefBGOF;
        delete pRefF2GOF;
        delete pRefB2GOF;
        delete pRefF3GOF;
        delete pRefB3GOF;

        if (refF)
            vsapi->freeFrame(refF);
        if (refB)
            vsapi->freeFrame(refB);
        if (refF2)
            vsapi->freeFrame(refF2);
        if (refB2)
            vsapi->freeFrame(refB2);
        if (refF3)
            vsapi->freeFrame(refF3);
        if (refB3)
            vsapi->freeFrame(refB3);

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvdegrain3Free(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVDegrain3Data *d = (MVDegrain3Data *)instanceData;

    delete d->mvClipB;
    delete d->mvClipF;
    delete d->mvClipB2;
    delete d->mvClipF2;
    delete d->mvClipB3;
    delete d->mvClipF3;

    delete d->bleh;

    vsapi->freeNode(d->super);
    vsapi->freeNode(d->mvfw);
    vsapi->freeNode(d->mvbw);
    vsapi->freeNode(d->mvfw2);
    vsapi->freeNode(d->mvbw2);
    vsapi->freeNode(d->mvfw3);
    vsapi->freeNode(d->mvbw3);
    vsapi->freeNode(d->node);
    free(d);
}


static void selectFunctions(MVDegrain3Data *d) {
    const int yRatioUV = d->bleh->yRatioUV;
    const int nBlkSizeX = d->bleh->nBlkSizeX;
    const int nBlkSizeY = d->bleh->nBlkSizeY;

    if (d->isse)
    {
        switch (nBlkSizeX)
        {
            case 32:
                if (nBlkSizeY==16) {          d->OVERSLUMA = Overlaps32x16_mmx;  d->DEGRAINLUMA = Degrain3_sse2<32,16>;
                    if (yRatioUV==2) {         d->OVERSCHROMA = Overlaps16x8_mmx; d->DEGRAINCHROMA = Degrain3_sse2<16,8>;         }
                    else {                     d->OVERSCHROMA = Overlaps16x16_mmx;d->DEGRAINCHROMA = Degrain3_sse2<16,16>;         }
                } else if (nBlkSizeY==32) {    d->OVERSLUMA = Overlaps32x32_mmx;  d->DEGRAINLUMA = Degrain3_sse2<32,32>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps16x16_mmx; d->DEGRAINCHROMA = Degrain3_sse2<16,16>;         }
                    else {                        d->OVERSCHROMA = Overlaps16x32_mmx; d->DEGRAINCHROMA = Degrain3_sse2<16,32>;         }
                } break;
            case 16:
                if (nBlkSizeY==16) {          d->OVERSLUMA = Overlaps16x16_mmx; d->DEGRAINLUMA = Degrain3_sse2<16,16>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps8x8_mmx; d->DEGRAINCHROMA = Degrain3_sse2<8,8>;         }
                    else {                        d->OVERSCHROMA = Overlaps8x16_mmx;d->DEGRAINCHROMA = Degrain3_sse2<8,16>;         }
                } else if (nBlkSizeY==8) {    d->OVERSLUMA = Overlaps16x8_mmx;  d->DEGRAINLUMA = Degrain3_sse2<16,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps8x4_mmx; d->DEGRAINCHROMA = Degrain3_sse2<8,4>;         }
                    else {                        d->OVERSCHROMA = Overlaps8x8_mmx; d->DEGRAINCHROMA = Degrain3_sse2<8,8>;         }
                } else if (nBlkSizeY==2) {    d->OVERSLUMA = Overlaps16x2_mmx;  d->DEGRAINLUMA = Degrain3_sse2<16,2>;
                    if (yRatioUV==2) {         d->OVERSCHROMA = Overlaps8x1_mmx; d->DEGRAINCHROMA = Degrain3_sse2<8,1>;         }
                    else {                        d->OVERSCHROMA = Overlaps8x2_mmx; d->DEGRAINCHROMA = Degrain3_sse2<8,2>;         }
                }
                break;
            case 4:
                d->OVERSLUMA = Overlaps4x4_mmx;    d->DEGRAINLUMA = Degrain3_mmx<4,4>;
                if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<2,2>;    d->DEGRAINCHROMA = Degrain3_C<2,2>;         }
                else {                        d->OVERSCHROMA = Overlaps_C<2,4>;    d->DEGRAINCHROMA = Degrain3_C<2,4>;         }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {           d->OVERSLUMA = Overlaps8x8_mmx;    d->DEGRAINLUMA = Degrain3_sse2<8,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps4x4_mmx;  d->DEGRAINCHROMA = Degrain3_mmx<4,4>;         }
                    else {                        d->OVERSCHROMA = Overlaps4x8_mmx;  d->DEGRAINCHROMA = Degrain3_mmx<4,8>;         }
                }else if (nBlkSizeY==4) {     d->OVERSLUMA = Overlaps8x4_mmx;    d->DEGRAINLUMA = Degrain3_sse2<8,4>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps4x2_mmx;    d->DEGRAINCHROMA = Degrain3_mmx<4,2>;         }
                    else {                        d->OVERSCHROMA = Overlaps4x4_mmx;  d->DEGRAINCHROMA = Degrain3_mmx<4,4>;         }
                }
        }
    }
    else
    {
        switch (nBlkSizeX)
        {
            case 32:
                if (nBlkSizeY==16) {          d->OVERSLUMA = Overlaps_C<32,16>;  d->DEGRAINLUMA = Degrain3_C<32,16>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<16,8>; d->DEGRAINCHROMA = Degrain3_C<16,8>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<16,16>;d->DEGRAINCHROMA = Degrain3_C<16,16>;         }
                } else if (nBlkSizeY==32) {    d->OVERSLUMA = Overlaps_C<32,32>;   d->DEGRAINLUMA = Degrain3_C<32,32>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<16,16>;  d->DEGRAINCHROMA = Degrain3_C<16,16>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<16,32>;  d->DEGRAINCHROMA = Degrain3_C<16,32>;         }
                } break;
            case 16:
                if (nBlkSizeY==16) {          d->OVERSLUMA = Overlaps_C<16,16>;  d->DEGRAINLUMA = Degrain3_C<16,16>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<8,8>;  d->DEGRAINCHROMA = Degrain3_C<8,8>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<8,16>; d->DEGRAINCHROMA = Degrain3_C<8,16>;         }
                } else if (nBlkSizeY==8) {    d->OVERSLUMA = Overlaps_C<16,8>;   d->DEGRAINLUMA = Degrain3_C<16,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<8,4>;  d->DEGRAINCHROMA = Degrain3_C<8,4>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<8,8>;  d->DEGRAINCHROMA = Degrain3_C<8,8>;         }
                } else if (nBlkSizeY==2) {    d->OVERSLUMA = Overlaps_C<16,2>;   d->DEGRAINLUMA = Degrain3_C<16,2>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<8,1>;  d->DEGRAINCHROMA = Degrain3_C<8,1>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<8,2>;  d->DEGRAINCHROMA = Degrain3_C<8,2>;         }
                }
                break;
            case 4:
                d->OVERSLUMA = Overlaps_C<4,4>;    d->DEGRAINLUMA = Degrain3_C<4,4>;
                if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<2,2>;  d->DEGRAINCHROMA = Degrain3_C<2,2>;         }
                else {                        d->OVERSCHROMA = Overlaps_C<2,4>;  d->DEGRAINCHROMA = Degrain3_C<2,4>;         }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {           d->OVERSLUMA = Overlaps_C<8,8>;    d->DEGRAINLUMA = Degrain3_C<8,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<4,4>;  d->DEGRAINCHROMA = Degrain3_C<4,4>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<4,8>;  d->DEGRAINCHROMA = Degrain3_C<4,8>;         }
                }else if (nBlkSizeY==4) {     d->OVERSLUMA = Overlaps_C<8,4>;    d->DEGRAINLUMA = Degrain3_C<8,4>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<4,2>;  d->DEGRAINCHROMA = Degrain3_C<4,2>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<4,4>;  d->DEGRAINCHROMA = Degrain3_C<4,4>;         }
                }
        }
    }
}


static void VS_CC mvdegrain3Create(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVDegrain3Data d;
    MVDegrain3Data *data;

    int err;

    d.thSAD = vsapi->propGetInt(in, "thsad", 0, &err);
    if (err)
        d.thSAD = 400;

    d.thSADC = vsapi->propGetInt(in, "thsadc", 0, &err);
    if (err)
        d.thSADC = d.thSAD;

    int plane = vsapi->propGetInt(in, "plane", 0, &err);
    if (err)
        plane = 4;

    d.nLimit = vsapi->propGetInt(in, "limit", 0, &err);
    if (err)
        d.nLimit = 255;

    d.nLimitC = vsapi->propGetInt(in, "limitc", 0, &err);
    if (err)
        d.nLimitC = d.nLimit;

    d.nSCD1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.nSCD1 = MV_DEFAULT_SCD1;

    d.nSCD2 = vsapi->propGetInt(in, "thscd2", 0, &err);
    if (err)
        d.nSCD2 = MV_DEFAULT_SCD2;

    d.isse = vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 0; // FIXME: used to be 1


    int planes[5] = { YPLANE, UPLANE, VPLANE, UVPLANES, YUVPLANES };
    d.YUVplanes = planes[plane];


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("Degrain3: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
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
            vsapi->setError(out, "Degrain3: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);
    d.mvbw2 = vsapi->propGetNode(in, "mvbw2", 0, NULL);
    d.mvfw2 = vsapi->propGetNode(in, "mvfw2", 0, NULL);
    d.mvbw3 = vsapi->propGetNode(in, "mvbw3", 0, NULL);
    d.mvfw3 = vsapi->propGetNode(in, "mvfw3", 0, NULL);

    try {
        d.mvClipB = new MVClipDicks(d.mvbw, d.nSCD1, d.nSCD2, vsapi);
        d.mvClipF = new MVClipDicks(d.mvfw, d.nSCD1, d.nSCD2, vsapi);
        d.mvClipB2 = new MVClipDicks(d.mvbw2, d.nSCD1, d.nSCD2, vsapi);
        d.mvClipF2 = new MVClipDicks(d.mvfw2, d.nSCD1, d.nSCD2, vsapi);
        d.mvClipB3 = new MVClipDicks(d.mvbw3, d.nSCD1, d.nSCD2, vsapi);
        d.mvClipF3 = new MVClipDicks(d.mvfw3, d.nSCD1, d.nSCD2, vsapi);

        // Make sure the motion vector clips are correct.
        if (!d.mvClipB3->IsBackward())
            throw MVException("Degrain3: mvbw3 must be generated with isb=True.");
        if (!d.mvClipB2->IsBackward())
            throw MVException("Degrain3: mvbw2 must be generated with isb=True.");
        if (!d.mvClipB->IsBackward())
            throw MVException("Degrain3: mvbw must be generated with isb=True.");
        if (d.mvClipF->IsBackward())
            throw MVException("Degrain3: mvfw must be generated with isb=False.");
        if (d.mvClipF2->IsBackward())
            throw MVException("Degrain3: mvfw2 must be generated with isb=False.");
        if (d.mvClipF3->IsBackward())
            throw MVException("Degrain3: mvfw3 must be generated with isb=False.");

        if (d.mvClipB3->GetDeltaFrame() <= d.mvClipB2->GetDeltaFrame())
            throw MVException("Degrain3: mvbw3 must have greater delta than mvbw2.");
        if (d.mvClipB2->GetDeltaFrame() <= d.mvClipB->GetDeltaFrame())
            throw MVException("Degrain3: mvbw2 must have greater delta than mvbw.");
        if (d.mvClipF3->GetDeltaFrame() <= d.mvClipF2->GetDeltaFrame())
            throw MVException("Degrain3: mvfw3 must have greater delta than mvfw2.");
        if (d.mvClipF2->GetDeltaFrame() <= d.mvClipF->GetDeltaFrame())
            throw MVException("Degrain3: mvfw2 must have greater delta than mvfw.");

        d.bleh = new MVFilter(d.mvfw3, "Degrain3", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        return;
    }


    try {
        d.bleh->CheckSimilarity(d.mvClipF, "mvfw");
        d.bleh->CheckSimilarity(d.mvClipB, "mvbw");
        d.bleh->CheckSimilarity(d.mvClipF2, "mvfw2");
        d.bleh->CheckSimilarity(d.mvClipB2, "mvbw2");
        d.bleh->CheckSimilarity(d.mvClipF2, "mvfw3");
        d.bleh->CheckSimilarity(d.mvClipB2, "mvbw3");
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        delete d.bleh;
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        return;
    }

    d.thSAD = d.thSAD * d.mvClipB->GetThSCD1() / d.nSCD1; // normalize to block SAD
    d.thSADC = d.thSADC * d.mvClipB->GetThSCD1() / d.nSCD1; // chroma threshold, normalized to block SAD



    d.dstShortPitch = ((d.bleh->nWidth + 15)/16)*16;


    selectFunctions(&d);


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.bleh->nHeight != nHeightS || d.bleh->nHeight != d.vi->height || d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2 || d.bleh->nWidth != d.vi->width) {
        vsapi->setError(out, "Degrain3: wrong source or super clip frame size.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        vsapi->freeNode(d.node);
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->id != pfYUV420P8) {
        vsapi->setError(out, "Degrain3: input clip must be YUV420P8 with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        vsapi->freeNode(d.node);
        return;
    }


    data = (MVDegrain3Data *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Degrain3", mvdegrain3Init, mvdegrain3GetFrame, mvdegrain3Free, fmParallel, 0, data, core);
}


void mvdegrain3Register(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Degrain3",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "mvbw2:clip;"
                 "mvfw2:clip;"
                 "mvbw3:clip;"
                 "mvfw3:clip;"
                 "thsad:int:opt;"
                 "thsadc:int:opt;"
                 "plane:int:opt;"
                 "limit:int:opt;"
                 "limitc:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "isse:int:opt;"
                 , mvdegrain3Create, 0, plugin);
}
