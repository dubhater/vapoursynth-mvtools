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

#include "MVDegrain1.h"
#include "MVInterface.h"
#include "CopyCode.h"
#include "Overlap.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

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

    MVFilter *bleh;

    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;

    int dstShortPitch;

    OverlapsFunction *OVERSLUMA;
    OverlapsFunction *OVERSCHROMA;
    Denoise1Function *DEGRAINLUMA;
    Denoise1Function *DEGRAINCHROMA;
} MVDegrain1Data;


static void VS_CC mvdegrain1Init(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVDegrain1Data *d = (MVDegrain1Data *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvdegrain1GetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVDegrain1Data *d = (MVDegrain1Data *) * instanceData;

    if (activationReason == arInitial) {
        int offF = -1 * d->mvClipF->GetDeltaFrame();
        int offB = d->mvClipB->GetDeltaFrame();

        vsapi->requestFrameFilter(n, d->mvfw, frameCtx);
        vsapi->requestFrameFilter(n, d->mvbw, frameCtx);

        if (n + offF >= 0)
            vsapi->requestFrameFilter(n + offF, d->super, frameCtx);
        if (n + offB < d->vi->numFrames || !d->vi->numFrames)
            vsapi->requestFrameFilter(n + offB, d->super, frameCtx);

        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

        uint8_t *pDst[3], *pDstCur[3];
        const uint8_t *pSrcCur[3];
        const uint8_t *pSrc[3];
        const uint8_t *pRefB[3];
        const uint8_t *pRefF[3];
        int nDstPitches[3], nSrcPitches[3];
        int nRefBPitches[3], nRefFPitches[3];
        bool isUsableB, isUsableF;
        int tmpPitch = d->bleh->nBlkSizeX;
        int WSrc, WRefB, WRefF;
        unsigned short *pDstShort;
        int nLogPel = (d->bleh->nPel==4) ? 2 : (d->bleh->nPel==2) ? 1 : 0;
        // nLogPel=0 for nPel=1, 1 for nPel=2, 2 for nPel=4, i.e. (x*nPel) = (x<<nLogPel)

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

        const VSFrameRef *refB = NULL;
        const VSFrameRef *refF = NULL;

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

        for (int i = 0; i < 3; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
            pSrc[i] = vsapi->getReadPtr(src, i);
            nSrcPitches[i] = vsapi->getStride(src, i);
            if (isUsableF) {
                pRefF[i] = vsapi->getReadPtr(refF, i);
                nRefFPitches[i] = vsapi->getStride(refF, i);
            }
            if (isUsableB) {
                pRefB[i] = vsapi->getReadPtr(refB, i);
                nRefBPitches[i] = vsapi->getStride(refB, i);
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

        int ySubUV = (yRatioUV == 2) ? 1 : 0;


        MVGroupOfFrames *pRefBGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefFGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);


        short *winOver;
        short *winOverUV;

        OverlapWindows *OverWins = NULL;
        OverlapWindows *OverWinsUV = NULL;
        unsigned short *DstShort = NULL;
        if (nOverlapX > 0 || nOverlapY > 0)
        {
            OverWins = new OverlapWindows(nBlkSizeX, nBlkSizeY, nOverlapX, nOverlapY);
            OverWinsUV = new OverlapWindows(nBlkSizeX / 2, nBlkSizeY / yRatioUV, nOverlapX / 2, nOverlapY / yRatioUV);
            DstShort = new unsigned short[dstShortPitch * nHeight];
        }

        uint8_t *tmpBlock = new uint8_t[32*32];

        int nWidth_B = nBlkX * (nBlkSizeX - nOverlapX) + nOverlapX;
        int nHeight_B = nBlkY * (nBlkSizeY - nOverlapY) + nOverlapY;


        MVPlane *pPlanesB[3] = { };
        MVPlane *pPlanesF[3] = { };

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

        pDstCur[0] = pDst[0];
        pDstCur[1] = pDst[1];
        pDstCur[2] = pDst[2];
        pSrcCur[0] = pSrc[0];
        pSrcCur[1] = pSrc[1];
        pSrcCur[2] = pSrc[2];
        // -----------------------------------------------------------------------------

        // FIXME: duplicating Huge Chuncks of Code is a bad idea.
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
                        const uint8_t * pB, *pF;
                        int npB, npF;
                        int WRefB, WRefF, WSrc;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = (blockB.GetX()<<nLogPel) + blockB.GetMV().x;
                            int blyB = (blockB.GetY()<<nLogPel) + blockB.GetMV().y;
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
                            int blxF = (blockF.GetX()<<nLogPel) + blockF.GetMV().x;
                            int blyF = (blockF.GetY()<<nLogPel) + blockF.GetMV().y;
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
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WSrc = 256 - WRefB - WRefF;

                        d->DEGRAINLUMA(pDstCur[0] + xx, nDstPitches[0], pSrcCur[0]+ xx, nSrcPitches[0],
                                pB, npB, pF, npF, WSrc, WRefB, WRefF);

                        xx += (nBlkSizeX);

                        if (bx == nBlkX-1 && nWidth_B < nWidth) // right non-covered region
                        {
                            BitBlt(pDstCur[0] + nWidth_B, nDstPitches[0],
                                    pSrcCur[0] + nWidth_B, nSrcPitches[0], nWidth-nWidth_B, nBlkSizeY, isse);
                        }
                    }
                    pDstCur[0] += ( nBlkSizeY ) * nDstPitches[0];
                    pSrcCur[0] += ( nBlkSizeY ) * nSrcPitches[0];

                    if (by == nBlkY-1 && nHeight_B < nHeight) // bottom uncovered region
                    {
                        BitBlt(pDstCur[0], nDstPitches[0], pSrcCur[0], nSrcPitches[0], nWidth, nHeight-nHeight_B, isse);
                    }
                }
            }
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
                        const uint8_t * pB, *pF;
                        int npB, npF;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = (blockB.GetX()<<nLogPel) + blockB.GetMV().x;
                            int blyB = (blockB.GetY()<<nLogPel) + blockB.GetMV().y;
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
                            int blxF = (blockF.GetX()<<nLogPel) + blockF.GetMV().x;
                            int blyF = (blockF.GetY()<<nLogPel) + blockF.GetMV().y;
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
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WSrc = 256 - WRefB - WRefF;
                        // luma
                        d->DEGRAINLUMA(tmpBlock, tmpPitch, pSrcCur[0]+ xx, nSrcPitches[0], pB, npB, pF, npF, WSrc, WRefB, WRefF);
                        d->OVERSLUMA(pDstShort + xx, dstShortPitch, tmpBlock, tmpPitch, winOver, nBlkSizeX);

                        xx += (nBlkSizeX - nOverlapX);

                    }
                    pSrcCur[0] += (nBlkSizeY - nOverlapY) * nSrcPitches[0];
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
                    mvtools_LimitChanges_sse2(pDst[0], nDstPitches[0], pSrc[0], nSrcPitches[0], nWidth, nHeight, nLimit);
                else
                    LimitChanges_c(pDst[0], nDstPitches[0], pSrc[0], nSrcPitches[0], nWidth, nHeight, nLimit);
            }
        }

        //-----------------------------------------------------------------------------------
        // CHROMA plane U
        if (!(YUVplanes & UPLANE & nSuperModeYUV)) // v2.0.8.1
            BitBlt(pDstCur[1], nDstPitches[1], pSrcCur[1], nSrcPitches[1], nWidth>>1, nHeight>>ySubUV, isse);
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
                        const uint8_t * pBU, *pFU;
                        int npBU, npFU;
                        int WRefB, WRefF, WSrc;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = (blockB.GetX()<<nLogPel) + blockB.GetMV().x;
                            int blyB = (blockB.GetY()<<nLogPel) + blockB.GetMV().y;
                            pBU = pPlanesB[1]->GetPointer(blxB>>1, blyB>>ySubUV);
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
                            int blxF = (blockF.GetX()<<nLogPel) + blockF.GetMV().x;
                            int blyF = (blockF.GetY()<<nLogPel) + blockF.GetMV().y;
                            pFU = pPlanesF[1]->GetPointer(blxF>>1, blyF>>ySubUV);
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
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WSrc = 256 - WRefB - WRefF;

                        d->DEGRAINCHROMA(pDstCur[1] + (xx>>1), nDstPitches[1], pSrcCur[1]+ (xx>>1), nSrcPitches[1],
                                pBU, npBU, pFU, npFU, WSrc, WRefB, WRefF);

                        xx += (nBlkSizeX);

                        if (bx == nBlkX-1 && nWidth_B < nWidth) // right non-covered region
                        {
                            BitBlt(pDstCur[1] + (nWidth_B>>1), nDstPitches[1],
                                    pSrcCur[1] + (nWidth_B>>1), nSrcPitches[1], (nWidth-nWidth_B)>>1, (nBlkSizeY)>>ySubUV, isse);
                        }
                    }
                    pDstCur[1] += ( nBlkSizeY >>ySubUV) * nDstPitches[1] ;
                    pSrcCur[1] += ( nBlkSizeY >>ySubUV) * nSrcPitches[1] ;

                    if (by == nBlkY-1 && nHeight_B < nHeight) // bottom uncovered region
                    {
                        BitBlt(pDstCur[1], nDstPitches[1], pSrcCur[1], nSrcPitches[1], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
                    }
                }
            }
            else // overlap
            {
                pDstShort = DstShort;
                MemZoneSet(reinterpret_cast<unsigned char*>(pDstShort), 0, nWidth_B, nHeight_B>>ySubUV, 0, 0, dstShortPitch*2);
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
                        const uint8_t * pBU, *pFU;
                        int npBU, npFU;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = (blockB.GetX()<<nLogPel) + blockB.GetMV().x;
                            int blyB = (blockB.GetY()<<nLogPel) + blockB.GetMV().y;
                            pBU = pPlanesB[1]->GetPointer(blxB>>1, blyB>>ySubUV);
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
                            int blxF = (blockF.GetX()<<nLogPel) + blockF.GetMV().x;
                            int blyF = (blockF.GetY()<<nLogPel) + blockF.GetMV().y;
                            pFU = pPlanesF[1]->GetPointer(blxF>>1, blyF>>ySubUV);
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
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WSrc = 256 - WRefB - WRefF;

                        d->DEGRAINCHROMA(tmpBlock, tmpPitch, pSrcCur[1]+ (xx>>1), nSrcPitches[1],
                                pBU, npBU, pFU, npFU, WSrc, WRefB, WRefF);
                        d->OVERSCHROMA(pDstShort + (xx>>1), dstShortPitch, tmpBlock, tmpPitch, winOverUV, nBlkSizeX/2);

                        xx += (nBlkSizeX - nOverlapX);

                    }
                    pSrcCur[1] += ((nBlkSizeY - nOverlapY)>>ySubUV) * nSrcPitches[1] ;
                    pDstShort += ((nBlkSizeY - nOverlapY)>>ySubUV) * dstShortPitch;

                }
                Short2Bytes(pDst[1], nDstPitches[1], DstShort, dstShortPitch, nWidth_B>>1, nHeight_B>>ySubUV);
                if (nWidth_B < nWidth)
                    BitBlt(pDst[1] + (nWidth_B>>1), nDstPitches[1], pSrc[1] + (nWidth_B>>1), nSrcPitches[1], (nWidth-nWidth_B)>>1, nHeight_B>>ySubUV, isse);
                if (nHeight_B < nHeight) // bottom noncovered region
                    BitBlt(pDst[1] + nDstPitches[1]*(nHeight_B>>ySubUV), nDstPitches[1], pSrc[1] + nSrcPitches[1]*(nHeight_B>>ySubUV), nSrcPitches[1], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
            }
            if (nLimitC < 255) {
                if (isse)
                    mvtools_LimitChanges_sse2(pDst[1], nDstPitches[1], pSrc[1], nSrcPitches[1], nWidth/2, nHeight/yRatioUV, nLimitC);
                else
                    LimitChanges_c(pDst[1], nDstPitches[1], pSrc[1], nSrcPitches[1], nWidth/2, nHeight/yRatioUV, nLimitC);
            }
        }
        //--------------------------------------------------------------------------------

        // CHROMA plane V
        if (!(YUVplanes & VPLANE & nSuperModeYUV))
            BitBlt(pDstCur[2], nDstPitches[2], pSrcCur[2], nSrcPitches[2], nWidth>>1, nHeight>>ySubUV, isse);
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
                        const uint8_t * pBV, *pFV;
                        int npBV, npFV;
                        int WRefB, WRefF, WSrc;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = (blockB.GetX()<<nLogPel) + blockB.GetMV().x;
                            int blyB = (blockB.GetY()<<nLogPel) + blockB.GetMV().y;
                            pBV = pPlanesB[2]->GetPointer(blxB>>1, blyB>>ySubUV);
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
                            int blxF = (blockF.GetX()<<nLogPel) + blockF.GetMV().x;
                            int blyF = (blockF.GetY()<<nLogPel) + blockF.GetMV().y;
                            pFV = pPlanesF[2]->GetPointer(blxF>>1, blyF>>ySubUV);
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
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WSrc = 256 - WRefB - WRefF;

                        d->DEGRAINCHROMA(pDstCur[2] + (xx>>1), nDstPitches[2], pSrcCur[2]+ (xx>>1), nSrcPitches[2],
                                pBV, npBV, pFV, npFV, WSrc, WRefB, WRefF);

                        xx += (nBlkSizeX);

                        if (bx == nBlkX-1 && nWidth_B < nWidth) // right non-covered region
                        {
                            BitBlt(pDstCur[2] + (nWidth_B>>1), nDstPitches[2],
                                    pSrcCur[2] + (nWidth_B>>1), nSrcPitches[2], (nWidth-nWidth_B)>>1, nBlkSizeY>>ySubUV, isse);
                        }
                    }
                    pDstCur[2] += ( nBlkSizeY >>ySubUV) * nDstPitches[2] ;
                    pSrcCur[2] += ( nBlkSizeY >>ySubUV) * nSrcPitches[2] ;

                    if (by == nBlkY-1 && nHeight_B < nHeight) // bottom uncovered region
                    {
                        BitBlt(pDstCur[2], nDstPitches[2], pSrcCur[2], nSrcPitches[2], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
                    }
                }
            }
            else // overlap
            {
                pDstShort = DstShort;
                MemZoneSet(reinterpret_cast<unsigned char*>(pDstShort), 0, nWidth_B, nHeight_B>>ySubUV, 0, 0, dstShortPitch*2);
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
                        const uint8_t * pBV, *pFV;
                        int npBV, npFV;

                        if (isUsableB)
                        {
                            const FakeBlockData &blockB = ballsB.GetBlock(0, i);
                            int blxB = (blockB.GetX()<<nLogPel) + blockB.GetMV().x;
                            int blyB = (blockB.GetY()<<nLogPel) + blockB.GetMV().y;
                            pBV = pPlanesB[2]->GetPointer(blxB>>1, blyB>>ySubUV);
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
                            int blxF = (blockF.GetX()<<nLogPel) + blockF.GetMV().x;
                            int blyF = (blockF.GetY()<<nLogPel) + blockF.GetMV().y;
                            pFV = pPlanesF[2]->GetPointer(blxF>>1, blyF>>ySubUV);
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
                        WSrc = 256;
                        int WSum = WRefB + WRefF + WSrc + 1;
                        WRefB = WRefB*256/WSum; // normailize weights to 256
                        WRefF = WRefF*256/WSum;
                        WSrc = 256 - WRefB - WRefF;

                        d->DEGRAINCHROMA(tmpBlock, tmpPitch, pSrcCur[2]+ (xx>>1), nSrcPitches[2],
                                pBV, npBV, pFV, npFV, WSrc, WRefB, WRefF);
                        d->OVERSCHROMA(pDstShort + (xx>>1), dstShortPitch, tmpBlock, tmpPitch, winOverUV, nBlkSizeX/2);

                        xx += (nBlkSizeX - nOverlapX);

                    }
                    pSrcCur[2] += ((nBlkSizeY - nOverlapY)>>ySubUV) * nSrcPitches[2] ;
                    pDstShort += ((nBlkSizeY - nOverlapY)>>ySubUV) * dstShortPitch;

                }
                Short2Bytes(pDst[2], nDstPitches[2], DstShort, dstShortPitch, nWidth_B>>1, nHeight_B>>ySubUV);
                if (nWidth_B < nWidth)
                    BitBlt(pDst[2] + (nWidth_B>>1), nDstPitches[2], pSrc[2] + (nWidth_B>>1), nSrcPitches[2], (nWidth-nWidth_B)>>1, nHeight_B>>ySubUV, isse);
                if (nHeight_B < nHeight) // bottom noncovered region
                    BitBlt(pDst[2] + nDstPitches[2]*(nHeight_B>>ySubUV), nDstPitches[2], pSrc[2] + nSrcPitches[2]*(nHeight_B>>ySubUV), nSrcPitches[2], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
            }
            if (nLimitC < 255) {
                if (isse)
                    mvtools_LimitChanges_sse2(pDst[2], nDstPitches[2], pSrc[2], nSrcPitches[2], nWidth/2, nHeight/yRatioUV, nLimitC);
                else
                    LimitChanges_c(pDst[2], nDstPitches[2], pSrc[2], nSrcPitches[2], nWidth/2, nHeight/yRatioUV, nLimitC);
            }
        }
        //-------------------------------------------------------------------------------



        delete[] tmpBlock;

        if (OverWins)
            delete OverWins;
        if (OverWinsUV)
            delete OverWinsUV;
        if (DstShort)
            delete[] DstShort;

        delete pRefFGOF;
        delete pRefBGOF;

        if (refF)
            vsapi->freeFrame(refF);
        if (refB)
            vsapi->freeFrame(refB);

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvdegrain1Free(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVDegrain1Data *d = (MVDegrain1Data *)instanceData;

    delete d->mvClipB;
    delete d->mvClipF;

    delete d->bleh;

    vsapi->freeNode(d->super);
    vsapi->freeNode(d->mvfw);
    vsapi->freeNode(d->mvbw);
    vsapi->freeNode(d->node);
    free(d);
}


static void selectFunctions(MVDegrain1Data *d) {
    const int yRatioUV = d->bleh->yRatioUV;
    const int nBlkSizeX = d->bleh->nBlkSizeX;
    const int nBlkSizeY = d->bleh->nBlkSizeY;

    if (d->isse)
    {
        switch (nBlkSizeX)
        {
            case 32:
                if (nBlkSizeY==16) {          d->OVERSLUMA = mvtools_Overlaps32x16_sse2;  d->DEGRAINLUMA = Degrain1_sse2<32,16>;
                    if (yRatioUV==2) {         d->OVERSCHROMA = mvtools_Overlaps16x8_sse2; d->DEGRAINCHROMA = Degrain1_sse2<16,8>;         }
                    else {                     d->OVERSCHROMA = mvtools_Overlaps16x16_sse2;d->DEGRAINCHROMA = Degrain1_sse2<16,16>;         }
                } else if (nBlkSizeY==32) {    d->OVERSLUMA = mvtools_Overlaps32x32_sse2;  d->DEGRAINLUMA = Degrain1_sse2<32,32>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = mvtools_Overlaps16x16_sse2; d->DEGRAINCHROMA = Degrain1_sse2<16,16>;         }
                    else {                        d->OVERSCHROMA = mvtools_Overlaps16x32_sse2; d->DEGRAINCHROMA = Degrain1_sse2<16,32>;         }
                } break;
            case 16:
                if (nBlkSizeY==16) {          d->OVERSLUMA = mvtools_Overlaps16x16_sse2; d->DEGRAINLUMA = Degrain1_sse2<16,16>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = mvtools_Overlaps8x8_sse2; d->DEGRAINCHROMA = Degrain1_sse2<8,8>;         }
                    else {                        d->OVERSCHROMA = mvtools_Overlaps8x16_sse2;d->DEGRAINCHROMA = Degrain1_sse2<8,16>;         }
                } else if (nBlkSizeY==8) {    d->OVERSLUMA = mvtools_Overlaps16x8_sse2;  d->DEGRAINLUMA = Degrain1_sse2<16,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = mvtools_Overlaps8x4_sse2; d->DEGRAINCHROMA = Degrain1_sse2<8,4>;         }
                    else {                        d->OVERSCHROMA = mvtools_Overlaps8x8_sse2; d->DEGRAINCHROMA = Degrain1_sse2<8,8>;         }
                } else if (nBlkSizeY==2) {    d->OVERSLUMA = mvtools_Overlaps16x2_sse2;  d->DEGRAINLUMA = Degrain1_sse2<16,2>;
                    if (yRatioUV==2) {         d->OVERSCHROMA = mvtools_Overlaps8x1_sse2; d->DEGRAINCHROMA = Degrain1_sse2<8,1>;         }
                    else {                        d->OVERSCHROMA = mvtools_Overlaps8x2_sse2; d->DEGRAINCHROMA = Degrain1_sse2<8,2>;         }
                }
                break;
            case 4:
                d->OVERSLUMA = mvtools_Overlaps4x4_sse2;    d->DEGRAINLUMA = Degrain1_mmx<4,4>;
                if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<2,2>;    d->DEGRAINCHROMA = Degrain1_C<2,2>;         }
                else {                        d->OVERSCHROMA = Overlaps_C<2,4>;    d->DEGRAINCHROMA = Degrain1_C<2,4>;         }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {           d->OVERSLUMA = mvtools_Overlaps8x8_sse2;    d->DEGRAINLUMA = Degrain1_sse2<8,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = mvtools_Overlaps4x4_sse2;  d->DEGRAINCHROMA = Degrain1_mmx<4,4>;         }
                    else {                        d->OVERSCHROMA = mvtools_Overlaps4x8_sse2;  d->DEGRAINCHROMA = Degrain1_mmx<4,8>;         }
                }else if (nBlkSizeY==4) {     d->OVERSLUMA = mvtools_Overlaps8x4_sse2;    d->DEGRAINLUMA = Degrain1_sse2<8,4>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = mvtools_Overlaps4x2_sse2;    d->DEGRAINCHROMA = Degrain1_mmx<4,2>;         }
                    else {                        d->OVERSCHROMA = mvtools_Overlaps4x4_sse2;  d->DEGRAINCHROMA = Degrain1_mmx<4,4>;         }
                }
        }
    }
    else
    {
        switch (nBlkSizeX)
        {
            case 32:
                if (nBlkSizeY==16) {          d->OVERSLUMA = Overlaps_C<32,16>;  d->DEGRAINLUMA = Degrain1_C<32,16>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<16,8>; d->DEGRAINCHROMA = Degrain1_C<16,8>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<16,16>;d->DEGRAINCHROMA = Degrain1_C<16,16>;         }
                } else if (nBlkSizeY==32) {    d->OVERSLUMA = Overlaps_C<32,32>;   d->DEGRAINLUMA = Degrain1_C<32,32>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<16,16>;  d->DEGRAINCHROMA = Degrain1_C<16,16>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<16,32>;  d->DEGRAINCHROMA = Degrain1_C<16,32>;         }
                } break;
            case 16:
                if (nBlkSizeY==16) {          d->OVERSLUMA = Overlaps_C<16,16>;  d->DEGRAINLUMA = Degrain1_C<16,16>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<8,8>;  d->DEGRAINCHROMA = Degrain1_C<8,8>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<8,16>; d->DEGRAINCHROMA = Degrain1_C<8,16>;         }
                } else if (nBlkSizeY==8) {    d->OVERSLUMA = Overlaps_C<16,8>;   d->DEGRAINLUMA = Degrain1_C<16,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<8,4>;  d->DEGRAINCHROMA = Degrain1_C<8,4>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<8,8>;  d->DEGRAINCHROMA = Degrain1_C<8,8>;         }
                } else if (nBlkSizeY==2) {    d->OVERSLUMA = Overlaps_C<16,2>;   d->DEGRAINLUMA = Degrain1_C<16,2>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<8,1>;  d->DEGRAINCHROMA = Degrain1_C<8,1>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<8,2>;  d->DEGRAINCHROMA = Degrain1_C<8,2>;         }
                }
                break;
            case 4:
                d->OVERSLUMA = Overlaps_C<4,4>;    d->DEGRAINLUMA = Degrain1_C<4,4>;
                if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<2,2>;  d->DEGRAINCHROMA = Degrain1_C<2,2>;         }
                else {                        d->OVERSCHROMA = Overlaps_C<2,4>;  d->DEGRAINCHROMA = Degrain1_C<2,4>;         }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {           d->OVERSLUMA = Overlaps_C<8,8>;    d->DEGRAINLUMA = Degrain1_C<8,8>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<4,4>;  d->DEGRAINCHROMA = Degrain1_C<4,4>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<4,8>;  d->DEGRAINCHROMA = Degrain1_C<4,8>;         }
                }else if (nBlkSizeY==4) {     d->OVERSLUMA = Overlaps_C<8,4>;    d->DEGRAINLUMA = Degrain1_C<8,4>;
                    if (yRatioUV==2) {            d->OVERSCHROMA = Overlaps_C<4,2>;  d->DEGRAINCHROMA = Degrain1_C<4,2>;         }
                    else {                        d->OVERSCHROMA = Overlaps_C<4,4>;  d->DEGRAINCHROMA = Degrain1_C<4,4>;         }
                }
        }
    }
}


static void VS_CC mvdegrain1Create(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVDegrain1Data d;
    MVDegrain1Data *data;

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
        d.isse = 1;


    int planes[5] = { YPLANE, UPLANE, VPLANE, UVPLANES, YUVPLANES };
    d.YUVplanes = planes[plane];


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("Degrain1: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
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
            vsapi->setError(out, "Degrain1: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);

    // XXX Fuck all this trying.
    try {
        d.mvClipB = new MVClipDicks(d.mvbw, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw);
        return;
    }

    try {
        d.mvClipF = new MVClipDicks(d.mvfw, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        return;
    }

    try {
        d.bleh = new MVFilter(d.mvfw, "Degrain1", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
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
            vsapi->setError(out, "Degrain1: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "Degrain1: mvfw must be generated with isb=False.");
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

    d.thSAD = d.thSAD * d.mvClipB->GetThSCD1() / d.nSCD1; // normalize to block SAD
    d.thSADC = d.thSADC * d.mvClipB->GetThSCD1() / d.nSCD1; // chroma threshold, normalized to block SAD



    d.dstShortPitch = ((d.bleh->nWidth + 15)/16)*16;


    selectFunctions(&d);


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.bleh->nHeight != nHeightS || d.bleh->nHeight != d.vi->height || d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2 || d.bleh->nWidth != d.vi->width) {
        vsapi->setError(out, "Degrain1: wrong source or super clip frame size.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->id != pfYUV420P8) {
        vsapi->setError(out, "Degrain1: input clip must be YUV420P8 with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }


    data = (MVDegrain1Data *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Degrain1", mvdegrain1Init, mvdegrain1GetFrame, mvdegrain1Free, fmParallel, 0, data, core);
}


void mvdegrain1Register(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Degrain1",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "thsad:int:opt;"
                 "thsadc:int:opt;"
                 "plane:int:opt;"
                 "limit:int:opt;"
                 "limitc:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "isse:int:opt;"
                 , mvdegrain1Create, 0, plugin);
}
