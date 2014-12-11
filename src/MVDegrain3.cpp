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
#include "MVDegrains.h"
#include "MVInterface.h"
#include "CopyCode.h"
#include "Overlap.h"


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

    int thSAD[3];
    int YUVplanes;
    int nLimit[3];
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

    OverlapsFunction OVERS[3];
    Denoise3Function DEGRAIN[3];

    bool process[3];

    int xSubUV;
    int ySubUV;

    int nWidth[3];
    int nHeight[3];
    int nOverlapX[3];
    int nOverlapY[3];
    int nBlkSizeX[3];
    int nBlkSizeY[3];
    int nWidth_B[3];
    int nHeight_B[3];
} MVDegrain3Data;


static void VS_CC mvdegrain3Init(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVDegrain3Data *d = (MVDegrain3Data *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static inline void normaliseWeights(int &WSrc, int &WRefB, int &WRefF, int &WRefB2, int &WRefF2, int &WRefB3, int &WRefF3) {
    WSrc = 256;
    int WSum = WRefB + WRefF + WSrc + WRefB2 + WRefF2 + WRefB3 + WRefF3 + 1;
    WRefB = WRefB * 256 / WSum; // normailize weights to 256
    WRefF = WRefF * 256 / WSum;
    WRefB2 = WRefB2 * 256 / WSum;
    WRefF2 = WRefF2 * 256 / WSum;
    WRefB3 = WRefB3 * 256 / WSum;
    WRefF3 = WRefF3 * 256 / WSum;
    WSrc = 256 - WRefB - WRefF - WRefB2 - WRefF2 - WRefB3 - WRefF3;
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
        if (n + offB < d->vi->numFrames || !d->vi->numFrames)
            vsapi->requestFrameFilter(n + offB, d->super, frameCtx);
        if (n + offB2 < d->vi->numFrames || !d->vi->numFrames)
            vsapi->requestFrameFilter(n + offB2, d->super, frameCtx);
        if (n + offB3 < d->vi->numFrames || !d->vi->numFrames)
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
        int nLogPel = (d->bleh->nPel == 4) ? 2 : (d->bleh->nPel == 2) ? 1 : 0;

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

        if (isUsableF3) {
            int offF3 = -1 * d->mvClipF3->GetDeltaFrame();
            refF3 = vsapi->getFrameFilter(n + offF3, d->super, frameCtx);
        }
        if (isUsableF2) {
            int offF2 = -1 * d->mvClipF2->GetDeltaFrame();
            refF2 = vsapi->getFrameFilter(n + offF2, d->super, frameCtx);
        }
        if (isUsableF) {
            int offF = -1 * d->mvClipF->GetDeltaFrame();
            refF = vsapi->getFrameFilter(n + offF, d->super, frameCtx);
        }
        if (isUsableB) {
            int offB = d->mvClipB->GetDeltaFrame();
            refB = vsapi->getFrameFilter(n + offB, d->super, frameCtx);
        }
        if (isUsableB2) {
            int offB2 = d->mvClipB2->GetDeltaFrame();
            refB2 = vsapi->getFrameFilter(n + offB2, d->super, frameCtx);
        }
        if (isUsableB3) {
            int offB3 = d->mvClipB3->GetDeltaFrame();
            refB3 = vsapi->getFrameFilter(n + offB3, d->super, frameCtx);
        }

        for (int i = 0; i < d->vi->format->numPlanes; i++) {
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

        const int xSubUV = d->xSubUV;
        const int ySubUV = d->ySubUV;
        const int yRatioUV = d->bleh->yRatioUV;
        const int nBlkX = d->bleh->nBlkX;
        const int nBlkY = d->bleh->nBlkY;
        const int isse = d->isse;
        const int YUVplanes = d->YUVplanes;
        const int dstShortPitch = d->dstShortPitch;
        const int *nWidth = d->nWidth;
        const int *nHeight = d->nHeight;
        const int *nOverlapX = d->nOverlapX;
        const int *nOverlapY = d->nOverlapY;
        const int *nBlkSizeX = d->nBlkSizeX;
        const int *nBlkSizeY = d->nBlkSizeY;
        const int *nWidth_B = d->nWidth_B;
        const int *nHeight_B = d->nHeight_B;
        const int *thSAD = d->thSAD;
        const int *nLimit = d->nLimit;


        MVGroupOfFrames *pRefBGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefFGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefB2GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefF2GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefB3GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);
        MVGroupOfFrames *pRefF3GOF = new MVGroupOfFrames(d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, yRatioUV);


        short *winOver;

        OverlapWindows *OverWins[3] = { NULL, NULL, NULL };
        unsigned short *DstShort = NULL;
        if (nOverlapX[0] > 0 || nOverlapY[0] > 0) {
            OverWins[0] = new OverlapWindows(nBlkSizeX[0], nBlkSizeY[0], nOverlapX[0], nOverlapY[0]);
            OverWins[1] = new OverlapWindows(nBlkSizeX[1], nBlkSizeY[1], nOverlapX[1], nOverlapY[1]);
            OverWins[2] = OverWins[1];
            DstShort = new unsigned short[d->dstShortPitch * nHeight[0]];
        }

        uint8_t *tmpBlock = new uint8_t[32*32];

        MVPlane *pPlanesB[3] = { };
        MVPlane *pPlanesF[3] = { };
        MVPlane *pPlanesB2[3] = { };
        MVPlane *pPlanesF2[3] = { };
        MVPlane *pPlanesB3[3] = { };
        MVPlane *pPlanesF3[3] = { };

        if (isUsableF3) {
            pRefF3GOF->Update(YUVplanes, (uint8_t*)pRefF3[0], nRefF3Pitches[0], (uint8_t*)pRefF3[1], nRefF3Pitches[1], (uint8_t*)pRefF3[2], nRefF3Pitches[2]);
            if (YUVplanes & YPLANE)
                pPlanesF3[0] = pRefF3GOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesF3[1] = pRefF3GOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesF3[2] = pRefF3GOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableF2) {
            pRefF2GOF->Update(YUVplanes, (uint8_t*)pRefF2[0], nRefF2Pitches[0], (uint8_t*)pRefF2[1], nRefF2Pitches[1], (uint8_t*)pRefF2[2], nRefF2Pitches[2]);
            if (YUVplanes & YPLANE)
                pPlanesF2[0] = pRefF2GOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesF2[1] = pRefF2GOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesF2[2] = pRefF2GOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableF) {
            pRefFGOF->Update(YUVplanes, (uint8_t*)pRefF[0], nRefFPitches[0], (uint8_t*)pRefF[1], nRefFPitches[1], (uint8_t*)pRefF[2], nRefFPitches[2]);
            if (YUVplanes & YPLANE)
                pPlanesF[0] = pRefFGOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesF[1] = pRefFGOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesF[2] = pRefFGOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableB) {
            pRefBGOF->Update(YUVplanes, (uint8_t*)pRefB[0], nRefBPitches[0], (uint8_t*)pRefB[1], nRefBPitches[1], (uint8_t*)pRefB[2], nRefBPitches[2]);// v2.0
            if (YUVplanes & YPLANE)
                pPlanesB[0] = pRefBGOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesB[1] = pRefBGOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesB[2] = pRefBGOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableB2) {
            pRefB2GOF->Update(YUVplanes, (uint8_t*)pRefB2[0], nRefB2Pitches[0], (uint8_t*)pRefB2[1], nRefB2Pitches[1], (uint8_t*)pRefB2[2], nRefB2Pitches[2]);// v2.0
            if (YUVplanes & YPLANE)
                pPlanesB2[0] = pRefB2GOF->GetFrame(0)->GetPlane(YPLANE);
            if (YUVplanes & UPLANE)
                pPlanesB2[1] = pRefB2GOF->GetFrame(0)->GetPlane(UPLANE);
            if (YUVplanes & VPLANE)
                pPlanesB2[2] = pRefB2GOF->GetFrame(0)->GetPlane(VPLANE);
        }
        if (isUsableB3) {
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

        for (int plane = 0; plane < d->vi->format->numPlanes; plane++) {
            if (!d->process[plane]) {
                memcpy(pDstCur[plane], pSrcCur[plane], nSrcPitches[plane] * nHeight[plane]);
                continue;
            }

            if (nOverlapX[0] == 0 && nOverlapY[0] == 0) {
                for (int by = 0; by < nBlkY; by++) {
                    int xx = 0;
                    for (int bx = 0; bx < nBlkX; bx++) {
                        int i = by * nBlkX + bx;
                        const uint8_t *pB, *pF, *pB2, *pF2, *pB3, *pF3;
                        int npB, npF, npB2, npF2, npB3, npF3;

                        useBlock(pB, npB, WRefB, isUsableB, ballsB, i, pPlanesB, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF, npF, WRefF, isUsableF, ballsF, i, pPlanesF, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pB2, npB2, WRefB2, isUsableB2, ballsB2, i, pPlanesB2, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF2, npF2, WRefF2, isUsableF2, ballsF2, i, pPlanesF2, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pB3, npB3, WRefB3, isUsableB3, ballsB3, i, pPlanesB3, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF3, npF3, WRefF3, isUsableF3, ballsF3, i, pPlanesF3, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights(WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);

                        d->DEGRAIN[plane](pDstCur[plane] + xx, nDstPitches[plane], pSrcCur[plane] + xx, nSrcPitches[plane],
                                pB, npB, pF, npF, pB2, npB2, pF2, npF2, pB3, npB3, pF3, npF3,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);

                        xx += nBlkSizeX[plane];

                        if (bx == nBlkX - 1 && nWidth_B[0] < nWidth[0]) // right non-covered region
                            vs_bitblt(pDstCur[plane] + nWidth_B[plane], nDstPitches[plane],
                                      pSrcCur[plane] + nWidth_B[plane], nSrcPitches[plane],
                                      nWidth[plane] - nWidth_B[plane], nBlkSizeY[plane]);
                    }
                    pDstCur[plane] += nBlkSizeY[plane] * (nDstPitches[plane]);
                    pSrcCur[plane] += nBlkSizeY[plane] * (nSrcPitches[plane]);

                    if (by == nBlkY - 1 && nHeight_B[0] < nHeight[0]) // bottom uncovered region
                        // chroma u
                        vs_bitblt(pDstCur[plane], nDstPitches[plane],
                                  pSrcCur[plane], nSrcPitches[plane],
                                  nWidth[plane], nHeight[plane] - nHeight_B[plane]);
                }
            } else {// overlap
                pDstShort = DstShort;
                memset(pDstShort, 0, dstShortPitch * 2 * nHeight_B[0]);

                for (int by = 0; by < nBlkY; by++) {
                    int wby = ((by + nBlkY - 3) / (nBlkY - 2)) * 3;
                    int xx = 0;
                    for (int bx = 0; bx < nBlkX; bx++) {
                        // select window
                        int wbx = (bx + nBlkX - 3) / (nBlkX - 2);
                        winOver = OverWins[plane]->GetWindow(wby + wbx);

                        int i = by * nBlkX + bx;
                        const uint8_t *pB, *pF, *pB2, *pF2, *pB3, *pF3;
                        int npB, npF, npB2, npF2, npB3, npF3;

                        useBlock(pB, npB, WRefB, isUsableB, ballsB, i, pPlanesB, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF, npF, WRefF, isUsableF, ballsF, i, pPlanesF, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pB2, npB2, WRefB2, isUsableB2, ballsB2, i, pPlanesB2, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF2, npF2, WRefF2, isUsableF2, ballsF2, i, pPlanesF2, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pB3, npB3, WRefB3, isUsableB3, ballsB3, i, pPlanesB3, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF3, npF3, WRefF3, isUsableF3, ballsF3, i, pPlanesF3, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights(WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);

                        d->DEGRAIN[plane](tmpBlock, tmpPitch, pSrcCur[plane] + xx, nSrcPitches[plane],
                                pB, npB, pF, npF, pB2, npB2, pF2, npF2, pB3, npB3, pF3, npF3,
                                WSrc, WRefB, WRefF, WRefB2, WRefF2, WRefB3, WRefF3);
                        d->OVERS[plane](pDstShort + xx, dstShortPitch, tmpBlock, tmpPitch, winOver, nBlkSizeX[plane]);

                        xx += nBlkSizeX[plane] - nOverlapX[plane];

                    }
                    pSrcCur[plane] += (nBlkSizeY[plane] - nOverlapY[plane]) * nSrcPitches[plane];
                    pDstShort += (nBlkSizeY[plane] - nOverlapY[plane]) * dstShortPitch;
                }

                Short2Bytes(pDst[plane], nDstPitches[plane], DstShort, dstShortPitch, nWidth_B[plane], nHeight_B[plane]);

                if (nWidth_B[0] < nWidth[0])
                    vs_bitblt(pDst[plane] + nWidth_B[plane], nDstPitches[plane],
                              pSrc[plane] + nWidth_B[plane], nSrcPitches[plane],
                              nWidth[plane] - nWidth_B[plane], nHeight_B[plane]);

                if (nHeight_B < nHeight) // bottom noncovered region
                    vs_bitblt(pDst[plane] + nDstPitches[plane] * nHeight_B[plane], nDstPitches[plane],
                              pSrc[plane] + nSrcPitches[plane] * nHeight_B[plane], nSrcPitches[plane],
                              nWidth[plane], nHeight[plane] - nHeight_B[plane]);
            }
            if (nLimit[plane] < 255) {
                if (isse)
                    mvtools_LimitChanges_sse2(pDst[plane], nDstPitches[plane],
                                              pSrc[plane], nSrcPitches[plane],
                                              nWidth[plane], nHeight[plane], nLimit[plane]);
                else
                    LimitChanges_c(pDst[plane], nDstPitches[plane],
                                   pSrc[plane], nSrcPitches[plane],
                                   nWidth[plane], nHeight[plane], nLimit[plane]);
            }
        }


        delete[] tmpBlock;

        if (OverWins[0]) {
            delete OverWins[0];
            delete OverWins[1];
        }
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
                if (nBlkSizeY==16) {          d->OVERS[0] = mvtools_Overlaps32x16_sse2;  d->DEGRAIN[0] = Degrain3_sse2<32,16>;
                    if (yRatioUV==2) {         d->OVERS[1] = mvtools_Overlaps16x8_sse2; d->DEGRAIN[1] = Degrain3_sse2<16,8>;         }
                    else {                     d->OVERS[1] = mvtools_Overlaps16x16_sse2;d->DEGRAIN[1] = Degrain3_sse2<16,16>;         }
                } else if (nBlkSizeY==32) {    d->OVERS[0] = mvtools_Overlaps32x32_sse2;  d->DEGRAIN[0] = Degrain3_sse2<32,32>;
                    if (yRatioUV==2) {            d->OVERS[1] = mvtools_Overlaps16x16_sse2; d->DEGRAIN[1] = Degrain3_sse2<16,16>;         }
                    else {                        d->OVERS[1] = mvtools_Overlaps16x32_sse2; d->DEGRAIN[1] = Degrain3_sse2<16,32>;         }
                } break;
            case 16:
                if (nBlkSizeY==16) {          d->OVERS[0] = mvtools_Overlaps16x16_sse2; d->DEGRAIN[0] = Degrain3_sse2<16,16>;
                    if (yRatioUV==2) {            d->OVERS[1] = mvtools_Overlaps8x8_sse2; d->DEGRAIN[1] = Degrain3_sse2<8,8>;         }
                    else {                        d->OVERS[1] = mvtools_Overlaps8x16_sse2;d->DEGRAIN[1] = Degrain3_sse2<8,16>;         }
                } else if (nBlkSizeY==8) {    d->OVERS[0] = mvtools_Overlaps16x8_sse2;  d->DEGRAIN[0] = Degrain3_sse2<16,8>;
                    if (yRatioUV==2) {            d->OVERS[1] = mvtools_Overlaps8x4_sse2; d->DEGRAIN[1] = Degrain3_sse2<8,4>;         }
                    else {                        d->OVERS[1] = mvtools_Overlaps8x8_sse2; d->DEGRAIN[1] = Degrain3_sse2<8,8>;         }
                } else if (nBlkSizeY==2) {    d->OVERS[0] = mvtools_Overlaps16x2_sse2;  d->DEGRAIN[0] = Degrain3_sse2<16,2>;
                    if (yRatioUV==2) {         d->OVERS[1] = mvtools_Overlaps8x1_sse2; d->DEGRAIN[1] = Degrain3_sse2<8,1>;         }
                    else {                        d->OVERS[1] = mvtools_Overlaps8x2_sse2; d->DEGRAIN[1] = Degrain3_sse2<8,2>;         }
                }
                break;
            case 4:
                d->OVERS[0] = mvtools_Overlaps4x4_sse2;    d->DEGRAIN[0] = Degrain3_sse2<4,4>;
                if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<2,2>;    d->DEGRAIN[1] = Degrain3_C<2,2>;         }
                else {                        d->OVERS[1] = Overlaps_C<2,4>;    d->DEGRAIN[1] = Degrain3_C<2,4>;         }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {           d->OVERS[0] = mvtools_Overlaps8x8_sse2;    d->DEGRAIN[0] = Degrain3_sse2<8,8>;
                    if (yRatioUV==2) {            d->OVERS[1] = mvtools_Overlaps4x4_sse2;  d->DEGRAIN[1] = Degrain3_sse2<4,4>;         }
                    else {                        d->OVERS[1] = mvtools_Overlaps4x8_sse2;  d->DEGRAIN[1] = Degrain3_sse2<4,8>;         }
                }else if (nBlkSizeY==4) {     d->OVERS[0] = mvtools_Overlaps8x4_sse2;    d->DEGRAIN[0] = Degrain3_sse2<8,4>;
                    if (yRatioUV==2) {            d->OVERS[1] = mvtools_Overlaps4x2_sse2;    d->DEGRAIN[1] = Degrain3_sse2<4,2>;         }
                    else {                        d->OVERS[1] = mvtools_Overlaps4x4_sse2;  d->DEGRAIN[1] = Degrain3_sse2<4,4>;         }
                }
        }
    }
    else
    {
        switch (nBlkSizeX)
        {
            case 32:
                if (nBlkSizeY==16) {          d->OVERS[0] = Overlaps_C<32,16>;  d->DEGRAIN[0] = Degrain3_C<32,16>;
                    if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<16,8>; d->DEGRAIN[1] = Degrain3_C<16,8>;         }
                    else {                        d->OVERS[1] = Overlaps_C<16,16>;d->DEGRAIN[1] = Degrain3_C<16,16>;         }
                } else if (nBlkSizeY==32) {    d->OVERS[0] = Overlaps_C<32,32>;   d->DEGRAIN[0] = Degrain3_C<32,32>;
                    if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<16,16>;  d->DEGRAIN[1] = Degrain3_C<16,16>;         }
                    else {                        d->OVERS[1] = Overlaps_C<16,32>;  d->DEGRAIN[1] = Degrain3_C<16,32>;         }
                } break;
            case 16:
                if (nBlkSizeY==16) {          d->OVERS[0] = Overlaps_C<16,16>;  d->DEGRAIN[0] = Degrain3_C<16,16>;
                    if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<8,8>;  d->DEGRAIN[1] = Degrain3_C<8,8>;         }
                    else {                        d->OVERS[1] = Overlaps_C<8,16>; d->DEGRAIN[1] = Degrain3_C<8,16>;         }
                } else if (nBlkSizeY==8) {    d->OVERS[0] = Overlaps_C<16,8>;   d->DEGRAIN[0] = Degrain3_C<16,8>;
                    if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<8,4>;  d->DEGRAIN[1] = Degrain3_C<8,4>;         }
                    else {                        d->OVERS[1] = Overlaps_C<8,8>;  d->DEGRAIN[1] = Degrain3_C<8,8>;         }
                } else if (nBlkSizeY==2) {    d->OVERS[0] = Overlaps_C<16,2>;   d->DEGRAIN[0] = Degrain3_C<16,2>;
                    if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<8,1>;  d->DEGRAIN[1] = Degrain3_C<8,1>;         }
                    else {                        d->OVERS[1] = Overlaps_C<8,2>;  d->DEGRAIN[1] = Degrain3_C<8,2>;         }
                }
                break;
            case 4:
                d->OVERS[0] = Overlaps_C<4,4>;    d->DEGRAIN[0] = Degrain3_C<4,4>;
                if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<2,2>;  d->DEGRAIN[1] = Degrain3_C<2,2>;         }
                else {                        d->OVERS[1] = Overlaps_C<2,4>;  d->DEGRAIN[1] = Degrain3_C<2,4>;         }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {           d->OVERS[0] = Overlaps_C<8,8>;    d->DEGRAIN[0] = Degrain3_C<8,8>;
                    if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<4,4>;  d->DEGRAIN[1] = Degrain3_C<4,4>;         }
                    else {                        d->OVERS[1] = Overlaps_C<4,8>;  d->DEGRAIN[1] = Degrain3_C<4,8>;         }
                }else if (nBlkSizeY==4) {     d->OVERS[0] = Overlaps_C<8,4>;    d->DEGRAIN[0] = Degrain3_C<8,4>;
                    if (yRatioUV==2) {            d->OVERS[1] = Overlaps_C<4,2>;  d->DEGRAIN[1] = Degrain3_C<4,2>;         }
                    else {                        d->OVERS[1] = Overlaps_C<4,4>;  d->DEGRAIN[1] = Degrain3_C<4,4>;         }
                }
        }
    }

    d->OVERS[2] = d->OVERS[1];
    d->DEGRAIN[2] = d->DEGRAIN[1];
}


static void VS_CC mvdegrain3Create(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVDegrain3Data d;
    MVDegrain3Data *data;

    int err;

    d.thSAD[0] = vsapi->propGetInt(in, "thsad", 0, &err);
    if (err)
        d.thSAD[0] = 400;

    d.thSAD[1] = d.thSAD[2] = vsapi->propGetInt(in, "thsadc", 0, &err);
    if (err)
        d.thSAD[1] = d.thSAD[2] = d.thSAD[0];

    int plane = vsapi->propGetInt(in, "plane", 0, &err);
    if (err)
        plane = 4;

    d.nLimit[0] = vsapi->propGetInt(in, "limit", 0, &err);
    if (err)
        d.nLimit[0] = 255;

    d.nLimit[1] = d.nLimit[2] = vsapi->propGetInt(in, "limitc", 0, &err);
    if (err)
        d.nLimit[1] = d.nLimit[2] = d.nLimit[0];

    d.nSCD1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.nSCD1 = MV_DEFAULT_SCD1;

    d.nSCD2 = vsapi->propGetInt(in, "thscd2", 0, &err);
    if (err)
        d.nSCD2 = MV_DEFAULT_SCD2;

    d.isse = vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;


    if (plane < 0 || plane > 4) {
        vsapi->setError(out, "Degrain3: plane must be between 0 and 4 (inclusive).");
        return;
    }

    if (d.nLimit[0] < 0 || d.nLimit[0] > 255) {
        vsapi->setError(out, "Degrain3: limit must be between 0 and 255 (inclusive).");
        return;
    }

    if (d.nLimit[1] < 0 || d.nLimit[1] > 255) {
        vsapi->setError(out, "Degrain3: limitc must be between 0 and 255 (inclusive).");
        return;
    }

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

    // XXX Yoda had the right idea.
    try {
        d.mvClipB = new MVClipDicks(d.mvbw, d.nSCD1, d.nSCD2, vsapi);
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
        d.mvClipF = new MVClipDicks(d.mvfw, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        delete d.mvClipB;
        return;
    }

    try {
        d.mvClipB2 = new MVClipDicks(d.mvbw2, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    try {
        d.mvClipF2 = new MVClipDicks(d.mvfw2, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        delete d.mvClipB;
        delete d.mvClipF;
        delete d.mvClipB2;
        return;
    }

    try {
        d.mvClipB3 = new MVClipDicks(d.mvbw3, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        delete d.mvClipB;
        delete d.mvClipF;
        delete d.mvClipB2;
        delete d.mvClipF2;
        return;
    }

    try {
        d.mvClipF3 = new MVClipDicks(d.mvfw3, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        delete d.mvClipB;
        delete d.mvClipF;
        delete d.mvClipB2;
        delete d.mvClipF2;
        delete d.mvClipB3;
        return;
    }

    try {
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
        delete d.mvClipB;
        delete d.mvClipF;
        delete d.mvClipB2;
        delete d.mvClipF2;
        delete d.mvClipB3;
        delete d.mvClipF3;
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
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        delete d.mvClipB;
        delete d.mvClipF;
        delete d.mvClipB2;
        delete d.mvClipF2;
        delete d.mvClipB3;
        delete d.mvClipF3;
        delete d.bleh;
        return;
    }

    d.thSAD[0] = d.thSAD[0] * d.mvClipB->GetThSCD1() / d.nSCD1; // normalize to block SAD
    d.thSAD[1] = d.thSAD[2] = d.thSAD[1] * d.mvClipB->GetThSCD1() / d.nSCD1; // chroma threshold, normalized to block SAD



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
        delete d.mvClipB;
        delete d.mvClipF;
        delete d.mvClipB2;
        delete d.mvClipF2;
        delete d.mvClipB3;
        delete d.mvClipF3;
        delete d.bleh;
        return;
    }

    int id = d.vi->format->id;
    if (!isConstantFormat(d.vi) || (id != pfYUV420P8 && id != pfYUV422P8)) {
        vsapi->setError(out, "Degrain3: input clip must be YUV420P8 or YUV422P8, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw2);
        vsapi->freeNode(d.mvbw2);
        vsapi->freeNode(d.mvfw3);
        vsapi->freeNode(d.mvbw3);
        vsapi->freeNode(d.node);
        delete d.mvClipB;
        delete d.mvClipF;
        delete d.mvClipB2;
        delete d.mvClipF2;
        delete d.mvClipB3;
        delete d.mvClipF3;
        delete d.bleh;
        return;
    }

    d.process[0] = d.YUVplanes & YPLANE;
    d.process[1] = d.YUVplanes & UPLANE & d.nSuperModeYUV;
    d.process[2] = d.YUVplanes & VPLANE & d.nSuperModeYUV;

    d.xSubUV = d.vi->format->subSamplingW;
    d.ySubUV = d.vi->format->subSamplingH;

    d.nWidth[0] = d.bleh->nWidth;
    d.nWidth[1] = d.nWidth[2] = d.nWidth[0] >> d.xSubUV;

    d.nHeight[0] = d.bleh->nHeight;
    d.nHeight[1] = d.nHeight[2] = d.nHeight[0] >> d.ySubUV;

    d.nOverlapX[0] = d.bleh->nOverlapX;
    d.nOverlapX[1] = d.nOverlapX[2] = d.nOverlapX[0] >> d.xSubUV;

    d.nOverlapY[0] = d.bleh->nOverlapY;
    d.nOverlapY[1] = d.nOverlapY[2] = d.nOverlapY[0] >> d.ySubUV;

    d.nBlkSizeX[0] = d.bleh->nBlkSizeX;
    d.nBlkSizeX[1] = d.nBlkSizeX[2] = d.nBlkSizeX[0] >> d.xSubUV;

    d.nBlkSizeY[0] = d.bleh->nBlkSizeY;
    d.nBlkSizeY[1] = d.nBlkSizeY[2] = d.nBlkSizeY[0] >> d.ySubUV;

    d.nWidth_B[0] = d.bleh->nBlkX * (d.nBlkSizeX[0] - d.nOverlapX[0]) + d.nOverlapX[0];
    d.nWidth_B[1] = d.nWidth_B[2] = d.nWidth_B[0] >> d.xSubUV;

    d.nHeight_B[0] = d.bleh->nBlkY * (d.nBlkSizeY[0] - d.nOverlapY[0]) + d.nOverlapY[0];
    d.nHeight_B[1] = d.nHeight_B[2] = d.nHeight_B[0] >> d.ySubUV;


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
