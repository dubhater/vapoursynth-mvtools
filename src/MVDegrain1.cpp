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

    int thSAD[3];
    int YUVplanes;
    int nLimit[3];
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

    OverlapsFunction OVERS[3];
    Denoise1Function DEGRAIN[3];

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
} MVDegrain1Data;


static void VS_CC mvdegrain1Init(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVDegrain1Data *d = (MVDegrain1Data *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static inline void normaliseWeights(int &WSrc, int &WRefB, int &WRefF) {
    WSrc = 256;
    int WSum = WRefB + WRefF + WSrc + 1;
    WRefB = WRefB * 256 / WSum; // normalise weights to 256
    WRefF = WRefF * 256 / WSum;
    WSrc = 256 - WRefB - WRefF;
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
        unsigned short *pDstShort;
        int nLogPel = (d->bleh->nPel == 4) ? 2 : (d->bleh->nPel == 2) ? 1 : 0;
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

        for (int i = 0; i < d->vi->format->numPlanes; i++) {
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


        short *winOver;

        OverlapWindows *OverWins[3] = { NULL, NULL, NULL };
        unsigned short *DstShort = NULL;
        if (nOverlapX[0] > 0 || nOverlapY[0] > 0) {
            OverWins[0] = new OverlapWindows(nBlkSizeX[0], nBlkSizeY[0], nOverlapX[0], nOverlapY[0]);
            OverWins[1] = new OverlapWindows(nBlkSizeX[1], nBlkSizeY[1], nOverlapX[1], nOverlapY[1]);
            OverWins[2] = OverWins[1];
            DstShort = new unsigned short[dstShortPitch * nHeight[0]];
        }

        uint8_t *tmpBlock = new uint8_t[32*32];


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
                        const uint8_t *pB, *pF;
                        int npB, npF;
                        int WRefB, WRefF, WSrc;

                        useBlock(pB, npB, WRefB, isUsableB, ballsB, i, pPlanesB, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF, npF, WRefF, isUsableF, ballsF, i, pPlanesF, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights(WSrc, WRefB, WRefF);

                        d->DEGRAIN[plane](pDstCur[plane] + xx, nDstPitches[plane], pSrcCur[plane] + xx, nSrcPitches[plane],
                                pB, npB, pF, npF, WSrc, WRefB, WRefF);

                        xx += nBlkSizeX[plane];

                        if (bx == nBlkX - 1 && nWidth_B[0] < nWidth[0]) {// right non-covered region
                            vs_bitblt(pDstCur[plane] + nWidth_B[plane], nDstPitches[plane],
                                      pSrcCur[plane] + nWidth_B[plane], nSrcPitches[plane],
                                      nWidth[plane] - nWidth_B[plane], nBlkSizeY[plane]);
                        }
                    }
                    pDstCur[plane] += nBlkSizeY[plane] * nDstPitches[plane];
                    pSrcCur[plane] += nBlkSizeY[plane] * nSrcPitches[plane];

                    if (by == nBlkY - 1 && nHeight_B[0] < nHeight[0]) {// bottom uncovered region
                        vs_bitblt(pDstCur[plane], nDstPitches[plane],
                                  pSrcCur[plane], nSrcPitches[plane],
                                  nWidth[plane], nHeight[plane] - nHeight_B[plane]);
                    }
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
                        const uint8_t *pB, *pF;
                        int npB, npF;
                        int WRefB, WRefF, WSrc;

                        useBlock(pB, npB, WRefB, isUsableB, ballsB, i, pPlanesB, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);
                        useBlock(pF, npF, WRefF, isUsableF, ballsF, i, pPlanesF, pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights(WSrc, WRefB, WRefF);

                        d->DEGRAIN[plane](tmpBlock, tmpPitch, pSrcCur[plane] + xx, nSrcPitches[plane],
                                          pB, npB, pF, npF, WSrc, WRefB, WRefF);
                        d->OVERS[plane](pDstShort + xx, dstShortPitch, tmpBlock, tmpPitch, winOver, nBlkSizeX[plane]);

                        xx += (nBlkSizeX[plane] - nOverlapX[plane]);
                    }
                    pSrcCur[plane] += ((nBlkSizeY[plane] - nOverlapY[plane]) * nSrcPitches[plane]);
                    pDstShort += (nBlkSizeY[plane] - nOverlapY[plane]) * dstShortPitch;
                }

                Short2Bytes(pDst[plane], nDstPitches[plane], DstShort, dstShortPitch, nWidth_B[plane], nHeight_B[plane]);

                if (nWidth_B[0] < nWidth[0])
                    vs_bitblt(pDst[plane] + nWidth_B[plane], nDstPitches[plane],
                              pSrc[plane] + nWidth_B[plane], nSrcPitches[plane],
                              nWidth[plane] - nWidth_B[plane], nHeight_B[plane]);
                if (nHeight_B[0] < nHeight[0]) // bottom noncovered region
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
    const int xRatioUV = 2;
    const int yRatioUV = d->bleh->yRatioUV;
    const int nBlkSizeX = d->bleh->nBlkSizeX;
    const int nBlkSizeY = d->bleh->nBlkSizeY;

    OverlapsFunction overs[33][33];
    Denoise1Function degs[33][33];

    overs[2][2] = Overlaps_C<2,2>;
    degs[2][2] = Degrain1_C<2,2>;

    overs[2][4] = Overlaps_C<2,4>;
    degs[2][4] = Degrain1_C<2,4>;

    overs[4][2] = d->isse ? mvtools_Overlaps4x2_sse2 : Overlaps_C<4,2>;
    degs[4][2] = d->isse ? Degrain1_sse2<4,2> : Degrain1_C<4,2>;

    overs[4][4] = d->isse ? mvtools_Overlaps4x4_sse2 : Overlaps_C<4,4>;
    degs[4][4] = d->isse ? Degrain1_sse2<4,4> : Degrain1_C<4,4>;

    overs[4][8] = d->isse ? mvtools_Overlaps4x8_sse2 : Overlaps_C<4,8>;
    degs[4][8] = d->isse ? Degrain1_sse2<4,8> : Degrain1_C<4,8>;

    overs[8][1] = d->isse ? mvtools_Overlaps8x1_sse2 : Overlaps_C<8,1>;
    degs[8][1] = d->isse ? Degrain1_sse2<8,1> : Degrain1_C<8,1>;

    overs[8][2] = d->isse ? mvtools_Overlaps8x2_sse2 : Overlaps_C<8,2>;
    degs[8][2] = d->isse ? Degrain1_sse2<8,2> : Degrain1_C<8,2>;

    overs[8][4] = d->isse ? mvtools_Overlaps8x4_sse2 : Overlaps_C<8,4>;
    degs[8][4] = d->isse ? Degrain1_sse2<8,4> : Degrain1_C<8,4>;

    overs[8][8] = d->isse ? mvtools_Overlaps8x8_sse2 : Overlaps_C<8,8>;
    degs[8][8] = d->isse ? Degrain1_sse2<8,8> : Degrain1_C<8,8>;

    overs[8][16] = d->isse ? mvtools_Overlaps8x16_sse2 : Overlaps_C<8,16>;
    degs[8][16] = d->isse ? Degrain1_sse2<8,16> : Degrain1_C<8,16>;

    overs[16][2] = d->isse ? mvtools_Overlaps16x2_sse2 : Overlaps_C<16,2>;
    degs[16][2] = d->isse ? Degrain1_sse2<16,2> : Degrain1_C<16,2>;

    overs[16][8] = d->isse ? mvtools_Overlaps16x8_sse2 : Overlaps_C<16,8>;
    degs[16][8] = d->isse ? Degrain1_sse2<16,8> : Degrain1_C<16,8>;

    overs[16][16] = d->isse ? mvtools_Overlaps16x16_sse2 : Overlaps_C<16,16>;
    degs[16][16] = d->isse ? Degrain1_sse2<16,16> : Degrain1_C<16,16>;

    overs[16][32] = d->isse ? mvtools_Overlaps16x32_sse2 : Overlaps_C<16,32>;
    degs[16][32] = d->isse ? Degrain1_sse2<16,32> : Degrain1_C<16,32>;

    overs[32][16] = d->isse ? mvtools_Overlaps32x16_sse2 : Overlaps_C<32,16>;
    degs[32][16] = d->isse ? Degrain1_sse2<32,16> : Degrain1_C<32,16>;

    overs[32][32] = d->isse ? mvtools_Overlaps32x32_sse2 : Overlaps_C<32,32>;
    degs[32][32] = d->isse ? Degrain1_sse2<32,32> : Degrain1_C<32,32>;

    d->OVERS[0] = overs[nBlkSizeX][nBlkSizeY];
    d->DEGRAIN[0] = degs[nBlkSizeX][nBlkSizeY];

    d->OVERS[1] = d->OVERS[2] = overs[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
    d->DEGRAIN[1] = d->DEGRAIN[2] = degs[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
}


static void VS_CC mvdegrain1Create(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVDegrain1Data d;
    MVDegrain1Data *data;

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
        vsapi->setError(out, "Degrain1: plane must be between 0 and 4 (inclusive).");
        return;
    }

    if (d.nLimit[0] < 0 || d.nLimit[0] > 255) {
        vsapi->setError(out, "Degrain1: limit must be between 0 and 255 (inclusive).");
        return;
    }

    if (d.nLimit[1] < 0 || d.nLimit[1] > 255) {
        vsapi->setError(out, "Degrain1: limitc must be between 0 and 255 (inclusive).");
        return;
    }

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

    d.thSAD[0] = d.thSAD[0] * d.mvClipB->GetThSCD1() / d.nSCD1; // normalize to block SAD
    d.thSAD[1] = d.thSAD[2] = d.thSAD[1] * d.mvClipB->GetThSCD1() / d.nSCD1; // chroma threshold, normalized to block SAD



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

    int id = d.vi->format->id;
    if (!isConstantFormat(d.vi) || (id != pfYUV420P8 && id != pfYUV422P8)) {
        vsapi->setError(out, "Degrain1: input clip must be YUV420P8 or YUV422P8, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
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
