// Make a motion compensate temporal denoiser
// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick (YUY2, overlap, edges processing)
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

#include "CopyCode.h"
#include "Fakery.h"
#include "Overlap.h"
#include "MaskFun.h"
#include "MVAnalysisData.h"


typedef struct MVCompensateData {
    VSNodeRef *node;
    const VSVideoInfo *vi;
    const VSVideoInfo *supervi;

    VSNodeRef *super;
    VSNodeRef *vectors;

    int scBehavior;
    int thSAD;
    int fields;
    int nSCD1;
    int nSCD2;
    int isse;
    int tff;
    int tffexists;

    MVAnalysisData vectors_data;

    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;

    int dstTempPitch;
    int dstTempPitchUV;

    OverlapWindows *OverWins;
    OverlapWindows *OverWinsUV;

    OverlapsFunction OVERSLUMA;
    OverlapsFunction OVERSCHROMA;
    COPYFunction BLITLUMA;
    COPYFunction BLITCHROMA;
    ToPixelsFunction ToPixels;
} MVCompensateData;


static void VS_CC mvcompensateInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVCompensateData *d = (MVCompensateData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvcompensateGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVCompensateData *d = (MVCompensateData *)*instanceData;

    if (activationReason == arInitial) {
        // XXX off could be calculated during initialisation
        int off, nref;
        if (d->vectors_data.nDeltaFrame > 0) {
            off = d->vectors_data.isBackward ? 1 : -1;
            off *= d->vectors_data.nDeltaFrame;
            nref = n + off;
        } else {
            nref = -d->vectors_data.nDeltaFrame; // positive frame number (special static mode)
        }

        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        if (nref < n && nref >= 0)
            vsapi->requestFrameFilter(nref, d->super, frameCtx);

        vsapi->requestFrameFilter(n, d->super, frameCtx);

        if (nref >= n && nref < d->vi->numFrames)
            vsapi->requestFrameFilter(nref, d->super, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->super, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);


        uint8_t *pDst[3] = { NULL };
        uint8_t *pDstCur[3] = { NULL };
        const uint8_t *pRef[3] = { NULL };
        int nDstPitches[3] = { 0 };
        int nRefPitches[3] = { 0 };
        const uint8_t *pSrc[3] = { NULL };
        const uint8_t *pSrcCur[3] = { NULL };
        int nSrcPitches[3] = { 0 };
        uint8_t *pDstTemp;
        uint8_t *pDstTempU;
        uint8_t *pDstTempV;
        int blx, bly;

        const VSFrameRef *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        FakeGroupOfPlanes fgop;
        fgopInit(&fgop, &d->vectors_data);
        const VSMap *mvprops = vsapi->getFramePropsRO(mvn);
        fgopUpdate(&fgop, (const int *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
        vsapi->freeFrame(mvn);

        int off, nref;
        if (d->vectors_data.nDeltaFrame > 0) {
            off = (d->vectors_data.isBackward) ? 1 : -1;
            off *= d->vectors_data.nDeltaFrame;
            nref = n + off;
        } else {
            nref = -d->vectors_data.nDeltaFrame; // positive frame number (special static mode)
        }


        const int nWidth = d->vectors_data.nWidth;
        const int nHeight = d->vectors_data.nHeight;
        const int xRatioUV = d->vectors_data.xRatioUV;
        const int yRatioUV = d->vectors_data.yRatioUV;
        const int nOverlapX = d->vectors_data.nOverlapX;
        const int nOverlapY = d->vectors_data.nOverlapY;
        const int nBlkSizeX = d->vectors_data.nBlkSizeX;
        const int nBlkSizeY = d->vectors_data.nBlkSizeY;
        const int nBlkX = d->vectors_data.nBlkX;
        const int nBlkY = d->vectors_data.nBlkY;
        const int isse = d->isse;
        const int thSAD = d->thSAD;
        const int dstTempPitch = d->dstTempPitch;
        const int dstTempPitchUV = d->dstTempPitchUV;
        const int nSuperModeYUV = d->nSuperModeYUV;
        const int nPel = d->vectors_data.nPel;
        const int nHPadding = d->vectors_data.nHPadding;
        const int nVPadding = d->vectors_data.nVPadding;
        const int scBehavior = d->scBehavior;
        const int fields = d->fields;

        int bitsPerSample = d->supervi->format->bitsPerSample;
        int bytesPerSample = d->supervi->format->bytesPerSample;


        int nWidth_B = nBlkX * (nBlkSizeX - nOverlapX) + nOverlapX;
        int nHeight_B = nBlkY * (nBlkSizeY - nOverlapY) + nOverlapY;

        int ySubUV = (yRatioUV == 2) ? 1 : 0;
        int xSubUV = (xRatioUV == 2) ? 1 : 0;

        if (fgopIsUsable(&fgop, d->nSCD1, d->nSCD2)) {
            // No need to check nref because nref is always in range when balls is usable.
            const VSFrameRef *ref = vsapi->getFrameFilter(nref, d->super, frameCtx);
            for (int i = 0; i < d->supervi->format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
            }

            MVGroupOfFrames pRefGOF, pSrcGOF;

            mvgofInit(&pRefGOF, d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, nSuperModeYUV, isse, xRatioUV, yRatioUV, bitsPerSample);
            mvgofInit(&pSrcGOF, d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, nSuperModeYUV, isse, xRatioUV, yRatioUV, bitsPerSample);

            mvgofUpdate(&pRefGOF, (uint8_t **)pRef, nRefPitches);
            mvgofUpdate(&pSrcGOF, (uint8_t **)pSrc, nSrcPitches);


            MVPlane **pPlanes = pRefGOF.frames[0]->planes;
            MVPlane **pSrcPlanes = pSrcGOF.frames[0]->planes;

            for (int plane = 0; plane < d->supervi->format->numPlanes; plane++) {
                pDstCur[plane] = pDst[plane];
                pSrcCur[plane] = pSrc[plane];
            }

            int fieldShift = 0;
            if (fields && nPel > 1 && ((nref - n) % 2 != 0)) {
                int err;
                const VSMap *props = vsapi->getFramePropsRO(src);
                int paritySrc = !!vsapi->propGetInt(props, "_Field", 0, &err); //child->GetParity(n);
                if (err && !d->tffexists) {
                    vsapi->setFilterError("Compensate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    fgopDeinit(&fgop);
                    mvgofDeinit(&pRefGOF);
                    mvgofDeinit(&pSrcGOF);
                    vsapi->freeFrame(src);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }

                props = vsapi->getFramePropsRO(ref);
                int parityRef = !!vsapi->propGetInt(props, "_Field", 0, &err); //child->GetParity(nref);
                if (err && !d->tffexists) {
                    vsapi->setFilterError("Compensate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    fgopDeinit(&fgop);
                    mvgofDeinit(&pRefGOF);
                    mvgofDeinit(&pSrcGOF);
                    vsapi->freeFrame(src);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }
                fieldShift = (paritySrc && !parityRef) ? nPel / 2 : ((parityRef && !paritySrc) ? -(nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }
            // -----------------------------------------------------------------------------
            if (nOverlapX == 0 && nOverlapY == 0) {
                for (int by = 0; by < nBlkY; by++) {
                    int xx = 0;
                    for (int bx = 0; bx < nBlkX; bx++) {
                        int i = by * nBlkX + bx;
                        const FakeBlockData *block = fgopGetBlock(&fgop, 0, i);
                        blx = block->x * nPel + block->vector.x;
                        bly = block->y * nPel + block->vector.y + fieldShift;
                        if (block->vector.sad < thSAD) {
                            // luma
                            d->BLITLUMA(pDstCur[0] + xx, nDstPitches[0], mvpGetPointer(pPlanes[0], blx, bly), pPlanes[0]->nPitch);
                            // chroma u
                            if (pPlanes[1])
                                d->BLITCHROMA(pDstCur[1] + (xx >> xSubUV), nDstPitches[1], mvpGetPointer(pPlanes[1], blx >> xSubUV, bly >> ySubUV), pPlanes[1]->nPitch);
                            // chroma v
                            if (pPlanes[2])
                                d->BLITCHROMA(pDstCur[2] + (xx >> xSubUV), nDstPitches[2], mvpGetPointer(pPlanes[2], blx >> xSubUV, bly >> ySubUV), pPlanes[2]->nPitch);
                        } else {
                            int blxsrc = bx * (nBlkSizeX)*nPel;
                            int blysrc = by * (nBlkSizeY)*nPel + fieldShift;

                            d->BLITLUMA(pDstCur[0] + xx, nDstPitches[0], mvpGetPointer(pSrcPlanes[0], blxsrc, blysrc), pSrcPlanes[0]->nPitch);
                            // chroma u
                            if (pSrcPlanes[1])
                                d->BLITCHROMA(pDstCur[1] + (xx >> xSubUV), nDstPitches[1], mvpGetPointer(pSrcPlanes[1], blxsrc >> xSubUV, blysrc >> ySubUV), pSrcPlanes[1]->nPitch);
                            // chroma v
                            if (pSrcPlanes[2])
                                d->BLITCHROMA(pDstCur[2] + (xx >> xSubUV), nDstPitches[2], mvpGetPointer(pSrcPlanes[2], blxsrc >> xSubUV, blysrc >> ySubUV), pSrcPlanes[2]->nPitch);
                        }


                        xx += (nBlkSizeX * bytesPerSample);
                    }
                    pDstCur[0] += (nBlkSizeY) * (nDstPitches[0]);
                    pSrcCur[0] += (nBlkSizeY) * (nSrcPitches[0]);
                    if (nSuperModeYUV & UVPLANES) {
                        pDstCur[1] += (nBlkSizeY >> ySubUV) * (nDstPitches[1]);
                        pDstCur[2] += (nBlkSizeY >> ySubUV) * (nDstPitches[2]);
                        pSrcCur[1] += (nBlkSizeY >> ySubUV) * (nSrcPitches[1]);
                        pSrcCur[2] += (nBlkSizeY >> ySubUV) * (nSrcPitches[2]);
                    }
                }
            }
            // -----------------------------------------------------------------
            else // overlap
            {
                OverlapWindows *OverWins = d->OverWins;
                OverlapWindows *OverWinsUV = d->OverWinsUV;
                uint8_t *DstTemp = (uint8_t *)malloc(dstTempPitch * nHeight);
                uint8_t *DstTempU = NULL;
                uint8_t *DstTempV = NULL;
                if (nSuperModeYUV & UVPLANES) {
                    DstTempU = (uint8_t *)malloc(dstTempPitchUV * nHeight);
                    DstTempV = (uint8_t *)malloc(dstTempPitchUV * nHeight);
                }

                pDstTemp = DstTemp;
                pDstTempU = DstTempU;
                pDstTempV = DstTempV;
                memset(DstTemp, 0, nHeight_B * dstTempPitch);
                if (pPlanes[1])
                    memset(DstTempU, 0, (nHeight_B >> ySubUV) * dstTempPitchUV);
                if (pPlanes[2])
                    memset(DstTempV, 0, (nHeight_B >> ySubUV) * dstTempPitchUV);

                for (int by = 0; by < nBlkY; by++) {
                    int wby = ((by + nBlkY - 3) / (nBlkY - 2)) * 3;
                    int xx = 0;
                    for (int bx = 0; bx < nBlkX; bx++) {
                        // select window
                        int wbx = (bx + nBlkX - 3) / (nBlkX - 2);
                        int16_t *winOver = overGetWindow(OverWins, wby + wbx);
                        int16_t *winOverUV = NULL;
                        if (nSuperModeYUV & UVPLANES)
                            winOverUV = overGetWindow(OverWinsUV, wby + wbx);

                        int i = by * nBlkX + bx;
                        const FakeBlockData *block = fgopGetBlock(&fgop, 0, i);

                        blx = block->y * nPel + block->vector.x;
                        bly = block->y * nPel + block->vector.y + fieldShift;

                        if (block->vector.sad < thSAD) {
                            // luma
                            d->OVERSLUMA(pDstTemp + xx * 2, dstTempPitch, mvpGetPointer(pPlanes[0], blx, bly), pPlanes[0]->nPitch, winOver, nBlkSizeX);
                            // chroma u
                            if (pPlanes[1])
                                d->OVERSCHROMA(pDstTempU + (xx >> xSubUV) * 2, dstTempPitchUV, mvpGetPointer(pPlanes[1], blx >> xSubUV, bly >> ySubUV), pPlanes[1]->nPitch, winOverUV, nBlkSizeX >> xSubUV);
                            // chroma v
                            if (pPlanes[2])
                                d->OVERSCHROMA(pDstTempV + (xx >> xSubUV) * 2, dstTempPitchUV, mvpGetPointer(pPlanes[2], blx >> xSubUV, bly >> ySubUV), pPlanes[2]->nPitch, winOverUV, nBlkSizeX >> xSubUV);
                        } else { // bad compensation, use src
                            int blxsrc = bx * (nBlkSizeX - nOverlapX) * nPel;
                            int blysrc = by * (nBlkSizeY - nOverlapY) * nPel + fieldShift;

                            d->OVERSLUMA(pDstTemp + xx * 2, dstTempPitch, mvpGetPointer(pSrcPlanes[0], blxsrc, blysrc), pSrcPlanes[0]->nPitch, winOver, nBlkSizeX);
                            // chroma u
                            if (pSrcPlanes[1])
                                d->OVERSCHROMA(pDstTempU + (xx >> xSubUV) * 2, dstTempPitchUV, mvpGetPointer(pSrcPlanes[1], blxsrc >> xSubUV, blysrc >> ySubUV), pSrcPlanes[1]->nPitch, winOverUV, nBlkSizeX >> xSubUV);
                            // chroma v
                            if (pSrcPlanes[2])
                                d->OVERSCHROMA(pDstTempV + (xx >> xSubUV) * 2, dstTempPitchUV, mvpGetPointer(pSrcPlanes[2], blxsrc >> xSubUV, blysrc >> ySubUV), pSrcPlanes[2]->nPitch, winOverUV, nBlkSizeX >> xSubUV);
                        }

                        xx += (nBlkSizeX - nOverlapX) * bytesPerSample;
                    }

                    pDstTemp += dstTempPitch * (nBlkSizeY - nOverlapY);
                    pDstCur[0] += (nBlkSizeY - nOverlapY) * (nDstPitches[0]);
                    pSrcCur[0] += (nBlkSizeY - nOverlapY) * (nSrcPitches[0]);
                    if (nSuperModeYUV & UVPLANES) {
                        pDstTempU += dstTempPitchUV * ((nBlkSizeY - nOverlapY) >> ySubUV);
                        pDstTempV += dstTempPitchUV * ((nBlkSizeY - nOverlapY) >> ySubUV);
                        pDstCur[1] += ((nBlkSizeY - nOverlapY) >> ySubUV) * (nDstPitches[1]);
                        pDstCur[2] += ((nBlkSizeY - nOverlapY) >> ySubUV) * (nDstPitches[2]);
                        pSrcCur[1] += ((nBlkSizeY - nOverlapY) >> ySubUV) * (nSrcPitches[1]);
                        pSrcCur[2] += ((nBlkSizeY - nOverlapY) >> ySubUV) * (nSrcPitches[2]);
                    }
                }

                d->ToPixels(pDst[0], nDstPitches[0], DstTemp, dstTempPitch, nWidth_B, nHeight_B, bitsPerSample);
                if (pPlanes[1])
                    d->ToPixels(pDst[1], nDstPitches[1], DstTempU, dstTempPitchUV, nWidth_B >> xSubUV, nHeight_B >> ySubUV, bitsPerSample);
                if (pPlanes[2])
                    d->ToPixels(pDst[2], nDstPitches[2], DstTempV, dstTempPitchUV, nWidth_B >> xSubUV, nHeight_B >> ySubUV, bitsPerSample);

                free(DstTemp);
                if (nSuperModeYUV & UVPLANES) {
                    free(DstTempU);
                    free(DstTempV);
                }
            }

            const uint8_t *scSrc[3] = { 0 };
            int scPitches[3] = { 0 };

            for (int i = 0; i < 3; i++) {
                if (scBehavior) {
                    scSrc[i] = pSrc[i];
                    scPitches[i] = nSrcPitches[i];
                } else {
                    scSrc[i] = pRef[i];
                    scPitches[i] = nRefPitches[i];
                }
            }

            if (nWidth_B < nWidth) // padding of right non-covered region
            {
                // luma
                vs_bitblt(pDst[0] + nWidth_B * bytesPerSample, nDstPitches[0],
                          scSrc[0] + (nWidth_B + nHPadding) * bytesPerSample + nVPadding * scPitches[0], scPitches[0],
                          (nWidth - nWidth_B) * bytesPerSample, nHeight_B);
                // chroma u
                if (pPlanes[1])
                    vs_bitblt(pDst[1] + (nWidth_B >> xSubUV) * bytesPerSample, nDstPitches[1],
                              scSrc[1] + ((nWidth_B >> xSubUV) + (nHPadding >> xSubUV)) * bytesPerSample + (nVPadding >> ySubUV) * scPitches[1], scPitches[1],
                              ((nWidth - nWidth_B) >> xSubUV) * bytesPerSample, nHeight_B >> ySubUV);
                // chroma v
                if (pPlanes[2])
                    vs_bitblt(pDst[2] + (nWidth_B >> xSubUV) * bytesPerSample, nDstPitches[2],
                              scSrc[2] + ((nWidth_B >> xSubUV) + (nHPadding >> xSubUV)) * bytesPerSample + (nVPadding >> ySubUV) * scPitches[2], scPitches[2],
                              ((nWidth - nWidth_B) >> xSubUV) * bytesPerSample, nHeight_B >> ySubUV);
            }

            if (nHeight_B < nHeight) // padding of bottom non-covered region
            {
                // luma
                vs_bitblt(pDst[0] + nHeight_B * nDstPitches[0], nDstPitches[0],
                          scSrc[0] + nHPadding * bytesPerSample + (nHeight_B + nVPadding) * scPitches[0], scPitches[0],
                          nWidth * bytesPerSample, nHeight - nHeight_B);
                // chroma u
                if (pPlanes[1])
                    vs_bitblt(pDst[1] + (nHeight_B >> ySubUV) * nDstPitches[1], nDstPitches[1],
                              scSrc[1] + nHPadding * bytesPerSample + ((nHeight_B + nVPadding) >> ySubUV) * scPitches[1], scPitches[1],
                              (nWidth >> xSubUV) * bytesPerSample, (nHeight - nHeight_B) >> ySubUV);
                // chroma v
                if (pPlanes[2])
                    vs_bitblt(pDst[2] + (nHeight_B >> ySubUV) * nDstPitches[2], nDstPitches[2],
                              scSrc[2] + nHPadding * bytesPerSample + ((nHeight_B + nVPadding) >> ySubUV) * scPitches[2], scPitches[2],
                              (nWidth >> xSubUV) * bytesPerSample, (nHeight - nHeight_B) >> ySubUV);
            }

            mvgofDeinit(&pRefGOF);
            mvgofDeinit(&pSrcGOF);

            vsapi->freeFrame(ref);
        } else { // balls.IsUsable()
            if (!scBehavior && nref < d->vi->numFrames && nref >= 0) {
                vsapi->freeFrame(src);
                src = vsapi->getFrameFilter(nref, d->super, frameCtx);
            }

            for (int i = 0; i < d->supervi->format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
            }

            int nOffset[3];

            nOffset[0] = nHPadding * bytesPerSample + nVPadding * nSrcPitches[0];
            nOffset[1] = nHPadding * bytesPerSample / xRatioUV + (nVPadding / yRatioUV) * nSrcPitches[1];
            nOffset[2] = nOffset[1];

            vs_bitblt(pDst[0], nDstPitches[0], pSrc[0] + nOffset[0], nSrcPitches[0], nWidth * bytesPerSample, nHeight);
            if (nSuperModeYUV & UVPLANES) {
                vs_bitblt(pDst[1], nDstPitches[1], pSrc[1] + nOffset[1], nSrcPitches[1], nWidth * bytesPerSample / xRatioUV, nHeight / yRatioUV);
                vs_bitblt(pDst[2], nDstPitches[2], pSrc[2] + nOffset[2], nSrcPitches[2], nWidth * bytesPerSample / xRatioUV, nHeight / yRatioUV);
            }
        }

        fgopDeinit(&fgop);

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvcompensateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVCompensateData *d = (MVCompensateData *)instanceData;

    if (d->vectors_data.nOverlapX || d->vectors_data.nOverlapY) {
        overDeinit(d->OverWins);
        free(d->OverWins);
        if (d->nSuperModeYUV & UVPLANES) {
            overDeinit(d->OverWinsUV);
            free(d->OverWinsUV);
        }
    }

    vsapi->freeNode(d->super);
    vsapi->freeNode(d->vectors);
    vsapi->freeNode(d->node);
    free(d);
}


static void selectFunctions(MVCompensateData *d) {
    const int xRatioUV = d->vectors_data.xRatioUV;
    const int yRatioUV = d->vectors_data.yRatioUV;
    const int nBlkSizeX = d->vectors_data.nBlkSizeX;
    const int nBlkSizeY = d->vectors_data.nBlkSizeY;

    OverlapsFunction overs[33][33];
    COPYFunction copys[33][33];

    if (d->vi->format->bitsPerSample == 8) {
        overs[2][2] = mvtools_overlaps_2x2_uint16_t_uint8_t_c;
        copys[2][2] = mvtools_copy_2x2_u8_c;

        overs[2][4] = mvtools_overlaps_2x4_uint16_t_uint8_t_c;
        copys[2][4] = mvtools_copy_2x4_u8_c;

        overs[4][2] = d->isse ? mvtools_overlaps_4x2_sse2 : mvtools_overlaps_4x2_uint16_t_uint8_t_c;
        copys[4][2] = mvtools_copy_4x2_u8_c;

        overs[4][4] = d->isse ? mvtools_overlaps_4x4_sse2 : mvtools_overlaps_4x4_uint16_t_uint8_t_c;
        copys[4][4] = mvtools_copy_4x4_u8_c;

        overs[4][8] = d->isse ? mvtools_overlaps_4x8_sse2 : mvtools_overlaps_4x8_uint16_t_uint8_t_c;
        copys[4][8] = mvtools_copy_4x8_u8_c;

        overs[8][1] = d->isse ? mvtools_overlaps_8x1_sse2 : mvtools_overlaps_8x1_uint16_t_uint8_t_c;
        copys[8][1] = mvtools_copy_8x1_u8_c;

        overs[8][2] = d->isse ? mvtools_overlaps_8x2_sse2 : mvtools_overlaps_8x2_uint16_t_uint8_t_c;
        copys[8][2] = mvtools_copy_8x2_u8_c;

        overs[8][4] = d->isse ? mvtools_overlaps_8x4_sse2 : mvtools_overlaps_8x4_uint16_t_uint8_t_c;
        copys[8][4] = mvtools_copy_8x4_u8_c;

        overs[8][8] = d->isse ? mvtools_overlaps_8x8_sse2 : mvtools_overlaps_8x8_uint16_t_uint8_t_c;
        copys[8][8] = mvtools_copy_8x8_u8_c;

        overs[8][16] = d->isse ? mvtools_overlaps_8x16_sse2 : mvtools_overlaps_8x16_uint16_t_uint8_t_c;
        copys[8][16] = mvtools_copy_8x16_u8_c;

        overs[16][1] = d->isse ? mvtools_overlaps_16x1_sse2 : mvtools_overlaps_16x1_uint16_t_uint8_t_c;
        copys[16][1] = mvtools_copy_16x1_u8_c;

        overs[16][2] = d->isse ? mvtools_overlaps_16x2_sse2 : mvtools_overlaps_16x2_uint16_t_uint8_t_c;
        copys[16][2] = mvtools_copy_16x2_u8_c;

        overs[16][4] = d->isse ? mvtools_overlaps_16x4_sse2 : mvtools_overlaps_16x4_uint16_t_uint8_t_c;
        copys[16][4] = mvtools_copy_16x4_u8_c;

        overs[16][8] = d->isse ? mvtools_overlaps_16x8_sse2 : mvtools_overlaps_16x8_uint16_t_uint8_t_c;
        copys[16][8] = mvtools_copy_16x8_u8_c;

        overs[16][16] = d->isse ? mvtools_overlaps_16x16_sse2 : mvtools_overlaps_16x16_uint16_t_uint8_t_c;
        copys[16][16] = mvtools_copy_16x16_u8_c;

        overs[16][32] = d->isse ? mvtools_overlaps_16x32_sse2 : mvtools_overlaps_16x32_uint16_t_uint8_t_c;
        copys[16][32] = mvtools_copy_16x32_u8_c;

        overs[32][8] = d->isse ? mvtools_overlaps_32x8_sse2 : mvtools_overlaps_32x8_uint16_t_uint8_t_c;
        copys[32][8] = mvtools_copy_32x8_u8_c;

        overs[32][16] = d->isse ? mvtools_overlaps_32x16_sse2 : mvtools_overlaps_32x16_uint16_t_uint8_t_c;
        copys[32][16] = mvtools_copy_32x16_u8_c;

        overs[32][32] = d->isse ? mvtools_overlaps_32x32_sse2 : mvtools_overlaps_32x32_uint16_t_uint8_t_c;
        copys[32][32] = mvtools_copy_32x32_u8_c;

        d->ToPixels = ToPixels_uint16_t_uint8_t;
    } else {
        overs[2][2] = mvtools_overlaps_2x2_uint32_t_uint16_t_c;
        copys[2][2] = mvtools_copy_2x2_u16_c;

        overs[2][4] = mvtools_overlaps_2x4_uint32_t_uint16_t_c;
        copys[2][4] = mvtools_copy_2x4_u16_c;

        overs[4][2] = mvtools_overlaps_4x2_uint32_t_uint16_t_c;
        copys[4][2] = mvtools_copy_4x2_u16_c;

        overs[4][4] = mvtools_overlaps_4x4_uint32_t_uint16_t_c;
        copys[4][4] = mvtools_copy_4x4_u16_c;

        overs[4][8] = mvtools_overlaps_4x8_uint32_t_uint16_t_c;
        copys[4][8] = mvtools_copy_4x8_u16_c;

        overs[8][1] = mvtools_overlaps_8x1_uint32_t_uint16_t_c;
        copys[8][1] = mvtools_copy_8x1_u16_c;

        overs[8][2] = mvtools_overlaps_8x2_uint32_t_uint16_t_c;
        copys[8][2] = mvtools_copy_8x2_u16_c;

        overs[8][4] = mvtools_overlaps_8x4_uint32_t_uint16_t_c;
        copys[8][4] = mvtools_copy_8x4_u16_c;

        overs[8][8] = mvtools_overlaps_8x8_uint32_t_uint16_t_c;
        copys[8][8] = mvtools_copy_8x8_u16_c;

        overs[8][16] = mvtools_overlaps_8x16_uint32_t_uint16_t_c;
        copys[8][16] = mvtools_copy_8x16_u16_c;

        overs[16][1] = mvtools_overlaps_16x1_uint32_t_uint16_t_c;
        copys[16][1] = mvtools_copy_16x1_u16_c;

        overs[16][2] = mvtools_overlaps_16x2_uint32_t_uint16_t_c;
        copys[16][2] = mvtools_copy_16x2_u16_c;

        overs[16][4] = mvtools_overlaps_16x4_uint32_t_uint16_t_c;
        copys[16][4] = mvtools_copy_16x4_u16_c;

        overs[16][8] = mvtools_overlaps_16x8_uint32_t_uint16_t_c;
        copys[16][8] = mvtools_copy_16x8_u16_c;

        overs[16][16] = mvtools_overlaps_16x16_uint32_t_uint16_t_c;
        copys[16][16] = mvtools_copy_16x16_u16_c;

        overs[16][32] = mvtools_overlaps_16x32_uint32_t_uint16_t_c;
        copys[16][32] = mvtools_copy_16x32_u16_c;

        overs[32][8] = mvtools_overlaps_32x8_uint32_t_uint16_t_c;
        copys[32][8] = mvtools_copy_32x8_u16_c;

        overs[32][16] = mvtools_overlaps_32x16_uint32_t_uint16_t_c;
        copys[32][16] = mvtools_copy_32x16_u16_c;

        overs[32][32] = mvtools_overlaps_32x32_uint32_t_uint16_t_c;
        copys[32][32] = mvtools_copy_32x32_u16_c;

        d->ToPixels = ToPixels_uint32_t_uint16_t;
    }

    d->OVERSLUMA = overs[nBlkSizeX][nBlkSizeY];
    d->BLITLUMA = copys[nBlkSizeX][nBlkSizeY];

    d->OVERSCHROMA = overs[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
    d->BLITCHROMA = copys[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
}


static void VS_CC mvcompensateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVCompensateData d;
    MVCompensateData *data;

    int err;

    d.scBehavior = !!vsapi->propGetInt(in, "scbehavior", 0, &err);
    if (err)
        d.scBehavior = 1;

    d.thSAD = int64ToIntS(vsapi->propGetInt(in, "thsad", 0, &err));
    if (err)
        d.thSAD = 10000;

    d.fields = !!vsapi->propGetInt(in, "fields", 0, &err);

    d.nSCD1 = int64ToIntS(vsapi->propGetInt(in, "thscd1", 0, &err));
    if (err)
        d.nSCD1 = MV_DEFAULT_SCD1;

    d.nSCD2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.nSCD2 = MV_DEFAULT_SCD2;

    d.isse = !!vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;

    d.tff = !!vsapi->propGetInt(in, "tff", 0, &err);
    d.tffexists = err;


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Compensate: failed to retrieve first frame from super clip. Error message: ";
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
            vsapi->setError(out, "Compensate: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.vectors = vsapi->propGetNode(in, "vectors", 0, NULL);

#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };
    const char *filter_name = "Compensate";

    adataFromVectorClip(&d.vectors_data, d.vectors, filter_name, "vectors", vsapi, error, ERROR_SIZE);

    int nSCD1_old = d.nSCD1;
    scaleThSCD(&d.nSCD1, &d.nSCD2, &d.vectors_data, filter_name, error, ERROR_SIZE);
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->setError(out, error);

        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        return;
    }


    if (d.fields && d.vectors_data.nPel < 2) {
        vsapi->setError(out, "Compensate: fields option requires pel > 1.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        return;
    }

    d.thSAD = (int64_t)d.thSAD * d.nSCD1 / nSCD1_old; // normalize to block SAD


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);


    d.dstTempPitch = ((d.vectors_data.nWidth + 15) / 16) * 16 * d.vi->format->bytesPerSample * 2;
    d.dstTempPitchUV = (((d.vectors_data.nWidth / d.vectors_data.xRatioUV) + 15) / 16) * 16 * d.vi->format->bytesPerSample * 2;


    d.supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = d.supervi->width;

    if (d.vectors_data.nHeight != nHeightS || d.vectors_data.nHeight != d.vi->height || d.vectors_data.nWidth != nSuperWidth - d.nSuperHPad * 2 || d.vectors_data.nWidth != d.vi->width) {
        vsapi->setError(out, "Compensate: wrong source or super clip frame size.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.node);
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, "Compensate: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.node);
        return;
    }

    if (d.vi->format->bitsPerSample > 8)
        d.isse = 0;

    if (d.vectors_data.nOverlapX || d.vectors_data.nOverlapY) {
        d.OverWins = (OverlapWindows *)malloc(sizeof(OverlapWindows));
        overInit(d.OverWins, d.vectors_data.nBlkSizeX, d.vectors_data.nBlkSizeY, d.vectors_data.nOverlapX, d.vectors_data.nOverlapY);
        if (d.nSuperModeYUV & UVPLANES) {
            d.OverWins = (OverlapWindows *)malloc(sizeof(OverlapWindows));
            overInit(d.OverWinsUV, d.vectors_data.nBlkSizeX / d.vectors_data.xRatioUV, d.vectors_data.nBlkSizeY / d.vectors_data.yRatioUV, d.vectors_data.nOverlapX / d.vectors_data.xRatioUV, d.vectors_data.nOverlapY / d.vectors_data.yRatioUV);
        }
    }

    selectFunctions(&d);


    data = (MVCompensateData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Compensate", mvcompensateInit, mvcompensateGetFrame, mvcompensateFree, fmParallel, 0, data, core);
}


void mvcompensateRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Compensate",
                 "clip:clip;"
                 "super:clip;"
                 "vectors:clip;"
                 "scbehavior:int:opt;"
                 "thsad:int:opt;"
                 "fields:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "isse:int:opt;"
                 "tff:int:opt;",
                 mvcompensateCreate, 0, plugin);
}
