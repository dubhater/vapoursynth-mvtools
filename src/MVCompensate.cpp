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

#include "MVFilter.h"
#include "CopyCode.h"
#include "Overlap.h"
#include "MaskFun.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;
    const VSVideoInfo *supervi;

    VSNodeRef *super;
    VSNodeRef *vectors;

    bool scBehavior;
    int thSAD;
    bool fields;
    int nSCD1;
    int nSCD2;
    bool isse;
    bool tff;
    int tffexists;

    MVClipDicks *mvClip;

    MVFilter *bleh;

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
    MVCompensateData *d = (MVCompensateData *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvcompensateGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVCompensateData *d = (MVCompensateData *) * instanceData;

    if (activationReason == arInitial) {
        // XXX off could be calculated during initialisation
        int off, nref;
        if (d->mvClip->GetDeltaFrame() > 0)
        {
            off = ( d->mvClip->IsBackward() ) ? 1 : -1;
            off *= d->mvClip->GetDeltaFrame();
            nref = n + off;
        }
        else
        {
            nref = - d->mvClip->GetDeltaFrame(); // positive frame number (special static mode)
        }

        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        if (nref < n && nref >= 0)
            vsapi->requestFrameFilter(nref, d->super, frameCtx);

        vsapi->requestFrameFilter(n, d->super, frameCtx);

        if (nref >= n && (!d->vi->numFrames || nref < d->vi->numFrames))
            vsapi->requestFrameFilter(nref, d->super, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->super, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);


        uint8_t *pDst[3], *pDstCur[3];
        const uint8_t *pRef[3];
        int nDstPitches[3], nRefPitches[3];
        const uint8_t *pSrc[3], *pSrcCur[3];
        int nSrcPitches[3];
        uint8_t *pDstTemp;
        uint8_t *pDstTempU;
        uint8_t *pDstTempV;
        int blx, bly;

        const VSFrameRef *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        MVClipBalls balls(d->mvClip, vsapi);
        balls.Update(mvn);
        vsapi->freeFrame(mvn);

        int off, nref;
        if (d->mvClip->GetDeltaFrame() > 0)
        {
            off = ( d->mvClip->IsBackward() ) ? 1 : -1;
            off *= d->mvClip->GetDeltaFrame();
            nref = n + off;
        }
        else
        {
            nref = - d->mvClip->GetDeltaFrame(); // positive frame number (special static mode)
        }


        const int nWidth = d->bleh->nWidth;
        const int nHeight = d->bleh->nHeight;
        const int xRatioUV = d->bleh->xRatioUV;
        const int yRatioUV = d->bleh->yRatioUV;
        const int nOverlapX = d->bleh->nOverlapX;
        const int nOverlapY = d->bleh->nOverlapY;
        const int nBlkSizeX = d->bleh->nBlkSizeX;
        const int nBlkSizeY = d->bleh->nBlkSizeY;
        const int nBlkX = d->bleh->nBlkX;
        const int nBlkY = d->bleh->nBlkY;
        const bool isse = d->isse;
        const int thSAD = d->thSAD;
        const int dstTempPitch = d->dstTempPitch;
        const int dstTempPitchUV = d->dstTempPitchUV;
        const int nSuperModeYUV = d->nSuperModeYUV;
        const int nPel = d->bleh->nPel;
        const int nHPadding = d->bleh->nHPadding;
        const int nVPadding = d->bleh->nVPadding;
        const int scBehavior = d->scBehavior;
        const int fields = d->fields;

        int bitsPerSample = d->supervi->format->bitsPerSample;
        int bytesPerSample = d->supervi->format->bytesPerSample;


        int nWidth_B = nBlkX*(nBlkSizeX - nOverlapX) + nOverlapX;
        int nHeight_B = nBlkY*(nBlkSizeY - nOverlapY) + nOverlapY;

        int ySubUV = (yRatioUV == 2) ? 1 : 0;
        int xSubUV = (xRatioUV == 2) ? 1 : 0;

        if ( balls.IsUsable() )
        {
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

            MVGroupOfFrames *pRefGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, nSuperModeYUV, isse, xRatioUV, yRatioUV, bitsPerSample);
            MVGroupOfFrames *pSrcGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, nSuperModeYUV, isse, xRatioUV, yRatioUV, bitsPerSample);

            pRefGOF->Update(nSuperModeYUV, (uint8_t*)pRef[0], nRefPitches[0], (uint8_t*)pRef[1], nRefPitches[1], (uint8_t*)pRef[2], nRefPitches[2]);// v2.0

            pSrcGOF->Update(nSuperModeYUV, (uint8_t*)pSrc[0], nSrcPitches[0], (uint8_t*)pSrc[1], nSrcPitches[1], (uint8_t*)pSrc[2], nSrcPitches[2]);


            MVPlaneSet planes[3] = { YPLANE, UPLANE, VPLANE };

            MVPlane *pPlanes[3] = { 0 };
            MVPlane *pSrcPlanes[3] = { 0 };

            for (int plane = 0; plane < d->supervi->format->numPlanes; plane++) {
                pPlanes[plane] = pRefGOF->GetFrame(0)->GetPlane(planes[plane]);
                pSrcPlanes[plane] = pSrcGOF->GetFrame(0)->GetPlane(planes[plane]);

                pDstCur[plane] = pDst[plane];
                pSrcCur[plane] = pSrc[plane];
            }

            int fieldShift = 0;
            if (fields && nPel > 1 && ((nref-n) %2 != 0))
            {
                int err;
                const VSMap *props = vsapi->getFramePropsRO(src);
                bool paritySrc = !!vsapi->propGetInt(props, "_Field", 0, &err); //child->GetParity(n);
                if (err && !d->tffexists) {
                    vsapi->setFilterError("Compensate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    delete pRefGOF;
                    delete pSrcGOF;
                    vsapi->freeFrame(src);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }

                props = vsapi->getFramePropsRO(ref);
                bool parityRef = !!vsapi->propGetInt(props, "_Field", 0, &err); //child->GetParity(nref);
                if (err && !d->tffexists) {
                    vsapi->setFilterError("Compensate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    delete pRefGOF;
                    delete pSrcGOF;
                    vsapi->freeFrame(src);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }
                fieldShift = (paritySrc && !parityRef) ? nPel/2 : ( (parityRef && !paritySrc) ? -(nPel/2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }
            // -----------------------------------------------------------------------------
            if (nOverlapX==0 && nOverlapY==0)
            {
                for (int by=0; by<nBlkY; by++)
                {
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        int i = by*nBlkX + bx;
                        const FakeBlockData &block = balls.GetBlock(0, i);
                        blx = block.GetX() * nPel + block.GetMV().x;
                        bly = block.GetY() * nPel + block.GetMV().y + fieldShift;
                        if (block.GetSAD() < thSAD)
                        {
                            // luma
                            d->BLITLUMA(pDstCur[0] + xx, nDstPitches[0], pPlanes[0]->GetPointer(blx, bly), pPlanes[0]->GetPitch());
                            // chroma u
                            if(pPlanes[1]) d->BLITCHROMA(pDstCur[1] + (xx >> xSubUV), nDstPitches[1], pPlanes[1]->GetPointer(blx >> xSubUV, bly>>ySubUV), pPlanes[1]->GetPitch());
                            // chroma v
                            if(pPlanes[2]) d->BLITCHROMA(pDstCur[2] + (xx >> xSubUV), nDstPitches[2], pPlanes[2]->GetPointer(blx >> xSubUV, bly>>ySubUV), pPlanes[2]->GetPitch());
                        }
                        else
                        {
                            int blxsrc = bx * (nBlkSizeX) * nPel;
                            int blysrc = by * (nBlkSizeY) * nPel  + fieldShift;

                            d->BLITLUMA(pDstCur[0] + xx, nDstPitches[0], pSrcPlanes[0]->GetPointer(blxsrc, blysrc), pSrcPlanes[0]->GetPitch());
                            // chroma u
                            if(pSrcPlanes[1]) d->BLITCHROMA(pDstCur[1] + (xx >> xSubUV), nDstPitches[1], pSrcPlanes[1]->GetPointer(blxsrc >> xSubUV, blysrc>>ySubUV), pSrcPlanes[1]->GetPitch());
                            // chroma v
                            if(pSrcPlanes[2]) d->BLITCHROMA(pDstCur[2] + (xx >> xSubUV), nDstPitches[2], pSrcPlanes[2]->GetPointer(blxsrc >> xSubUV, blysrc>>ySubUV), pSrcPlanes[2]->GetPitch());
                        }


                        xx += (nBlkSizeX * bytesPerSample);

                    }
                    pDstCur[0] += (nBlkSizeY) * (nDstPitches[0]);
                    pSrcCur[0] += (nBlkSizeY) * (nSrcPitches[0]);
                    if (nSuperModeYUV & UVPLANES) {
                        pDstCur[1] += ( nBlkSizeY>>ySubUV) * (nDstPitches[1]);
                        pDstCur[2] += ( nBlkSizeY>>ySubUV) * (nDstPitches[2]) ;
                        pSrcCur[1] += ( nBlkSizeY>>ySubUV) * (nSrcPitches[1]);
                        pSrcCur[2] += ( nBlkSizeY>>ySubUV) * (nSrcPitches[2]) ;
                    }

                }
            }
            // -----------------------------------------------------------------
            else // overlap
            {
                OverlapWindows *OverWins = d->OverWins;
                OverlapWindows *OverWinsUV = d->OverWinsUV;
                uint8_t *DstTemp = new uint8_t[dstTempPitch * nHeight];
                uint8_t *DstTempU = NULL;
                uint8_t *DstTempV = NULL;
                if (nSuperModeYUV & UVPLANES) {
                    DstTempU = new uint8_t[dstTempPitchUV * nHeight];
                    DstTempV = new uint8_t[dstTempPitchUV * nHeight];
                }

                pDstTemp = DstTemp;
                pDstTempU = DstTempU;
                pDstTempV = DstTempV;
                memset(DstTemp, 0, nHeight_B * dstTempPitch);
                if (pPlanes[1])
                    memset(DstTempU, 0, (nHeight_B >> ySubUV) * dstTempPitchUV);
                if (pPlanes[2])
                    memset(DstTempV, 0, (nHeight_B >> ySubUV) * dstTempPitchUV);

                for (int by=0; by<nBlkY; by++)
                {
                    int wby = ((by + nBlkY - 3)/(nBlkY - 2))*3;
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        // select window
                        int wbx = (bx + nBlkX - 3)/(nBlkX - 2);
                        short *winOver = OverWins->GetWindow(wby + wbx);
                        short *winOverUV = NULL;
                        if (nSuperModeYUV & UVPLANES)
                            winOverUV = OverWinsUV->GetWindow(wby + wbx);

                        int i = by*nBlkX + bx;
                        const FakeBlockData &block = balls.GetBlock(0, i);

                        blx = block.GetX() * nPel + block.GetMV().x;
                        bly = block.GetY() * nPel + block.GetMV().y  + fieldShift;

                        if (block.GetSAD() < thSAD)
                        {
                            // luma
                            d->OVERSLUMA(pDstTemp + xx*2, dstTempPitch, pPlanes[0]->GetPointer(blx, bly), pPlanes[0]->GetPitch(), winOver, nBlkSizeX);
                            // chroma u
                            if(pPlanes[1]) d->OVERSCHROMA(pDstTempU + (xx >> xSubUV)*2, dstTempPitchUV, pPlanes[1]->GetPointer(blx >> xSubUV, bly>>ySubUV), pPlanes[1]->GetPitch(), winOverUV, nBlkSizeX >> xSubUV);
                            // chroma v
                            if(pPlanes[2]) d->OVERSCHROMA(pDstTempV + (xx >> xSubUV)*2, dstTempPitchUV, pPlanes[2]->GetPointer(blx >> xSubUV, bly>>ySubUV), pPlanes[2]->GetPitch(), winOverUV, nBlkSizeX >> xSubUV);
                        }
                        else // bad compensation, use src
                        {
                            int blxsrc = bx * (nBlkSizeX - nOverlapX) * nPel;
                            int blysrc = by * (nBlkSizeY - nOverlapY) * nPel  + fieldShift;

                            d->OVERSLUMA(pDstTemp + xx*2, dstTempPitch, pSrcPlanes[0]->GetPointer(blxsrc, blysrc), pSrcPlanes[0]->GetPitch(), winOver, nBlkSizeX);
                            // chroma u
                            if(pSrcPlanes[1]) d->OVERSCHROMA(pDstTempU + (xx >> xSubUV)*2, dstTempPitchUV, pSrcPlanes[1]->GetPointer(blxsrc >> xSubUV, blysrc>>ySubUV), pSrcPlanes[1]->GetPitch(), winOverUV, nBlkSizeX >> xSubUV);
                            // chroma v
                            if(pSrcPlanes[2]) d->OVERSCHROMA(pDstTempV + (xx >> xSubUV)*2, dstTempPitchUV, pSrcPlanes[2]->GetPointer(blxsrc >> xSubUV, blysrc>>ySubUV), pSrcPlanes[2]->GetPitch(), winOverUV, nBlkSizeX >> xSubUV);
                        }

                        xx += (nBlkSizeX - nOverlapX) * bytesPerSample;
                    }

                    pDstTemp += dstTempPitch*(nBlkSizeY - nOverlapY);
                    pDstCur[0] += (nBlkSizeY - nOverlapY) * (nDstPitches[0]);
                    pSrcCur[0] += (nBlkSizeY - nOverlapY) * (nSrcPitches[0]);
                    if (nSuperModeYUV & UVPLANES) {
                        pDstTempU += dstTempPitchUV*((nBlkSizeY - nOverlapY)>>ySubUV);
                        pDstTempV += dstTempPitchUV*((nBlkSizeY - nOverlapY)>>ySubUV);
                        pDstCur[1] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nDstPitches[1]);
                        pDstCur[2] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nDstPitches[2]);
                        pSrcCur[1] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nSrcPitches[1]);
                        pSrcCur[2] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nSrcPitches[2]);
                    }
                }

                d->ToPixels(pDst[0], nDstPitches[0], DstTemp, dstTempPitch, nWidth_B, nHeight_B, bitsPerSample);
                if (pPlanes[1])
                    d->ToPixels(pDst[1], nDstPitches[1], DstTempU, dstTempPitchUV, nWidth_B >> xSubUV, nHeight_B>>ySubUV, bitsPerSample);
                if (pPlanes[2])
                    d->ToPixels(pDst[2], nDstPitches[2], DstTempV, dstTempPitchUV, nWidth_B >> xSubUV, nHeight_B>>ySubUV, bitsPerSample);

                delete[] DstTemp;
                if (nSuperModeYUV & UVPLANES) {
                    delete[] DstTempU;
                    delete[] DstTempV;
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
                if(pPlanes[1])
                    vs_bitblt(pDst[1] + (nWidth_B >> xSubUV) * bytesPerSample, nDstPitches[1],
                              scSrc[1] + ((nWidth_B >> xSubUV) + (nHPadding >> xSubUV)) * bytesPerSample + (nVPadding >> ySubUV) * scPitches[1], scPitches[1],
                              ((nWidth - nWidth_B) >> xSubUV) * bytesPerSample, nHeight_B >> ySubUV);
                // chroma v
                if(pPlanes[2])
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
                if(pPlanes[1])
                    vs_bitblt(pDst[1] + (nHeight_B >> ySubUV) * nDstPitches[1], nDstPitches[1],
                              scSrc[1] + nHPadding * bytesPerSample + ((nHeight_B + nVPadding) >> ySubUV) * scPitches[1], scPitches[1],
                              (nWidth >> xSubUV) * bytesPerSample, (nHeight - nHeight_B) >> ySubUV);
                // chroma v
                if(pPlanes[2])
                    vs_bitblt(pDst[2] + (nHeight_B >> ySubUV) * nDstPitches[2], nDstPitches[2],
                              scSrc[2] + nHPadding * bytesPerSample + ((nHeight_B + nVPadding) >> ySubUV) * scPitches[2], scPitches[2],
                              (nWidth >> xSubUV) * bytesPerSample, (nHeight - nHeight_B) >> ySubUV);
            }

            delete pSrcGOF;
            delete pRefGOF;

            vsapi->freeFrame(ref);
        }
        else { // balls.IsUsable()
            if ( !scBehavior && ( nref < d->vi->numFrames || !d->vi->numFrames ) && ( nref >= 0 )) {
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


        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvcompensateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVCompensateData *d = (MVCompensateData *)instanceData;

    if (d->bleh->nOverlapX || d->bleh->nOverlapY) {
        delete d->OverWins;
        if (d->nSuperModeYUV & UVPLANES)
            delete d->OverWinsUV;
    }

    delete d->mvClip;

    delete d->bleh;

    vsapi->freeNode(d->super);
    vsapi->freeNode(d->vectors);
    vsapi->freeNode(d->node);
    free(d);
}


static void selectFunctions(MVCompensateData *d) {
    const int xRatioUV = d->bleh->xRatioUV;
    const int yRatioUV = d->bleh->yRatioUV;
    const int nBlkSizeX = d->bleh->nBlkSizeX;
    const int nBlkSizeY = d->bleh->nBlkSizeY;

    OverlapsFunction overs[33][33];
    COPYFunction copys[33][33];

    if (d->vi->format->bitsPerSample == 8) {
        overs[2][2] = Overlaps_C<2,2, uint16_t, uint8_t>;
        copys[2][2] = Copy_C<2,2, uint8_t>;

        overs[2][4] = Overlaps_C<2,4, uint16_t, uint8_t>;
        copys[2][4] = Copy_C<2,4, uint8_t>;

        overs[4][2] = d->isse ? mvtools_Overlaps4x2_sse2 : Overlaps_C<4,2, uint16_t, uint8_t>;
        copys[4][2] = Copy_C<4,2, uint8_t>;

        overs[4][4] = d->isse ? mvtools_Overlaps4x4_sse2 : Overlaps_C<4,4, uint16_t, uint8_t>;
        copys[4][4] = Copy_C<4,4, uint8_t>;

        overs[4][8] = d->isse ? mvtools_Overlaps4x8_sse2 : Overlaps_C<4,8, uint16_t, uint8_t>;
        copys[4][8] = Copy_C<4,8, uint8_t>;

        overs[8][1] = d->isse ? mvtools_Overlaps8x1_sse2 : Overlaps_C<8,1, uint16_t, uint8_t>;
        copys[8][1] = Copy_C<8,1, uint8_t>;

        overs[8][2] = d->isse ? mvtools_Overlaps8x2_sse2 : Overlaps_C<8,2, uint16_t, uint8_t>;
        copys[8][2] = Copy_C<8,2, uint8_t>;

        overs[8][4] = d->isse ? mvtools_Overlaps8x4_sse2 : Overlaps_C<8,4, uint16_t, uint8_t>;
        copys[8][4] = Copy_C<8,4, uint8_t>;

        overs[8][8] = d->isse ? mvtools_Overlaps8x8_sse2 : Overlaps_C<8,8, uint16_t, uint8_t>;
        copys[8][8] = Copy_C<8,8, uint8_t>;

        overs[8][16] = d->isse ? mvtools_Overlaps8x16_sse2 : Overlaps_C<8,16, uint16_t, uint8_t>;
        copys[8][16] = Copy_C<8,16, uint8_t>;

        overs[16][1] = d->isse ? mvtools_Overlaps16x1_sse2 : Overlaps_C<16,1, uint16_t, uint8_t>;
        copys[16][1] = Copy_C<16,1, uint8_t>;

        overs[16][2] = d->isse ? mvtools_Overlaps16x2_sse2 : Overlaps_C<16,2, uint16_t, uint8_t>;
        copys[16][2] = Copy_C<16,2, uint8_t>;

        overs[16][4] = d->isse ? mvtools_Overlaps16x4_sse2 : Overlaps_C<16,4, uint16_t, uint8_t>;
        copys[16][4] = Copy_C<16,4, uint8_t>;

        overs[16][8] = d->isse ? mvtools_Overlaps16x8_sse2 : Overlaps_C<16,8, uint16_t, uint8_t>;
        copys[16][8] = Copy_C<16,8, uint8_t>;

        overs[16][16] = d->isse ? mvtools_Overlaps16x16_sse2 : Overlaps_C<16,16, uint16_t, uint8_t>;
        copys[16][16] = Copy_C<16,16, uint8_t>;

        overs[16][32] = d->isse ? mvtools_Overlaps16x32_sse2 : Overlaps_C<16,32, uint16_t, uint8_t>;
        copys[16][32] = Copy_C<16,32, uint8_t>;

        overs[32][8] = d->isse ? mvtools_Overlaps32x8_sse2 : Overlaps_C<32,8, uint16_t, uint8_t>;
        copys[32][8] = Copy_C<32,8, uint8_t>;

        overs[32][16] = d->isse ? mvtools_Overlaps32x16_sse2 : Overlaps_C<32,16, uint16_t, uint8_t>;
        copys[32][16] = Copy_C<32,16, uint8_t>;

        overs[32][32] = d->isse ? mvtools_Overlaps32x32_sse2 : Overlaps_C<32,32, uint16_t, uint8_t>;
        copys[32][32] = Copy_C<32,32, uint8_t>;

        d->ToPixels = ToPixels<uint16_t, uint8_t>;
    } else {
        overs[2][2] = Overlaps_C<2,2, uint32_t, uint16_t>;
        copys[2][2] = Copy_C<2,2, uint16_t>;

        overs[2][4] = Overlaps_C<2,4, uint32_t, uint16_t>;
        copys[2][4] = Copy_C<2,4, uint16_t>;

        overs[4][2] = Overlaps_C<4,2, uint32_t, uint16_t>;
        copys[4][2] = Copy_C<4,2, uint16_t>;

        overs[4][4] = Overlaps_C<4,4, uint32_t, uint16_t>;
        copys[4][4] = Copy_C<4,4, uint16_t>;

        overs[4][8] = Overlaps_C<4,8, uint32_t, uint16_t>;
        copys[4][8] = Copy_C<4,8, uint16_t>;

        overs[8][1] = Overlaps_C<8,1, uint32_t, uint16_t>;
        copys[8][1] = Copy_C<8,1, uint16_t>;

        overs[8][2] = Overlaps_C<8,2, uint32_t, uint16_t>;
        copys[8][2] = Copy_C<8,2, uint16_t>;

        overs[8][4] = Overlaps_C<8,4, uint32_t, uint16_t>;
        copys[8][4] = Copy_C<8,4, uint16_t>;

        overs[8][8] = Overlaps_C<8,8, uint32_t, uint16_t>;
        copys[8][8] = Copy_C<8,8, uint16_t>;

        overs[8][16] = Overlaps_C<8,16, uint32_t, uint16_t>;
        copys[8][16] = Copy_C<8,16, uint16_t>;

        overs[16][1] = Overlaps_C<16,1, uint32_t, uint16_t>;
        copys[16][1] = Copy_C<16,1, uint16_t>;

        overs[16][2] = Overlaps_C<16,2, uint32_t, uint16_t>;
        copys[16][2] = Copy_C<16,2, uint16_t>;

        overs[16][4] = Overlaps_C<16,4, uint32_t, uint16_t>;
        copys[16][4] = Copy_C<16,4, uint16_t>;

        overs[16][8] = Overlaps_C<16,8, uint32_t, uint16_t>;
        copys[16][8] = Copy_C<16,8, uint16_t>;

        overs[16][16] = Overlaps_C<16,16, uint32_t, uint16_t>;
        copys[16][16] = Copy_C<16,16, uint16_t>;

        overs[16][32] = Overlaps_C<16,32, uint32_t, uint16_t>;
        copys[16][32] = Copy_C<16,32, uint16_t>;

        overs[32][8] = Overlaps_C<32,8, uint32_t, uint16_t>;
        copys[32][8] = Copy_C<32,8, uint16_t>;

        overs[32][16] = Overlaps_C<32,16, uint32_t, uint16_t>;
        copys[32][16] = Copy_C<32,16, uint16_t>;

        overs[32][32] = Overlaps_C<32,32, uint32_t, uint16_t>;
        copys[32][32] = Copy_C<32,32, uint16_t>;

        d->ToPixels = ToPixels<uint32_t, uint16_t>;
    }

    d->OVERSLUMA = overs[nBlkSizeX][nBlkSizeY];
    d->BLITLUMA = copys[nBlkSizeX][nBlkSizeY];

    d->OVERSCHROMA = overs[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
    d->BLITCHROMA = copys[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
}


static void VS_CC mvcompensateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
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

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("Compensate: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
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

    try {
        d.mvClip = new MVClipDicks(d.vectors, d.nSCD1, d.nSCD2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("Compensate: ").append(e.what()).c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        return;
    }

    try {
        d.bleh = new MVFilter(d.vectors, "Compensate", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("Compensate: ").append(e.what()).c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        delete d.mvClip;
        return;
    }


    if (d.fields && d.bleh->nPel < 2) {
        vsapi->setError(out, "Compensate: fields option requires pel > 1.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        delete d.mvClip;
        delete d.bleh;
        return;
    }

    d.thSAD = (int64_t)d.thSAD * d.mvClip->GetThSCD1() / d.nSCD1; // normalize to block SAD


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);


    d.dstTempPitch = ((d.bleh->nWidth + 15)/16)*16 * d.vi->format->bytesPerSample * 2;
    d.dstTempPitchUV = (((d.bleh->nWidth / d.bleh->xRatioUV) + 15)/16)*16 * d.vi->format->bytesPerSample * 2;


    d.supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = d.supervi->width;

    if (d.bleh->nHeight != nHeightS || d.bleh->nHeight != d.vi->height || d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2 || d.bleh->nWidth != d.vi->width) {
        vsapi->setError(out, "Compensate: wrong source or super clip frame size.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.node);
        delete d.mvClip;
        delete d.bleh;
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, "Compensate: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.node);
        delete d.mvClip;
        delete d.bleh;
        return;
    }

    if (d.vi->format->bitsPerSample > 8)
        d.isse = 0;

    if (d.bleh->nOverlapX || d.bleh->nOverlapY) {
        d.OverWins = new OverlapWindows(d.bleh->nBlkSizeX, d.bleh->nBlkSizeY, d.bleh->nOverlapX, d.bleh->nOverlapY);
        if (d.nSuperModeYUV & UVPLANES)
            d.OverWinsUV = new OverlapWindows(d.bleh->nBlkSizeX / d.bleh->xRatioUV, d.bleh->nBlkSizeY / d.bleh->yRatioUV, d.bleh->nOverlapX / d.bleh->xRatioUV, d.bleh->nOverlapY / d.bleh->yRatioUV);
    }

    selectFunctions(&d);


    data = (MVCompensateData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Compensate", mvcompensateInit, mvcompensateGetFrame, mvcompensateFree, fmParallel, 0, data, core);
}


extern "C" void mvcompensateRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
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
            "tff:int:opt;"
            , mvcompensateCreate, 0, plugin);
}
