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

#include "MVInterface.h"
#include "CopyCode.h"
#include "Overlap.h"
#include "MaskFun.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *super;
    VSNodeRef *vectors;

    bool scBehavior;
    int thSAD;
    bool fields;
    int nSCD1;
    int nSCD2;
    int isse;
    int tff;
    int tffexists;

    MVClipDicks *mvClip;

    MVFilter *bleh;

    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;

    int dstShortPitch;
    int dstShortPitchUV;

    OverlapsFunction OVERSLUMA;
    OverlapsFunction OVERSCHROMA;
    COPYFunction *BLITLUMA;
    COPYFunction *BLITCHROMA;
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
        unsigned short *pDstShort;
        unsigned short *pDstShortU;
        unsigned short *pDstShortV;
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
        const int yRatioUV = d->bleh->yRatioUV;
        const int nOverlapX = d->bleh->nOverlapX;
        const int nOverlapY = d->bleh->nOverlapY;
        const int nBlkSizeX = d->bleh->nBlkSizeX;
        const int nBlkSizeY = d->bleh->nBlkSizeY;
        const int nBlkX = d->bleh->nBlkX;
        const int nBlkY = d->bleh->nBlkY;
        const int isse = d->isse;
        const int thSAD = d->thSAD;
        const int dstShortPitch = d->dstShortPitch;
        const int dstShortPitchUV = d->dstShortPitchUV;
        const int nSuperModeYUV = d->nSuperModeYUV;
        const int nPel = d->bleh->nPel;
        const int nHPadding = d->bleh->nHPadding;
        const int nVPadding = d->bleh->nVPadding;
        const int scBehavior = d->scBehavior;
        const int fields = d->fields;


        int nWidth_B = nBlkX*(nBlkSizeX - nOverlapX) + nOverlapX;
        int nHeight_B = nBlkY*(nBlkSizeY - nOverlapY) + nOverlapY;

        int ySubUV = (yRatioUV == 2) ? 1 : 0;

        if ( balls.IsUsable() )
        {
            const VSFrameRef *ref = vsapi->getFrameFilter(nref, d->super, frameCtx);
            for (int i = 0; i < 3; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
            }

            MVGroupOfFrames *pRefGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, nSuperModeYUV, isse, yRatioUV);
            MVGroupOfFrames *pSrcGOF = new MVGroupOfFrames(d->nSuperLevels, nWidth, nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, nSuperModeYUV, isse, yRatioUV);

            pRefGOF->Update(YUVPLANES, (uint8_t*)pRef[0], nRefPitches[0], (uint8_t*)pRef[1], nRefPitches[1], (uint8_t*)pRef[2], nRefPitches[2]);// v2.0

            pSrcGOF->Update(YUVPLANES, (uint8_t*)pSrc[0], nSrcPitches[0], (uint8_t*)pSrc[1], nSrcPitches[1], (uint8_t*)pSrc[2], nSrcPitches[2]);


            MVPlane *pPlanes[3];

            pPlanes[0] = pRefGOF->GetFrame(0)->GetPlane(YPLANE);
            pPlanes[1] = pRefGOF->GetFrame(0)->GetPlane(UPLANE);
            pPlanes[2] = pRefGOF->GetFrame(0)->GetPlane(VPLANE);

            MVPlane *pSrcPlanes[3];

            pSrcPlanes[0] = pSrcGOF->GetFrame(0)->GetPlane(YPLANE);
            pSrcPlanes[1] = pSrcGOF->GetFrame(0)->GetPlane(UPLANE);
            pSrcPlanes[2] = pSrcGOF->GetFrame(0)->GetPlane(VPLANE);

            pDstCur[0] = pDst[0];
            pDstCur[1] = pDst[1];
            pDstCur[2] = pDst[2];
            pSrcCur[0] = pSrc[0];
            pSrcCur[1] = pSrc[1];
            pSrcCur[2] = pSrc[2];

            int fieldShift = 0;
            if (fields && nPel > 1 && ((nref-n) %2 != 0))
            {
                int err;
                const VSMap *props = vsapi->getFramePropsRO(src);
                bool paritySrc = vsapi->propGetInt(props, "_Field", 0, &err); //child->GetParity(n);
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
                bool parityRef = vsapi->propGetInt(props, "_Field", 0, &err); //child->GetParity(nref);
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
                            if(pPlanes[1]) d->BLITCHROMA(pDstCur[1] + (xx>>1), nDstPitches[1], pPlanes[1]->GetPointer(blx>>1, bly>>ySubUV), pPlanes[1]->GetPitch());
                            // chroma v
                            if(pPlanes[2]) d->BLITCHROMA(pDstCur[2] + (xx>>1), nDstPitches[2], pPlanes[2]->GetPointer(blx>>1, bly>>ySubUV), pPlanes[2]->GetPitch());
                        }
                        else
                        {
                            int blxsrc = bx * (nBlkSizeX) * nPel;
                            int blysrc = by * (nBlkSizeY) * nPel  + fieldShift;

                            d->BLITLUMA(pDstCur[0] + xx, nDstPitches[0], pSrcPlanes[0]->GetPointer(blxsrc, blysrc), pSrcPlanes[0]->GetPitch());
                            // chroma u
                            if(pSrcPlanes[1]) d->BLITCHROMA(pDstCur[1] + (xx>>1), nDstPitches[1], pSrcPlanes[1]->GetPointer(blxsrc>>1, blysrc>>ySubUV), pSrcPlanes[1]->GetPitch());
                            // chroma v
                            if(pSrcPlanes[2]) d->BLITCHROMA(pDstCur[2] + (xx>>1), nDstPitches[2], pSrcPlanes[2]->GetPointer(blxsrc>>1, blysrc>>ySubUV), pSrcPlanes[2]->GetPitch());
                        }


                        xx += (nBlkSizeX);

                    }
                    pDstCur[0] += (nBlkSizeY) * (nDstPitches[0]);
                    pDstCur[1] += ( nBlkSizeY>>ySubUV) * (nDstPitches[1]);
                    pDstCur[2] += ( nBlkSizeY>>ySubUV) * (nDstPitches[2]) ;
                    pSrcCur[0] += (nBlkSizeY) * (nSrcPitches[0]);
                    pSrcCur[1] += ( nBlkSizeY>>ySubUV) * (nSrcPitches[1]);
                    pSrcCur[2] += ( nBlkSizeY>>ySubUV) * (nSrcPitches[2]) ;

                }
            }
            // -----------------------------------------------------------------
            else // overlap
            {
                OverlapWindows *OverWins = new OverlapWindows(nBlkSizeX, nBlkSizeY, nOverlapX, nOverlapY);
                OverlapWindows *OverWinsUV = new OverlapWindows(nBlkSizeX/2, nBlkSizeY/yRatioUV, nOverlapX/2, nOverlapY/yRatioUV);
                unsigned short *DstShort = new unsigned short[dstShortPitch*nHeight];
                unsigned short *DstShortU = new unsigned short[dstShortPitchUV*nHeight];
                unsigned short *DstShortV = new unsigned short[dstShortPitchUV*nHeight];

                pDstShort = DstShort;
                MemZoneSet(reinterpret_cast<unsigned char*>(DstShort), 0, nWidth_B*2, nHeight_B, 0, 0, dstShortPitch*2);
                pDstShortU = DstShortU;
                if(pPlanes[1]) MemZoneSet(reinterpret_cast<unsigned char*>(DstShortU), 0, nWidth_B, nHeight_B>>ySubUV, 0, 0, dstShortPitchUV*2);
                pDstShortV = DstShortV;
                if(pPlanes[2]) MemZoneSet(reinterpret_cast<unsigned char*>(DstShortV), 0, nWidth_B, nHeight_B>>ySubUV, 0, 0, dstShortPitchUV*2);

                for (int by=0; by<nBlkY; by++)
                {
                    int wby = ((by + nBlkY - 3)/(nBlkY - 2))*3;
                    int xx = 0;
                    for (int bx=0; bx<nBlkX; bx++)
                    {
                        // select window
                        int wbx = (bx + nBlkX - 3)/(nBlkX - 2);
                        short *winOver = OverWins->GetWindow(wby + wbx);
                        short *winOverUV = OverWinsUV->GetWindow(wby + wbx);

                        int i = by*nBlkX + bx;
                        const FakeBlockData &block = balls.GetBlock(0, i);

                        blx = block.GetX() * nPel + block.GetMV().x;
                        bly = block.GetY() * nPel + block.GetMV().y  + fieldShift;

                        if (block.GetSAD() < thSAD)
                        {
                            // luma
                            d->OVERSLUMA(pDstShort + xx, dstShortPitch, pPlanes[0]->GetPointer(blx, bly), pPlanes[0]->GetPitch(), winOver, nBlkSizeX);
                            // chroma u
                            if(pPlanes[1]) d->OVERSCHROMA(pDstShortU + (xx>>1), dstShortPitchUV, pPlanes[1]->GetPointer(blx>>1, bly>>ySubUV), pPlanes[1]->GetPitch(), winOverUV, nBlkSizeX/2);
                            // chroma v
                            if(pPlanes[2]) d->OVERSCHROMA(pDstShortV + (xx>>1), dstShortPitchUV, pPlanes[2]->GetPointer(blx>>1, bly>>ySubUV), pPlanes[2]->GetPitch(), winOverUV, nBlkSizeX/2);
                        }
                        else // bad compensation, use src
                        {
                            int blxsrc = bx * (nBlkSizeX - nOverlapX) * nPel;
                            int blysrc = by * (nBlkSizeY - nOverlapY) * nPel  + fieldShift;

                            d->OVERSLUMA(pDstShort + xx, dstShortPitch, pSrcPlanes[0]->GetPointer(blxsrc, blysrc), pSrcPlanes[0]->GetPitch(), winOver, nBlkSizeX);
                            // chroma u
                            if(pSrcPlanes[1]) d->OVERSCHROMA(pDstShortU + (xx>>1), dstShortPitchUV, pSrcPlanes[1]->GetPointer(blxsrc>>1, blysrc>>ySubUV), pSrcPlanes[1]->GetPitch(), winOverUV, nBlkSizeX/2);
                            // chroma v
                            if(pSrcPlanes[2]) d->OVERSCHROMA(pDstShortV + (xx>>1), dstShortPitchUV, pSrcPlanes[2]->GetPointer(blxsrc>>1, blysrc>>ySubUV), pSrcPlanes[2]->GetPitch(), winOverUV, nBlkSizeX/2);
                        }

                        xx += (nBlkSizeX - nOverlapX);

                    }
                    pDstShort += dstShortPitch*(nBlkSizeY - nOverlapY);
                    pDstShortU += dstShortPitchUV*((nBlkSizeY - nOverlapY)>>ySubUV);
                    pDstShortV += dstShortPitchUV*((nBlkSizeY - nOverlapY)>>ySubUV);
                    pDstCur[0] += (nBlkSizeY - nOverlapY) * (nDstPitches[0]);
                    pDstCur[1] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nDstPitches[1]);
                    pDstCur[2] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nDstPitches[2]);
                    pSrcCur[0] += (nBlkSizeY - nOverlapY) * (nSrcPitches[0]);
                    pSrcCur[1] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nSrcPitches[1]);
                    pSrcCur[2] += ((nBlkSizeY - nOverlapY)>>ySubUV) * (nSrcPitches[2]) ;


                }
                Short2Bytes(pDst[0], nDstPitches[0], DstShort, dstShortPitch, nWidth_B, nHeight_B);
                if(pPlanes[1]) Short2Bytes(pDst[1], nDstPitches[1], DstShortU, dstShortPitchUV, nWidth_B>>1, nHeight_B>>ySubUV);
                if(pPlanes[2]) Short2Bytes(pDst[2], nDstPitches[2], DstShortV, dstShortPitchUV, nWidth_B>>1, nHeight_B>>ySubUV);

                delete OverWins;
                delete OverWinsUV;
                delete[] DstShort;
                delete[] DstShortU;
                delete[] DstShortV;
            }

            if (nWidth_B < nWidth) // padding of right non-covered region
            {
                // luma
                if (scBehavior)
                    BitBlt(pDst[0] + nWidth_B, nDstPitches[0], pSrc[0] + nWidth_B + nHPadding + nVPadding * nSrcPitches[0], nSrcPitches[0], nWidth-nWidth_B, nHeight_B, isse);
                else
                    BitBlt(pDst[0] + nWidth_B, nDstPitches[0], pRef[0] + nWidth_B + nHPadding + nVPadding * nRefPitches[0], nRefPitches[0], nWidth-nWidth_B, nHeight_B, isse);
                // chroma u
                if(pPlanes[1])
                {
                    if (scBehavior)
                        BitBlt(pDst[1] + (nWidth_B>>1), nDstPitches[1], pSrc[1] + (nWidth_B>>1) + (nHPadding>>1) + (nVPadding>>ySubUV) * nSrcPitches[1], nSrcPitches[1], (nWidth-nWidth_B)>>1, nHeight_B>>ySubUV, isse);
                    else
                        BitBlt(pDst[1] + (nWidth_B>>1), nDstPitches[1], pRef[1] + (nWidth_B>>1) + (nHPadding>>1) + (nVPadding>>ySubUV) * nRefPitches[1], nRefPitches[1], (nWidth-nWidth_B)>>1, nHeight_B>>ySubUV, isse);
                }
                // chroma v
                if(pPlanes[2])
                {
                    if (scBehavior)
                        BitBlt(pDst[2] + (nWidth_B>>1), nDstPitches[2], pSrc[2] + (nWidth_B>>1) + (nHPadding>>1) + (nVPadding>>ySubUV) * nSrcPitches[2], nSrcPitches[2], (nWidth-nWidth_B)>>1, nHeight_B>>ySubUV, isse);
                    else
                        BitBlt(pDst[2] + (nWidth_B>>1), nDstPitches[2], pRef[2] + (nWidth_B>>1) + (nHPadding>>1) + (nVPadding>>ySubUV) * nRefPitches[2], nRefPitches[2], (nWidth-nWidth_B)>>1, nHeight_B>>ySubUV, isse);
                }
            }

            if (nHeight_B < nHeight) // padding of bottom non-covered region
            {
                // luma
                if (scBehavior)
                    BitBlt(pDst[0] + nHeight_B*nDstPitches[0], nDstPitches[0], pSrc[0] + nHPadding + (nHeight_B + nVPadding) * nSrcPitches[0], nSrcPitches[0], nWidth, nHeight-nHeight_B, isse);
                else
                    BitBlt(pDst[0] + nHeight_B*nDstPitches[0], nDstPitches[0], pRef[0] + nHPadding + (nHeight_B + nVPadding) * nRefPitches[0], nRefPitches[0], nWidth, nHeight-nHeight_B, isse);
                // chroma u
                if(pPlanes[1])
                {
                    if (scBehavior)
                        BitBlt(pDst[1] + (nHeight_B>>ySubUV)*nDstPitches[1], nDstPitches[1], pSrc[1] + nHPadding + ((nHeight_B + nVPadding)>>ySubUV) * nSrcPitches[1], nSrcPitches[1], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
                    else
                        BitBlt(pDst[1] + (nHeight_B>>ySubUV)*nDstPitches[1], nDstPitches[1], pRef[1] + nHPadding + ((nHeight_B + nVPadding)>>ySubUV) * nRefPitches[1], nRefPitches[1], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
                }
                // chroma v
                if(pPlanes[2])
                {
                    if (scBehavior)
                        BitBlt(pDst[2] + (nHeight_B>>ySubUV)*nDstPitches[2], nDstPitches[2], pSrc[2] + nHPadding + ((nHeight_B + nVPadding)>>ySubUV) * nSrcPitches[2], nSrcPitches[2], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
                    else
                        BitBlt(pDst[2] + (nHeight_B>>ySubUV)*nDstPitches[2], nDstPitches[2], pRef[2] + nHPadding + ((nHeight_B + nVPadding)>>ySubUV) * nRefPitches[2], nRefPitches[2], nWidth>>1, (nHeight-nHeight_B)>>ySubUV, isse);
                }
            }

            delete pSrcGOF;
            delete pRefGOF;

            vsapi->freeFrame(ref);
        }
        else { // balls.IsUsable()
            if ( !scBehavior && ( nref < d->vi->numFrames ) && ( nref >= 0 )) {
                vsapi->freeFrame(src);
                src = vsapi->getFrameFilter(nref, d->super, frameCtx);
            }

            for (int i = 0; i < 3; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
            }

            int nOffset[3];

            nOffset[0] = nHPadding + nVPadding * nSrcPitches[0];
            nOffset[1] = nHPadding / 2 + (nVPadding / yRatioUV) * nSrcPitches[1];
            nOffset[2] = nOffset[1];

            vs_bitblt(pDst[0], nDstPitches[0], pSrc[0] + nOffset[0], nSrcPitches[0], nWidth, nHeight);
            vs_bitblt(pDst[1], nDstPitches[1], pSrc[1] + nOffset[1], nSrcPitches[1], nWidth / 2, nHeight / yRatioUV);
            vs_bitblt(pDst[2], nDstPitches[2], pSrc[2] + nOffset[2], nSrcPitches[2], nWidth / 2, nHeight / yRatioUV);
        }


        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvcompensateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVCompensateData *d = (MVCompensateData *)instanceData;

    delete d->mvClip;

    delete d->bleh;

    vsapi->freeNode(d->super);
    vsapi->freeNode(d->vectors);
    vsapi->freeNode(d->node);
    free(d);
}


static void selectFunctions(MVCompensateData *d) {
    const int yRatioUV = d->bleh->yRatioUV;
    const int nBlkSizeX = d->bleh->nBlkSizeX;
    const int nBlkSizeY = d->bleh->nBlkSizeY;

    if (d->isse)
    {
        switch (nBlkSizeX)
        {
            case 32:
                if (nBlkSizeY==16) {
                    d->BLITLUMA = mvtools_Copy32x16_sse2;
                    d->OVERSLUMA = mvtools_Overlaps32x16_sse2;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = mvtools_Copy16x8_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps16x8_sse2;
                    } else {
                        d->BLITCHROMA = mvtools_Copy16x16_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps16x16_sse2;
                    }
                } else if (nBlkSizeY==32) {
                    d->BLITLUMA = mvtools_Copy32x32_sse2;
                    d->OVERSLUMA = mvtools_Overlaps32x32_sse2;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = mvtools_Copy16x16_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps16x16_sse2;
                    } else {
                        d->BLITCHROMA = mvtools_Copy16x32_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps16x32_sse2;
                    }
                } break;
            case 16:
                if (nBlkSizeY==16) {
                    d->BLITLUMA = mvtools_Copy16x16_sse2;
                    d->OVERSLUMA = mvtools_Overlaps16x16_sse2;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = mvtools_Copy8x8_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps8x8_sse2;
                    } else {
                        d->BLITCHROMA = mvtools_Copy8x16_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps8x16_sse2;
                    }
                } else if (nBlkSizeY==8) {
                    d->BLITLUMA = mvtools_Copy16x8_sse2;
                    d->OVERSLUMA = mvtools_Overlaps16x8_sse2;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = mvtools_Copy8x4_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps8x4_sse2;
                    } else {
                        d->BLITCHROMA = mvtools_Copy8x8_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps8x8_sse2;
                    }
                } else if (nBlkSizeY==2) {
                    d->BLITLUMA = mvtools_Copy16x2_sse2;
                    d->OVERSLUMA = mvtools_Overlaps16x2_sse2;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = mvtools_Copy8x1_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps8x1_sse2;
                    } else {
                        d->BLITCHROMA = mvtools_Copy8x2_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps8x2_sse2;
                    }
                }
                break;
            case 4:
                d->BLITLUMA = mvtools_Copy4x4_sse2;
                d->OVERSLUMA = mvtools_Overlaps4x4_sse2;
                if (yRatioUV==2) {
                    d->BLITCHROMA = mvtools_Copy2x2_sse2;
                    d->OVERSCHROMA = Overlaps_C<2,2>;
                } else {
                    d->BLITCHROMA = mvtools_Copy2x4_sse2;
                    d->OVERSCHROMA = Overlaps_C<2,4>;
                }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {
                    d->BLITLUMA = mvtools_Copy8x8_sse2;
                    d->OVERSLUMA = mvtools_Overlaps8x8_sse2;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = mvtools_Copy4x4_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps4x4_sse2;
                    } else {
                        d->BLITCHROMA = mvtools_Copy4x8_sse2;
                        d->OVERSCHROMA = mvtools_Overlaps4x8_sse2;
                    }
                } else if (nBlkSizeY==4) { // 8x4
                    d->BLITLUMA = mvtools_Copy8x4_sse2;
                    d->OVERSLUMA = mvtools_Overlaps8x4_sse2;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = mvtools_Copy4x2_sse2; // idem
                        d->OVERSCHROMA = mvtools_Overlaps4x2_sse2;
                    } else {
                        d->BLITCHROMA = mvtools_Copy4x4_sse2; // idem
                        d->OVERSCHROMA = mvtools_Overlaps4x4_sse2;
                    }
                }
        }
    }
    else // pure C, no isse opimization ("mmx" version could be used, but it's more like a debugging version)
    {
        switch (nBlkSizeX)
        {
            case 32:
                if (nBlkSizeY==16) {
                    d->BLITLUMA = Copy_C<32,16>;         d->OVERSLUMA = Overlaps_C<32,16>;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = Copy_C<16,8>;	         d->OVERSCHROMA = Overlaps_C<16,8>;
                    } else {
                        d->BLITCHROMA = Copy_C<16>;          d->OVERSCHROMA = Overlaps_C<16,16>;
                    }
                } else if (nBlkSizeY==32) {
                    d->BLITLUMA = Copy_C<32,32>;         d->OVERSLUMA = Overlaps_C<32,32>;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = Copy_C<16,16>;	         d->OVERSCHROMA = Overlaps_C<16,16>;
                    } else {
                        d->BLITCHROMA = Copy_C<16,32>;	         d->OVERSCHROMA = Overlaps_C<16,32>;
                    }
                } break;
            case 16:
                if (nBlkSizeY==16) {
                    d->BLITLUMA = Copy_C<16>;         d->OVERSLUMA = Overlaps_C<16,16>;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = Copy_C<8>;	         d->OVERSCHROMA = Overlaps_C<8,8>;
                    } else {
                        d->BLITCHROMA = Copy_C<8,16>;	         d->OVERSCHROMA = Overlaps_C<8,16>;
                    }
                } else if (nBlkSizeY==8) {
                    d->BLITLUMA = Copy_C<16,8>;         d->OVERSLUMA = Overlaps_C<16,8>;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = Copy_C<8,4>;	         d->OVERSCHROMA = Overlaps_C<8,4>;
                    } else {
                        d->BLITCHROMA = Copy_C<8>;	         d->OVERSCHROMA = Overlaps_C<8,8>;
                    }
                } else if (nBlkSizeY==2) {
                    d->BLITLUMA = Copy_C<16,2>;         d->OVERSLUMA = Overlaps_C<16,2>;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = Copy_C<8,1>;	         d->OVERSCHROMA = Overlaps_C<8,1>;
                    } else {
                        d->BLITCHROMA = Copy_C<8,2>;	         d->OVERSCHROMA = Overlaps_C<8,2>;
                    }
                }
                break;
            case 4:
                d->BLITLUMA = Copy_C<4>;         d->OVERSLUMA = Overlaps_C<4,4>;
                if (yRatioUV==2) {
                    d->BLITCHROMA = Copy_C<2>;			 d->OVERSCHROMA = Overlaps_C<2,2>;
                } else {
                    d->BLITCHROMA = Copy_C<2,4>;			 d->OVERSCHROMA = Overlaps_C<2,4>;
                }
                break;
            case 8:
            default:
                if (nBlkSizeY==8) {
                    d->BLITLUMA = Copy_C<8>;         d->OVERSLUMA = Overlaps_C<8,8>;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = Copy_C<4>;	         d->OVERSCHROMA = Overlaps_C<4,4>;
                    }
                    else {
                        d->BLITCHROMA = Copy_C<4,8>;	         d->OVERSCHROMA = Overlaps_C<4,8>;
                    }
                } else if (nBlkSizeY==4) { // 8x4
                    d->BLITLUMA = Copy_C<8,4>;         d->OVERSLUMA = Overlaps_C<8,4>;
                    if (yRatioUV==2) {
                        d->BLITCHROMA = Copy_C<4,2>;	         d->OVERSCHROMA = Overlaps_C<4,2>;
                    } else {
                        d->BLITCHROMA = Copy_C<4>;	         d->OVERSCHROMA = Overlaps_C<4,4>;
                    }
                }
        }
    }
}


static void VS_CC mvcompensateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVCompensateData d;
    MVCompensateData *data;

    int err;

    d.scBehavior = vsapi->propGetInt(in, "scbehavior", 0, &err);
    if (err)
        d.scBehavior = 1;

    d.thSAD = vsapi->propGetInt(in, "thsad", 0, &err);
    if (err)
        d.thSAD = 10000;

    d.fields = vsapi->propGetInt(in, "fields", 0, &err);

    d.nSCD1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.nSCD1 = MV_DEFAULT_SCD1;

    d.nSCD2 = vsapi->propGetInt(in, "thscd2", 0, &err);
    if (err)
        d.nSCD2 = MV_DEFAULT_SCD2;

    d.isse = vsapi->propGetInt(in, "isse", 0, &err);
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
    int nHeightS = vsapi->propGetInt(props, "Super height", 0, &evil_err[0]);
    d.nSuperHPad = vsapi->propGetInt(props, "Super hpad", 0, &evil_err[1]);
    d.nSuperVPad = vsapi->propGetInt(props, "Super vpad", 0, &evil_err[2]);
    d.nSuperPel = vsapi->propGetInt(props, "Super pel", 0, &evil_err[3]);
    d.nSuperModeYUV = vsapi->propGetInt(props, "Super modeyuv", 0, &evil_err[4]);
    d.nSuperLevels = vsapi->propGetInt(props, "Super levels", 0, &evil_err[5]);
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
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        return;
    }

    try {
        d.bleh = new MVFilter(d.vectors, "Compensate", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
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

    d.thSAD = d.thSAD * d.mvClip->GetThSCD1() / d.nSCD1; // normalize to block SAD


    d.dstShortPitch = ((d.bleh->nWidth + 15)/16)*16;
    d.dstShortPitchUV = (((d.bleh->nWidth / 2) + 15)/16)*16;


    selectFunctions(&d);


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.bleh->nHeight != nHeightS || d.bleh->nHeight != d.vi->height || d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2 || d.bleh->nWidth != d.vi->width) {
        vsapi->setError(out, "Compensate: wrong source or super clip frame size.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.node);
        delete d.mvClip;
        delete d.bleh;
        return;
    }

    int id = d.vi->format->id;
    if (!isConstantFormat(d.vi) || (id != pfYUV420P8 && id != pfYUV422P8)) {
        vsapi->setError(out, "Compensate: input clip must be YUV420P8 or YUV422P8, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.node);
        delete d.mvClip;
        delete d.bleh;
        return;
    }


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
                 "tff:int:opt;"
                 , mvcompensateCreate, 0, plugin);
}
