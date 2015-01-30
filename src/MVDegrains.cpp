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

#include "MVDegrains.h"
#include "MVInterface.h"
#include "Overlap.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *super;
    VSNodeRef *vectors[6];

    int thSAD[3];
    int YUVplanes;
    int nLimit[3];
    int nSCD1;
    int nSCD2;
    int isse;

    MVClipDicks *mvClips[6];

    MVFilter *bleh;

    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;

    int dstTempPitch;

    OverlapsFunction OVERS[3];
    DenoiseFunction DEGRAIN[3];
    LimitFunction LimitChanges;
    ToPixelsFunction ToPixels;

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
} MVDegrainData;


static void VS_CC mvdegrainInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVDegrainData *d = (MVDegrainData *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


template <int radius>
static const VSFrameRef *VS_CC mvdegrainGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVDegrainData *d = (MVDegrainData *) * instanceData;

    if (activationReason == arInitial) {
        if (radius > 2)
            vsapi->requestFrameFilter(n, d->vectors[Forward3], frameCtx);
        if (radius > 1)
            vsapi->requestFrameFilter(n, d->vectors[Forward2], frameCtx);
        vsapi->requestFrameFilter(n, d->vectors[Forward1], frameCtx);
        vsapi->requestFrameFilter(n, d->vectors[Backward1], frameCtx);
        if (radius > 1)
            vsapi->requestFrameFilter(n, d->vectors[Backward2], frameCtx);
        if (radius > 2)
            vsapi->requestFrameFilter(n, d->vectors[Backward3], frameCtx);

        if (radius > 2) {
            int offF3 = -1 * d->mvClips[Forward3]->GetDeltaFrame();
            if (n + offF3 >= 0)
                vsapi->requestFrameFilter(n + offF3, d->super, frameCtx);
        }

        if (radius > 1) {
            int offF2 = -1 * d->mvClips[Forward2]->GetDeltaFrame();
            if (n + offF2 >= 0)
                vsapi->requestFrameFilter(n + offF2, d->super, frameCtx);
        }

        int offF = -1 * d->mvClips[Forward1]->GetDeltaFrame();
        if (n + offF >= 0)
            vsapi->requestFrameFilter(n + offF, d->super, frameCtx);

        int offB = d->mvClips[Backward1]->GetDeltaFrame();
        if (n + offB < d->vi->numFrames || !d->vi->numFrames)
            vsapi->requestFrameFilter(n + offB, d->super, frameCtx);

        if (radius > 1) {
            int offB2 = d->mvClips[Backward2]->GetDeltaFrame();
            if (n + offB2 < d->vi->numFrames || !d->vi->numFrames)
                vsapi->requestFrameFilter(n + offB2, d->super, frameCtx);
        }

        if (radius > 2) {
            int offB3 = d->mvClips[Backward3]->GetDeltaFrame();
            if (n + offB3 < d->vi->numFrames || !d->vi->numFrames)
                vsapi->requestFrameFilter(n + offB3, d->super, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

        int bitsPerSample = d->vi->format->bitsPerSample;
        int bytesPerSample = d->vi->format->bytesPerSample;

        uint8_t *pDst[3], *pDstCur[3];
        const uint8_t *pSrcCur[3];
        const uint8_t *pSrc[3];
        const uint8_t *pRefs[3][radius*2];
        int nDstPitches[3], nSrcPitches[3];
        int nRefPitches[3][radius*2];
        bool isUsable[radius*2];
        int nLogPel = (d->bleh->nPel == 4) ? 2 : (d->bleh->nPel == 2) ? 1 : 0;

        MVClipBalls *balls[radius*2];
        const VSFrameRef *refFrames[radius*2] = { 0 };

        for (int r = 0; r < radius*2; r++) {
            const VSFrameRef *frame = vsapi->getFrameFilter(n, d->vectors[r], frameCtx);
            balls[r] = new MVClipBalls(d->mvClips[r], vsapi);
            balls[r]->Update(frame);
            isUsable[r] = balls[r]->IsUsable();
            vsapi->freeFrame(frame);

            if (isUsable[r]) {
                int offset = d->mvClips[r]->GetDeltaFrame() * (d->mvClips[r]->IsBackward() ? 1 : -1);
                refFrames[r] = vsapi->getFrameFilter(n + offset, d->super, frameCtx);
            }
        }


        for (int i = 0; i < d->vi->format->numPlanes; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
            pSrc[i] = vsapi->getReadPtr(src, i);
            nSrcPitches[i] = vsapi->getStride(src, i);

            for (int r = 0; r < radius*2; r++)
                if (isUsable[r]) {
                    pRefs[i][r] = vsapi->getReadPtr(refFrames[r], i);
                    nRefPitches[i][r] = vsapi->getStride(refFrames[r], i);
                }
        }

        const int xSubUV = d->xSubUV;
        const int ySubUV = d->ySubUV;
        const int xRatioUV = d->bleh->xRatioUV;
        const int yRatioUV = d->bleh->yRatioUV;
        const int nBlkX = d->bleh->nBlkX;
        const int nBlkY = d->bleh->nBlkY;
        const int isse = d->isse;
        const int YUVplanes = d->YUVplanes;
        const int dstTempPitch = d->dstTempPitch;
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


        MVGroupOfFrames *pRefGOF[radius*2];
        for (int r = 0; r < radius*2; r++)
            pRefGOF[r] = new MVGroupOfFrames(d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, isse, xRatioUV, yRatioUV, bitsPerSample);


        OverlapWindows *OverWins[3] = { NULL, NULL, NULL };
        uint8_t *DstTemp = NULL;
        int tmpBlockPitch = nBlkSizeX[0] * bytesPerSample;
        uint8_t *tmpBlock = NULL;
        if (nOverlapX[0] > 0 || nOverlapY[0] > 0) {
            OverWins[0] = new OverlapWindows(nBlkSizeX[0], nBlkSizeY[0], nOverlapX[0], nOverlapY[0]);
            if (d->vi->format->colorFamily != cmGray) {
                OverWins[1] = new OverlapWindows(nBlkSizeX[1], nBlkSizeY[1], nOverlapX[1], nOverlapY[1]);
                OverWins[2] = OverWins[1];
            }
            DstTemp = new uint8_t[dstTempPitch * nHeight[0]];
            tmpBlock = new uint8_t[tmpBlockPitch * nBlkSizeY[0]];
        }

        MVPlane *pPlanes[3][radius*2] = { };

        MVPlaneSet planes[3] = { YPLANE, UPLANE, VPLANE };

        for (int r = 0; r < radius*2; r++)
            if (isUsable[r]) {
                pRefGOF[r]->Update(YUVplanes, (uint8_t*)pRefs[0][r], nRefPitches[0][r], (uint8_t*)pRefs[1][r], nRefPitches[1][r], (uint8_t*)pRefs[2][r], nRefPitches[2][r]);
                for (int plane = 0; plane < d->vi->format->numPlanes; plane++)
                    if (YUVplanes & planes[plane])
                        pPlanes[plane][r] = pRefGOF[r]->GetFrame(0)->GetPlane(planes[plane]);
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

                        const uint8_t *pointers[radius*2]; // Moved by the degrain function.
                        int strides[radius*2];

                        int WSrc, WRefs[radius*2];

                        for (int r = 0; r < radius*2; r++)
                            useBlock(pointers[r], strides[r], WRefs[r], isUsable[r], balls[r], i, pPlanes[plane][r], pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights<radius>(WSrc, WRefs);

                        d->DEGRAIN[plane](pDstCur[plane] + xx, nDstPitches[plane], pSrcCur[plane] + xx, nSrcPitches[plane],
                                pointers, strides,
                                WSrc, WRefs);

                        xx += nBlkSizeX[plane] * bytesPerSample;

                        if (bx == nBlkX - 1 && nWidth_B[0] < nWidth[0]) // right non-covered region
                            vs_bitblt(pDstCur[plane] + nWidth_B[plane] * bytesPerSample, nDstPitches[plane],
                                    pSrcCur[plane] + nWidth_B[plane] * bytesPerSample, nSrcPitches[plane],
                                    (nWidth[plane] - nWidth_B[plane]) * bytesPerSample, nBlkSizeY[plane]);
                    }
                    pDstCur[plane] += nBlkSizeY[plane] * (nDstPitches[plane]);
                    pSrcCur[plane] += nBlkSizeY[plane] * (nSrcPitches[plane]);

                    if (by == nBlkY - 1 && nHeight_B[0] < nHeight[0]) // bottom uncovered region
                        vs_bitblt(pDstCur[plane], nDstPitches[plane],
                                pSrcCur[plane], nSrcPitches[plane],
                                nWidth[plane] * bytesPerSample, nHeight[plane] - nHeight_B[plane]);
                }
            } else {// overlap
                uint8_t *pDstTemp = DstTemp;
                memset(pDstTemp, 0, dstTempPitch * nHeight_B[0]);

                for (int by = 0; by < nBlkY; by++) {
                    int wby = ((by + nBlkY - 3) / (nBlkY - 2)) * 3;
                    int xx = 0;
                    for (int bx = 0; bx < nBlkX; bx++) {
                        // select window
                        int wbx = (bx + nBlkX - 3) / (nBlkX - 2);
                        short *winOver = OverWins[plane]->GetWindow(wby + wbx);

                        int i = by * nBlkX + bx;

                        const uint8_t *pointers[radius*2]; // Moved by the degrain function.
                        int strides[radius*2];

                        int WSrc, WRefs[radius*2];

                        for (int r = 0; r < radius*2; r++)
                            useBlock(pointers[r], strides[r], WRefs[r], isUsable[r], balls[r], i, pPlanes[plane][r], pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights<radius>(WSrc, WRefs);

                        d->DEGRAIN[plane](tmpBlock, tmpBlockPitch, pSrcCur[plane] + xx, nSrcPitches[plane],
                                pointers, strides,
                                WSrc, WRefs);
                        d->OVERS[plane](pDstTemp + xx, dstTempPitch, tmpBlock, tmpBlockPitch, winOver, nBlkSizeX[plane]);

                        xx += (nBlkSizeX[plane] - nOverlapX[plane]) * bytesPerSample;

                    }
                    pSrcCur[plane] += (nBlkSizeY[plane] - nOverlapY[plane]) * nSrcPitches[plane];
                    pDstTemp += (nBlkSizeY[plane] - nOverlapY[plane]) * dstTempPitch;
                }

                d->ToPixels(pDst[plane], nDstPitches[plane], DstTemp, dstTempPitch, nWidth_B[plane], nHeight_B[plane], bitsPerSample);

                if (nWidth_B[0] < nWidth[0])
                    vs_bitblt(pDst[plane] + nWidth_B[plane] * bytesPerSample, nDstPitches[plane],
                            pSrc[plane] + nWidth_B[plane] * bytesPerSample, nSrcPitches[plane],
                            (nWidth[plane] - nWidth_B[plane]) * bytesPerSample, nHeight_B[plane]);

                if (nHeight_B[0] < nHeight[0]) // bottom noncovered region
                    vs_bitblt(pDst[plane] + nDstPitches[plane] * nHeight_B[plane], nDstPitches[plane],
                            pSrc[plane] + nSrcPitches[plane] * nHeight_B[plane], nSrcPitches[plane],
                            nWidth[plane] * bytesPerSample, nHeight[plane] - nHeight_B[plane]);
            }

            int pixelMax = (1 << bitsPerSample) - 1;
            if (nLimit[plane] < pixelMax)
                d->LimitChanges(pDst[plane], nDstPitches[plane],
                                pSrc[plane], nSrcPitches[plane],
                                nWidth[plane], nHeight[plane], nLimit[plane]);
        }


        if (tmpBlock)
            delete[] tmpBlock;

        if (OverWins[0])
            delete OverWins[0];
        if (OverWins[1])
            delete OverWins[1];

        if (DstTemp)
            delete[] DstTemp;

        for (int r = 0; r < radius*2; r++) {
            delete pRefGOF[r];

            if (refFrames[r])
                vsapi->freeFrame(refFrames[r]);

            delete balls[r];
        }

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


template <int radius>
static void VS_CC mvdegrainFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVDegrainData *d = (MVDegrainData *)instanceData;

    for (int r = 0; r < radius*2; r++) {
        delete d->mvClips[r];
        vsapi->freeNode(d->vectors[r]);
    }
    vsapi->freeNode(d->super);
    vsapi->freeNode(d->node);

    delete d->bleh;

    free(d);
}


template <int radius>
static void selectFunctions(MVDegrainData *d) {
    const int xRatioUV = d->bleh->xRatioUV;
    const int yRatioUV = d->bleh->yRatioUV;
    const int nBlkSizeX = d->bleh->nBlkSizeX;
    const int nBlkSizeY = d->bleh->nBlkSizeY;

    OverlapsFunction overs[33][33];
    DenoiseFunction degs[33][33];

    if (d->vi->format->bitsPerSample == 8) {
        overs[2][2] = Overlaps_C<2,2, uint16_t, uint8_t>;
        degs[2][2] = Degrain_C<radius, 2,2, uint8_t>;

        overs[2][4] = Overlaps_C<2,4, uint16_t, uint8_t>;
        degs[2][4] = Degrain_C<radius, 2,4, uint8_t>;

        overs[4][2] = d->isse ? mvtools_Overlaps4x2_sse2 : Overlaps_C<4,2, uint16_t, uint8_t>;
        degs[4][2] = d->isse ? Degrain_sse2<radius, 4,2> : Degrain_C<radius, 4,2, uint8_t>;

        overs[4][4] = d->isse ? mvtools_Overlaps4x4_sse2 : Overlaps_C<4,4, uint16_t, uint8_t>;
        degs[4][4] = d->isse ? Degrain_sse2<radius, 4,4> : Degrain_C<radius, 4,4, uint8_t>;

        overs[4][8] = d->isse ? mvtools_Overlaps4x8_sse2 : Overlaps_C<4,8, uint16_t, uint8_t>;
        degs[4][8] = d->isse ? Degrain_sse2<radius, 4,8> : Degrain_C<radius, 4,8, uint8_t>;

        overs[8][1] = d->isse ? mvtools_Overlaps8x1_sse2 : Overlaps_C<8,1, uint16_t, uint8_t>;
        degs[8][1] = d->isse ? Degrain_sse2<radius, 8,1> : Degrain_C<radius, 8,1, uint8_t>;

        overs[8][2] = d->isse ? mvtools_Overlaps8x2_sse2 : Overlaps_C<8,2, uint16_t, uint8_t>;
        degs[8][2] = d->isse ? Degrain_sse2<radius, 8,2> : Degrain_C<radius, 8,2, uint8_t>;

        overs[8][4] = d->isse ? mvtools_Overlaps8x4_sse2 : Overlaps_C<8,4, uint16_t, uint8_t>;
        degs[8][4] = d->isse ? Degrain_sse2<radius, 8,4> : Degrain_C<radius, 8,4, uint8_t>;

        overs[8][8] = d->isse ? mvtools_Overlaps8x8_sse2 : Overlaps_C<8,8, uint16_t, uint8_t>;
        degs[8][8] = d->isse ? Degrain_sse2<radius, 8,8> : Degrain_C<radius, 8,8, uint8_t>;

        overs[8][16] = d->isse ? mvtools_Overlaps8x16_sse2 : Overlaps_C<8,16, uint16_t, uint8_t>;
        degs[8][16] = d->isse ? Degrain_sse2<radius, 8,16> : Degrain_C<radius, 8,16, uint8_t>;

        overs[16][1] = d->isse ? mvtools_Overlaps16x1_sse2 : Overlaps_C<16,1, uint16_t, uint8_t>;
        degs[16][1] = d->isse ? Degrain_sse2<radius, 16,1> : Degrain_C<radius, 16,1, uint8_t>;

        overs[16][2] = d->isse ? mvtools_Overlaps16x2_sse2 : Overlaps_C<16,2, uint16_t, uint8_t>;
        degs[16][2] = d->isse ? Degrain_sse2<radius, 16,2> : Degrain_C<radius, 16,2, uint8_t>;

        overs[16][4] = d->isse ? mvtools_Overlaps16x4_sse2 : Overlaps_C<16,4, uint16_t, uint8_t>;
        degs[16][4] = d->isse ? Degrain_sse2<radius, 16,4> : Degrain_C<radius, 16,4, uint8_t>;

        overs[16][8] = d->isse ? mvtools_Overlaps16x8_sse2 : Overlaps_C<16,8, uint16_t, uint8_t>;
        degs[16][8] = d->isse ? Degrain_sse2<radius, 16,8> : Degrain_C<radius, 16,8, uint8_t>;

        overs[16][16] = d->isse ? mvtools_Overlaps16x16_sse2 : Overlaps_C<16,16, uint16_t, uint8_t>;
        degs[16][16] = d->isse ? Degrain_sse2<radius, 16,16> : Degrain_C<radius, 16,16, uint8_t>;

        overs[16][32] = d->isse ? mvtools_Overlaps16x32_sse2 : Overlaps_C<16,32, uint16_t, uint8_t>;
        degs[16][32] = d->isse ? Degrain_sse2<radius, 16,32> : Degrain_C<radius, 16,32, uint8_t>;

        overs[32][8] = d->isse ? mvtools_Overlaps32x8_sse2 : Overlaps_C<32,8, uint16_t, uint8_t>;
        degs[32][8] = d->isse ? Degrain_sse2<radius, 32,8> : Degrain_C<radius, 32,8, uint8_t>;

        overs[32][16] = d->isse ? mvtools_Overlaps32x16_sse2 : Overlaps_C<32,16, uint16_t, uint8_t>;
        degs[32][16] = d->isse ? Degrain_sse2<radius, 32,16> : Degrain_C<radius, 32,16, uint8_t>;

        overs[32][32] = d->isse ? mvtools_Overlaps32x32_sse2 : Overlaps_C<32,32, uint16_t, uint8_t>;
        degs[32][32] = d->isse ? Degrain_sse2<radius, 32,32> : Degrain_C<radius, 32,32, uint8_t>;

        d->LimitChanges = d->isse ? mvtools_LimitChanges_sse2 : LimitChanges_C<uint8_t>;

        d->ToPixels = ToPixels<uint16_t, uint8_t>;
    } else {
        overs[2][2] = Overlaps_C<2,2, uint32_t, uint16_t>;
        degs[2][2] = Degrain_C<radius, 2,2, uint16_t>;

        overs[2][4] = Overlaps_C<2,4, uint32_t, uint16_t>;
        degs[2][4] = Degrain_C<radius, 2,4, uint16_t>;

        overs[4][2] = Overlaps_C<4,2, uint32_t, uint16_t>;
        degs[4][2] = Degrain_C<radius, 4,2, uint16_t>;

        overs[4][4] = Overlaps_C<4,4, uint32_t, uint16_t>;
        degs[4][4] = Degrain_C<radius, 4,4, uint16_t>;

        overs[4][8] = Overlaps_C<4,8, uint32_t, uint16_t>;
        degs[4][8] = Degrain_C<radius, 4,8, uint16_t>;

        overs[8][1] = Overlaps_C<8,1, uint32_t, uint16_t>;
        degs[8][1] = Degrain_C<radius, 8,1, uint16_t>;

        overs[8][2] = Overlaps_C<8,2, uint32_t, uint16_t>;
        degs[8][2] = Degrain_C<radius, 8,2, uint16_t>;

        overs[8][4] = Overlaps_C<8,4, uint32_t, uint16_t>;
        degs[8][4] = Degrain_C<radius, 8,4, uint16_t>;

        overs[8][8] = Overlaps_C<8,8, uint32_t, uint16_t>;
        degs[8][8] = Degrain_C<radius, 8,8, uint16_t>;

        overs[8][16] = Overlaps_C<8,16, uint32_t, uint16_t>;
        degs[8][16] = Degrain_C<radius, 8,16, uint16_t>;

        overs[16][1] = Overlaps_C<16,1, uint32_t, uint16_t>;
        degs[16][1] = Degrain_C<radius, 16,1, uint16_t>;

        overs[16][2] = Overlaps_C<16,2, uint32_t, uint16_t>;
        degs[16][2] = Degrain_C<radius, 16,2, uint16_t>;

        overs[16][4] = Overlaps_C<16,4, uint32_t, uint16_t>;
        degs[16][4] = Degrain_C<radius, 16,4, uint16_t>;

        overs[16][8] = Overlaps_C<16,8, uint32_t, uint16_t>;
        degs[16][8] = Degrain_C<radius, 16,8, uint16_t>;

        overs[16][16] = Overlaps_C<16,16, uint32_t, uint16_t>;
        degs[16][16] = Degrain_C<radius, 16,16, uint16_t>;

        overs[16][32] = Overlaps_C<16,32, uint32_t, uint16_t>;
        degs[16][32] = Degrain_C<radius, 16,32, uint16_t>;

        overs[32][8] = Overlaps_C<32,8, uint32_t, uint16_t>;
        degs[32][8] = Degrain_C<radius, 32,8, uint16_t>;

        overs[32][16] = Overlaps_C<32,16, uint32_t, uint16_t>;
        degs[32][16] = Degrain_C<radius, 32,16, uint16_t>;

        overs[32][32] = Overlaps_C<32,32, uint32_t, uint16_t>;
        degs[32][32] = Degrain_C<radius, 32,32, uint16_t>;

        d->LimitChanges = LimitChanges_C<uint16_t>;

        d->ToPixels = ToPixels<uint32_t, uint16_t>;
    }

    d->OVERS[0] = overs[nBlkSizeX][nBlkSizeY];
    d->DEGRAIN[0] = degs[nBlkSizeX][nBlkSizeY];

    d->OVERS[1] = d->OVERS[2] = overs[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
    d->DEGRAIN[1] = d->DEGRAIN[2] = degs[nBlkSizeX / xRatioUV][nBlkSizeY / yRatioUV];
}


template <int radius>
static void VS_CC mvdegrainCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::string filter = "Degrain";
    filter.append(std::to_string(radius));

    MVDegrainData d;
    MVDegrainData *data;

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
        vsapi->setError(out, (filter + ": plane must be between 0 and 4 (inclusive).").c_str());
        return;
    }

    int planes[5] = { YPLANE, UPLANE, VPLANE, UVPLANES, YUVPLANES };
    d.YUVplanes = planes[plane];


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, (filter + ": failed to retrieve first frame from super clip. Error message: " + errorMsg).c_str());
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
            vsapi->setError(out, (filter + ": required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?").c_str());
            vsapi->freeNode(d.super);
            return;
        }


    d.vectors[Backward1] = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.vectors[Forward1] = vsapi->propGetNode(in, "mvfw", 0, NULL);
    d.vectors[Backward2] = vsapi->propGetNode(in, "mvbw2", 0, &err);
    d.vectors[Forward2] = vsapi->propGetNode(in, "mvfw2", 0, &err);
    d.vectors[Backward3] = vsapi->propGetNode(in, "mvbw3", 0, &err);
    d.vectors[Forward3] = vsapi->propGetNode(in, "mvfw3", 0, &err);

    // XXX Yoda had the right idea.

    // Loops are nice.
    for (int r = 0; r < radius*2; r++) {
        try {
            d.mvClips[r] = new MVClipDicks(d.vectors[r], d.nSCD1, d.nSCD2, vsapi);
        } catch (MVException &e) {
            vsapi->setError(out, (filter + ": " + e.what()).c_str());

            vsapi->freeNode(d.super);

            for (int rr = 0; rr < radius*2; rr++)
                vsapi->freeNode(d.vectors[rr]);

            for (int rr = 0; rr < r; rr++)
                delete d.mvClips[rr];

            return;
        }
    }


    try {
        // Make sure the motion vector clips are correct.
        if (!d.mvClips[Backward1]->IsBackward())
            throw MVException("mvbw must be generated with isb=True.");
        if (d.mvClips[Forward1]->IsBackward())
            throw MVException("mvfw must be generated with isb=False.");
        if (radius > 1) {
            if (!d.mvClips[Backward2]->IsBackward())
                throw MVException("mvbw2 must be generated with isb=True.");
            if (d.mvClips[Forward2]->IsBackward())
                throw MVException("mvfw2 must be generated with isb=False.");

            if (d.mvClips[Backward2]->GetDeltaFrame() <= d.mvClips[Backward1]->GetDeltaFrame())
                throw MVException("mvbw2 must have greater delta than mvbw.");
            if (d.mvClips[Forward2]->GetDeltaFrame() <= d.mvClips[Forward1]->GetDeltaFrame())
                throw MVException("mvfw2 must have greater delta than mvfw.");
        }
        if (radius > 2) {
            if (!d.mvClips[Backward3]->IsBackward())
                throw MVException("mvbw3 must be generated with isb=True.");
            if (d.mvClips[Forward3]->IsBackward())
                throw MVException("mvfw3 must be generated with isb=False.");

            if (d.mvClips[Backward3]->GetDeltaFrame() <= d.mvClips[Backward2]->GetDeltaFrame())
                throw MVException("mvbw3 must have greater delta than mvbw2.");
            if (d.mvClips[Forward3]->GetDeltaFrame() <= d.mvClips[Forward2]->GetDeltaFrame())
                throw MVException("mvfw3 must have greater delta than mvfw2.");
        }

        d.bleh = new MVFilter(d.vectors[Forward1], filter.c_str(), vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, (filter + ": " + e.what()).c_str());
        vsapi->freeNode(d.super);

        for (int r = 0; r < radius*2; r++) {
            vsapi->freeNode(d.vectors[r]);
            delete d.mvClips[r];
        }
        return;
    }


    try {
        const char *vectorNames[6] = { "mvbw", "mvfw", "mvbw2", "mvfw2", "mvbw3", "mvfw3" };

        for (int r = 0; r < radius*2; r++)
            d.bleh->CheckSimilarity(d.mvClips[r], vectorNames[r]);
    } catch (MVException &e) {
        vsapi->setError(out, (filter + ": " + e.what()).c_str());

        vsapi->freeNode(d.super);

        for (int r = 0; r < radius*2; r++) {
            vsapi->freeNode(d.vectors[r]);
            delete d.mvClips[r];
        }

        delete d.bleh;
        return;
    }

    d.thSAD[0] = d.thSAD[0] * d.mvClips[Backward1]->GetThSCD1() / d.nSCD1; // normalize to block SAD
    d.thSAD[1] = d.thSAD[2] = d.thSAD[1] * d.mvClips[Backward1]->GetThSCD1() / d.nSCD1; // chroma threshold, normalized to block SAD



    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.bleh->nHeight != nHeightS || d.bleh->nHeight != d.vi->height || d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2 || d.bleh->nWidth != d.vi->width) {
        vsapi->setError(out, (filter + ": wrong source or super clip frame size.").c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius*2; r++) {
            vsapi->freeNode(d.vectors[r]);
            delete d.mvClips[r];
        }

        delete d.bleh;
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, (filter + ": input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.").c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius*2; r++) {
            vsapi->freeNode(d.vectors[r]);
            delete d.mvClips[r];
        }

        delete d.bleh;
        return;
    }

    if (d.vi->format->bitsPerSample > 8)
        d.isse = 0;

    int pixelMax = (1 << d.vi->format->bitsPerSample) - 1;

    d.nLimit[0] = vsapi->propGetInt(in, "limit", 0, &err);
    if (err)
        d.nLimit[0] = pixelMax;

    d.nLimit[1] = d.nLimit[2] = vsapi->propGetInt(in, "limitc", 0, &err);
    if (err)
        d.nLimit[1] = d.nLimit[2] = d.nLimit[0];

    if (d.nLimit[0] < 0 || d.nLimit[0] > pixelMax) {
        vsapi->setError(out, (filter + ": limit must be between 0 and " + std::to_string(pixelMax) + " (inclusive).").c_str());

        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius*2; r++) {
            vsapi->freeNode(d.vectors[r]);
            delete d.mvClips[r];
        }

        delete d.bleh;

        return;
    }

    if (d.nLimit[1] < 0 || d.nLimit[1] > pixelMax) {
        vsapi->setError(out, (filter + ": limitc must be between 0 and " + std::to_string(pixelMax) + " (inclusive).").c_str());

        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius*2; r++) {
            vsapi->freeNode(d.vectors[r]);
            delete d.mvClips[r];
        }

        delete d.bleh;

        return;
    }


    d.dstTempPitch = ((d.bleh->nWidth + 15)/16)*16 * d.vi->format->bytesPerSample * 2;

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


    selectFunctions<radius>(&d);


    data = (MVDegrainData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, filter.c_str(), mvdegrainInit, mvdegrainGetFrame<radius>, mvdegrainFree<radius>, fmParallel, 0, data, core);
}


void mvdegrainsRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
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
            , mvdegrainCreate<1>, 0, plugin);
    registerFunc("Degrain2",
            "clip:clip;"
            "super:clip;"
            "mvbw:clip;"
            "mvfw:clip;"
            "mvbw2:clip;"
            "mvfw2:clip;"
            "thsad:int:opt;"
            "thsadc:int:opt;"
            "plane:int:opt;"
            "limit:int:opt;"
            "limitc:int:opt;"
            "thscd1:int:opt;"
            "thscd2:int:opt;"
            "isse:int:opt;"
            , mvdegrainCreate<2>, 0, plugin);
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
            , mvdegrainCreate<3>, 0, plugin);
}
