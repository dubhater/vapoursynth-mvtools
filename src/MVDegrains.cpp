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

#include <limits.h>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "Bullshit.h"
#include "CPU.h"
#include "Fakery.h"
#include "MVAnalysisData.h"
#include "MVDegrains.h"
#include "MVFrame.h"
#include "Overlap.h"

struct MVDegrainData {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *super;
    VSNodeRef *vectors[12];

    int64_t thSAD[3];
    int YUVplanes;
    int nLimit[3];
    int64_t nSCD1;
    int nSCD2;
    int opt;

    MVAnalysisData vectors_data[12];

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

    int process[3];

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

    OverlapWindows *OverWins[3];
};


static void VS_CC mvdegrainInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVDegrainData *d = (MVDegrainData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


template <int radius>
static const VSFrameRef *VS_CC mvdegrainGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVDegrainData *d = (MVDegrainData *)*instanceData;

    if (activationReason == arInitial) {

        for (int r = 0; r < radius * 2; r += 2) {
            //Backward
            vsapi->requestFrameFilter(n, d->vectors[r], frameCtx);
            //Forward
            vsapi->requestFrameFilter(n, d->vectors[r + 1], frameCtx);

            // Backward
            int offB = d->vectors_data[r].nDeltaFrame;
            if (n + offB < d->vi->numFrames)
                vsapi->requestFrameFilter(n + offB, d->super, frameCtx);

            // Forward
            int offF = -1 * d->vectors_data[r + 1].nDeltaFrame;
            if (n + offF >= 0)
                vsapi->requestFrameFilter(n + offF, d->super, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

        int bitsPerSample = d->vi->format->bitsPerSample;
        int bytesPerSample = d->vi->format->bytesPerSample;

        uint8_t *pDst[3] = { 0 };
        uint8_t *pDstCur[3] = { 0 };
        const uint8_t *pSrcCur[3] = { NULL };
        const uint8_t *pSrc[3] = { NULL };
        const uint8_t *pRefs[radius * 2][3] = { { NULL } };
        int nDstPitches[3] = { 0 };
        int nSrcPitches[3] = { 0 };
        int nRefPitches[radius * 2][3] = { { 0 } };
        int isUsable[radius * 2];
        int nLogPel = (d->vectors_data[0].nPel == 4) ? 2 : (d->vectors_data[0].nPel == 2) ? 1 : 0;

        FakeGroupOfPlanes fgops[radius * 2];
        const VSFrameRef *refFrames[radius * 2] = { 0 };

        for (int r = 0; r < radius * 2; r++) {
            const VSFrameRef *frame = vsapi->getFrameFilter(n, d->vectors[r], frameCtx);
            fgopInit(&fgops[r], &d->vectors_data[r]);
            const VSMap *mvprops = vsapi->getFramePropsRO(frame);
            fgopUpdate(&fgops[r], (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            isUsable[r] = fgopIsUsable(&fgops[r], d->nSCD1, d->nSCD2);
            vsapi->freeFrame(frame);

            if (isUsable[r]) {
                int offset = d->vectors_data[r].nDeltaFrame * (d->vectors_data[r].isBackward ? 1 : -1);
                refFrames[r] = vsapi->getFrameFilter(n + offset, d->super, frameCtx);
            }
        }


        for (int i = 0; i < d->vi->format->numPlanes; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
            pSrc[i] = vsapi->getReadPtr(src, i);
            nSrcPitches[i] = vsapi->getStride(src, i);

            for (int r = 0; r < radius * 2; r++)
                if (isUsable[r]) {
                    pRefs[r][i] = vsapi->getReadPtr(refFrames[r], i);
                    nRefPitches[r][i] = vsapi->getStride(refFrames[r], i);
                }
        }

        const int xSubUV = d->xSubUV;
        const int ySubUV = d->ySubUV;
        const int xRatioUV = d->vectors_data[0].xRatioUV;
        const int yRatioUV = d->vectors_data[0].yRatioUV;
        const int nBlkX = d->vectors_data[0].nBlkX;
        const int nBlkY = d->vectors_data[0].nBlkY;
        const int opt = d->opt;
        const int dstTempPitch = d->dstTempPitch;
        const int *nWidth = d->nWidth;
        const int *nHeight = d->nHeight;
        const int *nOverlapX = d->nOverlapX;
        const int *nOverlapY = d->nOverlapY;
        const int *nBlkSizeX = d->nBlkSizeX;
        const int *nBlkSizeY = d->nBlkSizeY;
        const int *nWidth_B = d->nWidth_B;
        const int *nHeight_B = d->nHeight_B;
        const int64_t *thSAD = d->thSAD;
        const int *nLimit = d->nLimit;


        MVGroupOfFrames pRefGOF[radius * 2];
        for (int r = 0; r < radius * 2; r++)
            mvgofInit(&pRefGOF[r], d->nSuperLevels, nWidth[0], nHeight[0], d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, opt, xRatioUV, yRatioUV, bitsPerSample);


        OverlapWindows *OverWins[3] = { d->OverWins[0], d->OverWins[1], d->OverWins[2] };
        uint8_t *DstTemp = NULL;
        int tmpBlockPitch = nBlkSizeX[0] * bytesPerSample;
        uint8_t *tmpBlock = NULL;
        if (nOverlapX[0] > 0 || nOverlapY[0] > 0) {
            DstTemp = new uint8_t[dstTempPitch * nHeight[0]];
            tmpBlock = new uint8_t[tmpBlockPitch * nBlkSizeY[0]];
        }

        MVPlane **pPlanes[radius * 2] = { NULL };

        for (int r = 0; r < radius * 2; r++)
            if (isUsable[r]) {
                mvgofUpdate(&pRefGOF[r], (uint8_t **)pRefs[r], nRefPitches[r]);
                pPlanes[r] = pRefGOF[r].frames[0]->planes;
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

                        const uint8_t *pointers[radius * 2]; // Moved by the degrain function.
                        int strides[radius * 2];

                        int WSrc, WRefs[radius * 2];

                        for (int r = 0; r < radius * 2; r++)
                            useBlock(pointers[r], strides[r], WRefs[r], isUsable[r], &fgops[r], i, pPlanes[r], pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

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
            } else { // overlap
                uint8_t *pDstTemp = DstTemp;
                memset(pDstTemp, 0, dstTempPitch * nHeight_B[0]);

                for (int by = 0; by < nBlkY; by++) {
                    int wby = ((by + nBlkY - 3) / (nBlkY - 2)) * 3;
                    int wbx = 0;
                    int xx = 0;
                    for (int bx = 0; bx < nBlkX; bx++) {
                        // select window
                        wbx = bx == nBlkX - 1 ? 2 : wbx; //(bx + nBlkX - 3) / (nBlkX - 2);
                        int16_t *winOver = overGetWindow(OverWins[plane], wby + wbx);

                        int i = by * nBlkX + bx;

                        const uint8_t *pointers[radius * 2]; // Moved by the degrain function.
                        int strides[radius * 2];

                        int WSrc, WRefs[radius * 2];

                        for (int r = 0; r < radius * 2; r++)
                            useBlock(pointers[r], strides[r], WRefs[r], isUsable[r], &fgops[r], i, pPlanes[r], pSrcCur, xx, nSrcPitches, nLogPel, plane, xSubUV, ySubUV, thSAD);

                        normaliseWeights<radius>(WSrc, WRefs);

                        d->DEGRAIN[plane](tmpBlock, tmpBlockPitch, pSrcCur[plane] + xx, nSrcPitches[plane],
                                          pointers, strides,
                                          WSrc, WRefs);
                        d->OVERS[plane](pDstTemp + xx * 2, dstTempPitch, tmpBlock, tmpBlockPitch, winOver, nBlkSizeX[plane]);

                        xx += (nBlkSizeX[plane] - nOverlapX[plane]) * bytesPerSample;
                        wbx = 1;
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

        if (DstTemp)
            delete[] DstTemp;

        for (int r = 0; r < radius * 2; r++) {
            mvgofDeinit(&pRefGOF[r]);

            if (refFrames[r])
                vsapi->freeFrame(refFrames[r]);

            fgopDeinit(&fgops[r]);
        }

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


template <int radius>
static void VS_CC mvdegrainFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVDegrainData *d = (MVDegrainData *)instanceData;

    if (d->nOverlapX[0] || d->nOverlapY[0]) {
        overDeinit(d->OverWins[0]);
        free(d->OverWins[0]);
        if (d->vi->format->colorFamily != cmGray) {
            overDeinit(d->OverWins[1]);
            free(d->OverWins[1]);
        }
    }
    for (int r = 0; r < radius * 2; r++)
        vsapi->freeNode(d->vectors[r]);

    vsapi->freeNode(d->super);
    vsapi->freeNode(d->node);

    free(d);
}


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86)
#define DEGRAIN_SSE2(radius, width, height) \
    { KEY(width, height, 8, MVOPT_SSE2), Degrain_sse2<radius, width, height> },

#define DEGRAIN_LEVEL_SSE2(radius)\
    {\
        DEGRAIN_SSE2(radius, 4, 2)\
        DEGRAIN_SSE2(radius, 4, 4)\
        DEGRAIN_SSE2(radius, 4, 8)\
        DEGRAIN_SSE2(radius, 8, 1)\
        DEGRAIN_SSE2(radius, 8, 2)\
        DEGRAIN_SSE2(radius, 8, 4)\
        DEGRAIN_SSE2(radius, 8, 8)\
        DEGRAIN_SSE2(radius, 8, 16)\
        DEGRAIN_SSE2(radius, 16, 1)\
        DEGRAIN_SSE2(radius, 16, 2)\
        DEGRAIN_SSE2(radius, 16, 4)\
        DEGRAIN_SSE2(radius, 16, 8)\
        DEGRAIN_SSE2(radius, 16, 16)\
        DEGRAIN_SSE2(radius, 16, 32)\
        DEGRAIN_SSE2(radius, 32, 8)\
        DEGRAIN_SSE2(radius, 32, 16)\
        DEGRAIN_SSE2(radius, 32, 32)\
        DEGRAIN_SSE2(radius, 32, 64)\
        DEGRAIN_SSE2(radius, 64, 16)\
        DEGRAIN_SSE2(radius, 64, 32)\
        DEGRAIN_SSE2(radius, 64, 64)\
        DEGRAIN_SSE2(radius, 64, 128)\
        DEGRAIN_SSE2(radius, 128, 32)\
        DEGRAIN_SSE2(radius, 128, 64)\
        DEGRAIN_SSE2(radius, 128, 128)\
    }
#else
#define DEGRAIN_SSE2(radius, width, height)
#define DEGRAIN_LEVEL_SSE2(radius)
#endif

#define DEGRAIN(radius, width, height) \
    { KEY(width, height, 8, MVOPT_SCALAR), Degrain_C<radius, width, height, uint8_t> }, \
    { KEY(width, height, 16, MVOPT_SCALAR), Degrain_C<radius, width, height, uint16_t> },

#define DEGRAIN_LEVEL(radius)\
    {\
        DEGRAIN(radius, 2, 2)\
        DEGRAIN(radius, 2, 4)\
        DEGRAIN(radius, 4, 2)\
        DEGRAIN(radius, 4, 4)\
        DEGRAIN(radius, 4, 8)\
        DEGRAIN(radius, 8, 1)\
        DEGRAIN(radius, 8, 2)\
        DEGRAIN(radius, 8, 4)\
        DEGRAIN(radius, 8, 8)\
        DEGRAIN(radius, 8, 16)\
        DEGRAIN(radius, 16, 1)\
        DEGRAIN(radius, 16, 2)\
        DEGRAIN(radius, 16, 4)\
        DEGRAIN(radius, 16, 8)\
        DEGRAIN(radius, 16, 16)\
        DEGRAIN(radius, 16, 32)\
        DEGRAIN(radius, 32, 8)\
        DEGRAIN(radius, 32, 16)\
        DEGRAIN(radius, 32, 32)\
        DEGRAIN(radius, 32, 64)\
        DEGRAIN(radius, 64, 16)\
        DEGRAIN(radius, 64, 32)\
        DEGRAIN(radius, 64, 64)\
        DEGRAIN(radius, 64, 128)\
        DEGRAIN(radius, 128, 32)\
        DEGRAIN(radius, 128, 64)\
        DEGRAIN(radius, 128, 128)\
    }

static const std::unordered_map<uint32_t, DenoiseFunction> degrain_functions[6] = {
    DEGRAIN_LEVEL(1),
    DEGRAIN_LEVEL(2),
    DEGRAIN_LEVEL(3),
    DEGRAIN_LEVEL(4),
    DEGRAIN_LEVEL(5),
    DEGRAIN_LEVEL(6),
};

static const std::unordered_map<uint32_t, DenoiseFunction> degrain_functions_sse2[6] = {
    DEGRAIN_LEVEL_SSE2(1),
    DEGRAIN_LEVEL_SSE2(2),
    DEGRAIN_LEVEL_SSE2(3),
    DEGRAIN_LEVEL_SSE2(4),
    DEGRAIN_LEVEL_SSE2(5),
    DEGRAIN_LEVEL_SSE2(6),
};

static DenoiseFunction selectDegrainFunction(unsigned radius, unsigned width, unsigned height, unsigned bits, int opt) {
    DenoiseFunction degrain = degrain_functions[radius - 1].at(KEY(width, height, bits, MVOPT_SCALAR));

#if defined(MVTOOLS_X86)
    if (opt) {
        try {
            degrain = degrain_functions_sse2[radius - 1].at(KEY(width, height, bits, MVOPT_SSE2));
        } catch (std::out_of_range &) { }

        if (g_cpuinfo & X264_CPU_AVX2) {
            DenoiseFunction tmp = selectDegrainFunctionAVX2(radius, width, height, bits);
            if (tmp)
                degrain = tmp;
        }
    }
#endif

    return degrain;
}

#undef DEGRAIN
#undef DEGRAIN_SSE2
#undef DEGRAIN_LEVEL
#undef DEGRAIN_LEVEL_SSE2

#undef KEY


template <int radius>
static void selectFunctions(MVDegrainData *d) {
    const unsigned xRatioUV = d->vectors_data[0].xRatioUV;
    const unsigned yRatioUV = d->vectors_data[0].yRatioUV;
    const unsigned nBlkSizeX = d->vectors_data[0].nBlkSizeX;
    const unsigned nBlkSizeY = d->vectors_data[0].nBlkSizeY;
    const unsigned bits = d->vi->format->bytesPerSample * 8;

    if (d->vi->format->bitsPerSample == 8) {
        d->LimitChanges = LimitChanges_C<uint8_t>;

        d->ToPixels = ToPixels_uint16_t_uint8_t;

#if defined(MVTOOLS_X86)
        if (d->opt) {
            d->LimitChanges = LimitChanges_sse2;
        }
#endif
    } else {
        d->LimitChanges = LimitChanges_C<uint16_t>;

        d->ToPixels = ToPixels_uint32_t_uint16_t;
    }

    d->OVERS[0] = selectOverlapsFunction(nBlkSizeX, nBlkSizeY, bits, d->opt);
    d->DEGRAIN[0] = selectDegrainFunction(radius, nBlkSizeX, nBlkSizeY, bits, d->opt);

    d->OVERS[1] = d->OVERS[2] = selectOverlapsFunction(nBlkSizeX / xRatioUV, nBlkSizeY / yRatioUV, bits, d->opt);
    d->DEGRAIN[1] = d->DEGRAIN[2] = selectDegrainFunction(radius, nBlkSizeX / xRatioUV, nBlkSizeY / yRatioUV, bits, d->opt);
}


template <int radius>
static void VS_CC mvdegrainCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

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

    int plane = int64ToIntS(vsapi->propGetInt(in, "plane", 0, &err));
    if (err)
        plane = 4;

    d.nSCD1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.nSCD1 = MV_DEFAULT_SCD1;

    d.nSCD2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.nSCD2 = MV_DEFAULT_SCD2;

    d.opt = !!vsapi->propGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;


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
    int nHeightS = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    d.nSuperVPad = int64ToIntS(vsapi->propGetInt(props, "Super_vpad", 0, &evil_err[2]));
    d.nSuperPel = int64ToIntS(vsapi->propGetInt(props, "Super_pel", 0, &evil_err[3]));
    d.nSuperModeYUV = int64ToIntS(vsapi->propGetInt(props, "Super_modeyuv", 0, &evil_err[4]));
    d.nSuperLevels = int64ToIntS(vsapi->propGetInt(props, "Super_levels", 0, &evil_err[5]));
    vsapi->freeFrame(evil);

    for (int i = 0; i < 6; i++)
        if (evil_err[i]) {
            vsapi->setError(out, (filter + ": required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?").c_str());
            vsapi->freeNode(d.super);
            return;
        }


#define ERROR_SIZE 512

    char error[ERROR_SIZE + 1] = { 0 };

    const char *vector_names[] = { "mvbw", "mvfw", "mvbw2", "mvfw2", "mvbw3", "mvfw3", "mvbw4", "mvfw4", "mvbw5", "mvfw5", "mvbw6", "mvfw6"};

    for (int r = 0; r < radius * 2; r++) {
        d.vectors[r] = vsapi->propGetNode(in, vector_names[r], 0, NULL);

        adataFromVectorClip(&d.vectors_data[r], d.vectors[r], filter.c_str(), vector_names[r], vsapi, error, ERROR_SIZE);
    }

    int64_t nSCD1_old = d.nSCD1;
    scaleThSCD(&d.nSCD1, &d.nSCD2, &d.vectors_data[0], filter.c_str(), error, ERROR_SIZE);

    for (int r = 1; r < radius * 2; r++)
        adataCheckSimilarity(&d.vectors_data[0], &d.vectors_data[r], filter.c_str(), vector_names[0], vector_names[r], error, ERROR_SIZE);

    if (error[0]) {
        vsapi->setError(out, error);

        vsapi->freeNode(d.super);
        for (int r = 0; r < radius * 2; r++)
            vsapi->freeNode(d.vectors[r]);

        return;
    }


    error[0] = '\0';


    for (int r = 0; r < radius * 2; r++)
        if (d.vectors_data[r].nDeltaFrame <= 0)
            snprintf(error, ERROR_SIZE, "%s", "cannot use motion vectors with absolute frame references.");

#define CHECK_VECTORS(rThreshold, backwardN, forwardN, backwardP, forwardP, mvbwN, mvfwN, mvbwP, mvfwP)\
    if (radius > rThreshold) {\
        if (!d.vectors_data[backwardN].isBackward)\
            snprintf(error, ERROR_SIZE, "%s", "mvbw must be generated with isb=True.");\
        if (d.vectors_data[forwardN].isBackward)\
            snprintf(error, ERROR_SIZE, "%s", "mvfw must be generated with isb=False.");\
        if (d.vectors_data[backwardN].nDeltaFrame <= d.vectors_data[backwardP].nDeltaFrame)\
            snprintf(error, ERROR_SIZE, "%s", "mvbwN must have greater delta than mvbwP.");\
        if (d.vectors_data[forwardN].nDeltaFrame <= d.vectors_data[forwardP].nDeltaFrame)\
            snprintf(error, ERROR_SIZE, "%s", "mvfwN must have greater delta than mvfwP.");\
    }

    // Make sure the motion vector clips are correct.
    if (!d.vectors_data[Backward1].isBackward)
        snprintf(error, ERROR_SIZE, "%s", "mvbw must be generated with isb=True.");
    if (d.vectors_data[Forward1].isBackward)
        snprintf(error, ERROR_SIZE, "%s", "mvfw must be generated with isb=False.");

    CHECK_VECTORS(1, Backward2, Forward2, Backward1, Forward1, mvbw2, mvfw2, mvbw, mvfw)
    CHECK_VECTORS(2, Backward3, Forward3, Backward2, Forward2, mvbw3, mvfw3, mvbw2, mvfw2)
    CHECK_VECTORS(3, Backward4, Forward4, Backward3, Forward3, mvbw4, mvfw4, mvbw3, mvfw3)
    CHECK_VECTORS(4, Backward5, Forward5, Backward4, Forward4, mvbw5, mvfw5, mvbw4, mvfw4)
    CHECK_VECTORS(5, Backward6, Forward6, Backward5, Forward5, mvbw6, mvfw6, mvbw5, mvfw5)

#undef CHECK_VECTORS
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->setError(out, (filter + ": " + error).c_str());
        vsapi->freeNode(d.super);

        for (int r = 0; r < radius * 2; r++)
            vsapi->freeNode(d.vectors[r]);

        return;
    }


    d.thSAD[0] = d.thSAD[0] * d.nSCD1 / nSCD1_old;              // normalize to block SAD
    d.thSAD[1] = d.thSAD[2] = d.thSAD[1] * d.nSCD1 / nSCD1_old; // chroma threshold, normalized to block SAD

    if (d.thSAD[0] >= INT_MAX || d.thSAD[1] >= INT_MAX) {
        int64_t maximum = INT_MAX * nSCD1_old / d.nSCD1;

        bool c = d.thSAD[0] < INT_MAX;

        vsapi->setError(out, (filter + ": with this block size and video format, thsad" + (c ? "c" : "") + " must not exceed " + std::to_string(maximum) + " or some calculations would overflow.").c_str());
        vsapi->freeNode(d.super);

        for (int r = 0; r < radius * 2; r++)
            vsapi->freeNode(d.vectors[r]);

        return;
    }


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.vectors_data[0].nHeight != nHeightS || d.vectors_data[0].nHeight != d.vi->height || d.vectors_data[0].nWidth != nSuperWidth - d.nSuperHPad * 2 || d.vectors_data[0].nWidth != d.vi->width || d.vectors_data[0].nPel != d.nSuperPel) {
        vsapi->setError(out, (filter + ": wrong source or super clip frame size.").c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius * 2; r++)
            vsapi->freeNode(d.vectors[r]);

        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, (filter + ": input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.").c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius * 2; r++)
            vsapi->freeNode(d.vectors[r]);

        return;
    }

    int pixelMax = (1 << d.vi->format->bitsPerSample) - 1;

    d.nLimit[0] = int64ToIntS(vsapi->propGetInt(in, "limit", 0, &err));
    if (err)
        d.nLimit[0] = pixelMax;

    d.nLimit[1] = d.nLimit[2] = int64ToIntS(vsapi->propGetInt(in, "limitc", 0, &err));
    if (err)
        d.nLimit[1] = d.nLimit[2] = d.nLimit[0];

    if (d.nLimit[0] < 0 || d.nLimit[0] > pixelMax) {
        vsapi->setError(out, (filter + ": limit must be between 0 and " + std::to_string(pixelMax) + " (inclusive).").c_str());

        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius * 2; r++)
            vsapi->freeNode(d.vectors[r]);

        return;
    }

    if (d.nLimit[1] < 0 || d.nLimit[1] > pixelMax) {
        vsapi->setError(out, (filter + ": limitc must be between 0 and " + std::to_string(pixelMax) + " (inclusive).").c_str());

        vsapi->freeNode(d.super);
        vsapi->freeNode(d.node);

        for (int r = 0; r < radius * 2; r++)
            vsapi->freeNode(d.vectors[r]);

        return;
    }


    d.dstTempPitch = ((d.vectors_data[0].nWidth + 15) / 16) * 16 * d.vi->format->bytesPerSample * 2;

    d.process[0] = !!(d.YUVplanes & YPLANE);
    d.process[1] = !!(d.YUVplanes & UPLANE & d.nSuperModeYUV);
    d.process[2] = !!(d.YUVplanes & VPLANE & d.nSuperModeYUV);

    d.xSubUV = d.vi->format->subSamplingW;
    d.ySubUV = d.vi->format->subSamplingH;

    d.nWidth[0] = d.vectors_data[0].nWidth;
    d.nWidth[1] = d.nWidth[2] = d.nWidth[0] >> d.xSubUV;

    d.nHeight[0] = d.vectors_data[0].nHeight;
    d.nHeight[1] = d.nHeight[2] = d.nHeight[0] >> d.ySubUV;

    d.nOverlapX[0] = d.vectors_data[0].nOverlapX;
    d.nOverlapX[1] = d.nOverlapX[2] = d.nOverlapX[0] >> d.xSubUV;

    d.nOverlapY[0] = d.vectors_data[0].nOverlapY;
    d.nOverlapY[1] = d.nOverlapY[2] = d.nOverlapY[0] >> d.ySubUV;

    d.nBlkSizeX[0] = d.vectors_data[0].nBlkSizeX;
    d.nBlkSizeX[1] = d.nBlkSizeX[2] = d.nBlkSizeX[0] >> d.xSubUV;

    d.nBlkSizeY[0] = d.vectors_data[0].nBlkSizeY;
    d.nBlkSizeY[1] = d.nBlkSizeY[2] = d.nBlkSizeY[0] >> d.ySubUV;

    d.nWidth_B[0] = d.vectors_data[0].nBlkX * (d.nBlkSizeX[0] - d.nOverlapX[0]) + d.nOverlapX[0];
    d.nWidth_B[1] = d.nWidth_B[2] = d.nWidth_B[0] >> d.xSubUV;

    d.nHeight_B[0] = d.vectors_data[0].nBlkY * (d.nBlkSizeY[0] - d.nOverlapY[0]) + d.nOverlapY[0];
    d.nHeight_B[1] = d.nHeight_B[2] = d.nHeight_B[0] >> d.ySubUV;

    if (d.nOverlapX[0] || d.nOverlapY[0]) {
        d.OverWins[0] = (OverlapWindows *)malloc(sizeof(OverlapWindows));
        overInit(d.OverWins[0], d.nBlkSizeX[0], d.nBlkSizeY[0], d.nOverlapX[0], d.nOverlapY[0]);

        if (d.vi->format->colorFamily != cmGray) {
            d.OverWins[1] = (OverlapWindows *)malloc(sizeof(OverlapWindows));
            overInit(d.OverWins[1], d.nBlkSizeX[1], d.nBlkSizeY[1], d.nOverlapX[1], d.nOverlapY[1]);

            d.OverWins[2] = d.OverWins[1];
        }
    }

    selectFunctions<radius>(&d);


    data = (MVDegrainData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, filter.c_str(), mvdegrainInit, mvdegrainGetFrame<radius>, mvdegrainFree<radius>, fmParallel, 0, data, core);
}


extern "C" void mvdegrainsRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
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
                 "opt:int:opt;",
                 mvdegrainCreate<1>, 0, plugin);
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
                 "opt:int:opt;",
                 mvdegrainCreate<2>, 0, plugin);
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
                 "opt:int:opt;",
                 mvdegrainCreate<3>, 0, plugin);
    registerFunc("Degrain4",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "mvbw2:clip;"
                 "mvfw2:clip;"
                 "mvbw3:clip;"
                 "mvfw3:clip;"
                 "mvbw4:clip;"
                 "mvfw4:clip;"
                 "thsad:int:opt;"
                 "thsadc:int:opt;"
                 "plane:int:opt;"
                 "limit:int:opt;"
                 "limitc:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "opt:int:opt;",
                 mvdegrainCreate<4>, 0, plugin);
    registerFunc("Degrain5",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "mvbw2:clip;"
                 "mvfw2:clip;"
                 "mvbw3:clip;"
                 "mvfw3:clip;"
                 "mvbw4:clip;"
                 "mvfw4:clip;"
                 "mvbw5:clip;"
                 "mvfw5:clip;"
                 "thsad:int:opt;"
                 "thsadc:int:opt;"
                 "plane:int:opt;"
                 "limit:int:opt;"
                 "limitc:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "opt:int:opt;",
                 mvdegrainCreate<5>, 0, plugin);
    registerFunc("Degrain6",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "mvbw2:clip;"
                 "mvfw2:clip;"
                 "mvbw3:clip;"
                 "mvfw3:clip;"
                 "mvbw4:clip;"
                 "mvfw4:clip;"
                 "mvbw5:clip;"
                 "mvfw5:clip;"
                 "mvbw6:clip;"
                 "mvfw6:clip;"
                 "thsad:int:opt;"
                 "thsadc:int:opt;"
                 "plane:int:opt;"
                 "limit:int:opt;"
                 "limitc:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "opt:int:opt;",
                 mvdegrainCreate<6>, 0, plugin);
}
