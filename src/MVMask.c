// Create an overlay mask with the motion vectors
// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - YUY2, occlusion
// See legal notice in Copying.txt for more information
//
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
#include <math.h>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "MaskFun.h"
#include "MVAnalysisData.h"
#include "SimpleResize.h"
#include "CommonMacros.h"



typedef struct MVMaskData {
    VSNode *node;
    VSVideoInfo vi;

    VSNode *vectors;
    float ml;
    float fGamma;
    int kind;
    int time256;
    int nSceneChangeValue;
    int64_t thscd1;
    int thscd2;
    int opt;

    float fMaskNormFactor;
    float fMaskNormFactor2;
    float fHalfGamma;

    int nWidthUV;
    int nHeightUV;
    int nWidthB;
    int nHeightB;
    int nWidthBUV;
    int nHeightBUV;

    MVAnalysisData vectors_data;

    SimpleResize upsizer;
    SimpleResize upsizerUV;
} MVMaskData;


static inline uint8_t mvmaskLength(VECTOR v, uint8_t pel, float fMaskNormFactor2, float fHalfGamma) {
    double norme = (double)(v.x * v.x + v.y * v.y) / (pel * pel);

    double l = 255 * pow(norme * fMaskNormFactor2, fHalfGamma); //Fizick - simply rewritten

    return (uint8_t)((l > 255) ? 255 : l);
}


static const VSFrame *VS_CC mvmaskGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVMaskData *d = (MVMaskData *)instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, src, core);

        const uint8_t *pSrc[3];
        uint8_t *pDst[3];
        int nDstPitches[3];
        int nSrcPitches[3];

        pSrc[0] = vsapi->getReadPtr(src, 0);
        nSrcPitches[0] = vsapi->getStride(src, 0);

        for (int i = 0; i < 3; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
        }

        FakeGroupOfPlanes fgop;
        const VSFrame *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        fgopInit(&fgop, &d->vectors_data);
        const VSMap *mvprops = vsapi->getFramePropertiesRO(mvn);
        fgopUpdate(&fgop, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));
        vsapi->freeFrame(mvn);

        const int kind = d->kind;
        const int nWidth = d->vectors_data.nWidth;
        const int nHeight = d->vectors_data.nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int nSceneChangeValue = d->nSceneChangeValue;

        if (fgopIsUsable(&fgop, d->thscd1, d->thscd2)) {
            const int nBlkX = d->vectors_data.nBlkX;
            const int nBlkY = d->vectors_data.nBlkY;
            const int nBlkCount = nBlkX * nBlkY;
            const float fMaskNormFactor = d->fMaskNormFactor;
            const float fMaskNormFactor2 = d->fMaskNormFactor2;
            const float fGamma = d->fGamma;
            const float fHalfGamma = d->fHalfGamma;
            const int nPel = d->vectors_data.nPel;
            const int nBlkSizeX = d->vectors_data.nBlkSizeX;
            const int nBlkSizeY = d->vectors_data.nBlkSizeY;
            const int nOverlapX = d->vectors_data.nOverlapX;
            const int nOverlapY = d->vectors_data.nOverlapY;
            const int nWidthB = d->nWidthB;
            const int nHeightB = d->nHeightB;
            const int nWidthBUV = d->nWidthBUV;
            const int nHeightBUV = d->nHeightBUV;
            SimpleResize *upsizer = &d->upsizer;
            SimpleResize *upsizerUV = &d->upsizerUV;
            const int time256 = d->time256;
            const int bitsPerSample = vsapi->getVideoFrameFormat(src)->bitsPerSample;

            uint8_t *smallMask = (uint8_t *)malloc(nBlkX * nBlkY);
            uint8_t *smallMaskV = (uint8_t *)malloc(nBlkX * nBlkY);

            if (kind == 0) { // vector length mask
                for (int j = 0; j < nBlkCount; j++)
                    smallMask[j] = mvmaskLength(fgopGetBlock(&fgop, 0, j)->vector, nPel, fMaskNormFactor2, fHalfGamma);
            } else if (kind == 1) { // SAD mask
                MakeSADMaskTime(&fgop, nBlkX, nBlkY, 4.0 * fMaskNormFactor / (nBlkSizeX * nBlkSizeY), fGamma, nPel, smallMask, nBlkX, time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY, bitsPerSample);
            } else if (kind == 2) { // occlusion mask
                MakeVectorOcclusionMaskTime(&fgop, d->vectors_data.isBackward, nBlkX, nBlkY, 1.0 / fMaskNormFactor, fGamma, nPel, smallMask, nBlkX, time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            } else if (kind == 3) { // vector x mask
                for (int j = 0; j < nBlkCount; j++)
                    smallMask[j] = VSMAX(0, VSMIN(255, (int)(fgopGetBlock(&fgop, 0, j)->vector.x * fMaskNormFactor * 100 + 128))); // shited by 128 for signed support
            } else if (kind == 4) { // vector y mask
                for (int j = 0; j < nBlkCount; j++)
                    smallMask[j] = VSMAX(0, VSMIN(255, (int)(fgopGetBlock(&fgop, 0, j)->vector.y * fMaskNormFactor * 100 + 128))); // shited by 128 for signed support
            } else if (kind == 5) {                                      // vector x mask in U, y mask in V
                for (int j = 0; j < nBlkCount; j++) {
                    VECTOR v = fgopGetBlock(&fgop, 0, j)->vector;
                    smallMask[j] = VSMAX(0, VSMIN(255, (int)(v.x * fMaskNormFactor * 100 + 128)));  // shited by 128 for signed support
                    smallMaskV[j] = VSMAX(0, VSMIN(255, (int)(v.y * fMaskNormFactor * 100 + 128))); // shited by 128 for signed support
                }
            }

            if (kind == 5) { // do not change luma for kind=5
                memcpy(pDst[0], pSrc[0], nSrcPitches[0] * nHeight);
            } else {
                upsizer->simpleResize_uint8_t(upsizer, pDst[0], nDstPitches[0], smallMask, nBlkX, 0);
                if (nWidth > nWidthB)
                    for (int h = 0; h < nHeight; h++)
                        for (int w = nWidthB; w < nWidth; w++)
                            *(pDst[0] + h * nDstPitches[0] + w) = *(pDst[0] + h * nDstPitches[0] + nWidthB - 1);
                if (nHeight > nHeightB)
                    vsh_bitblt(pDst[0] + nHeightB * nDstPitches[0], nDstPitches[0], pDst[0] + (nHeightB - 1) * nDstPitches[0], nDstPitches[0], nWidth, nHeight - nHeightB);
            }

            // chroma
            upsizerUV->simpleResize_uint8_t(upsizerUV, pDst[1], nDstPitches[1], smallMask, nBlkX, 0);

            if (kind == 5)
                upsizerUV->simpleResize_uint8_t(upsizerUV, pDst[2], nDstPitches[2], smallMaskV, nBlkX, 0);
            else
                memcpy(pDst[2], pDst[1], nHeightUV * nDstPitches[1]);

            if (nWidthUV > nWidthBUV)
                for (int h = 0; h < nHeightUV; h++)
                    for (int w = nWidthBUV; w < nWidthUV; w++) {
                        *(pDst[1] + h * nDstPitches[1] + w) = *(pDst[1] + h * nDstPitches[1] + nWidthBUV - 1);
                        *(pDst[2] + h * nDstPitches[2] + w) = *(pDst[2] + h * nDstPitches[2] + nWidthBUV - 1);
                    }
            if (nHeightUV > nHeightBUV) {
                vsh_bitblt(pDst[1] + nHeightBUV * nDstPitches[1], nDstPitches[1], pDst[1] + (nHeightBUV - 1) * nDstPitches[1], nDstPitches[1], nWidthUV, nHeightUV - nHeightBUV);
                vsh_bitblt(pDst[2] + nHeightBUV * nDstPitches[2], nDstPitches[2], pDst[2] + (nHeightBUV - 1) * nDstPitches[2], nDstPitches[2], nWidthUV, nHeightUV - nHeightBUV);
            }

            free(smallMask);
            free(smallMaskV);
        } else { // not usable
            if (kind == 5)
                memcpy(pDst[0], pSrc[0], nSrcPitches[0] * nHeight);
            else
                memset(pDst[0], nSceneChangeValue, nHeight * nDstPitches[0]);

            memset(pDst[1], nSceneChangeValue, nHeightUV * nDstPitches[1]);
            memset(pDst[2], nSceneChangeValue, nHeightUV * nDstPitches[2]);
        }

        fgopDeinit(&fgop);

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvmaskFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVMaskData *d = (MVMaskData *)instanceData;

    vsapi->freeNode(d->node);
    vsapi->freeNode(d->vectors);
    simpleDeinit(&d->upsizer);
    simpleDeinit(&d->upsizerUV);
    free(d);
}


static void VS_CC mvmaskCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVMaskData d;
    MVMaskData *data;

    int err;

    d.ml = (float)vsapi->mapGetFloat(in, "ml", 0, &err);
    if (err)
        d.ml = 100.0f;

    d.fGamma = (float)vsapi->mapGetFloat(in, "gamma", 0, &err);
    if (err)
        d.fGamma = 1.0f;

    d.kind = vsapi->mapGetIntSaturated(in, "kind", 0, &err);

    double time = vsapi->mapGetFloat(in, "time", 0, &err);
    if (err)
        time = 100.0;

    d.nSceneChangeValue = vsapi->mapGetIntSaturated(in, "ysc", 0, &err);

    d.thscd1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.opt = !!vsapi->mapGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;


    if (d.fGamma < 0.0f) {
        vsapi->mapSetError(out, "Mask: gamma must not be negative.");
        return;
    }

    if (d.kind < 0 || d.kind > 5) {
        vsapi->mapSetError(out, "Mask: kind must 0, 1, 2, 3, 4, or 5.");
        return;
    }

    if (time < 0.0 || time > 100.0) {
        vsapi->mapSetError(out, "Mask: time must be between 0.0 and 100.0 (inclusive).");
        return;
    }

    if (d.nSceneChangeValue < 0 || d.nSceneChangeValue > 255) {
        vsapi->mapSetError(out, "Mask: ysc must be between 0 and 255 (inclusive).");
        return;
    }


    d.vectors = vsapi->mapGetNode(in, "vectors", 0, NULL);

#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };
    const char *filter_name = "Mask";

    adataFromVectorClip(&d.vectors_data, d.vectors, filter_name, "vectors", vsapi, error, ERROR_SIZE);

    scaleThSCD(&d.thscd1, &d.thscd2, &d.vectors_data, filter_name, error, ERROR_SIZE);
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->mapSetError(out, error);

        vsapi->freeNode(d.vectors);
        return;
    }


    d.fMaskNormFactor = 1.0f / d.ml; // Fizick
    d.fMaskNormFactor2 = d.fMaskNormFactor * d.fMaskNormFactor;

    d.fHalfGamma = d.fGamma * 0.5f;

    d.nWidthB = d.vectors_data.nBlkX * (d.vectors_data.nBlkSizeX - d.vectors_data.nOverlapX) + d.vectors_data.nOverlapX;
    d.nHeightB = d.vectors_data.nBlkY * (d.vectors_data.nBlkSizeY - d.vectors_data.nOverlapY) + d.vectors_data.nOverlapY;

    d.nHeightUV = d.vectors_data.nHeight / d.vectors_data.yRatioUV;
    d.nWidthUV = d.vectors_data.nWidth / d.vectors_data.xRatioUV;
    d.nHeightBUV = d.nHeightB / d.vectors_data.yRatioUV;
    d.nWidthBUV = d.nWidthB / d.vectors_data.xRatioUV;


    d.node = vsapi->mapGetNode(in, "clip", 0, NULL);
    d.vi = *vsapi->getVideoInfo(d.node);

    if (!vsh_isConstantVideoFormat(&d.vi) || d.vi.format.bitsPerSample > 8 || d.vi.format.subSamplingW > 1 || d.vi.format.subSamplingH > 1 || (d.vi.format.colorFamily != cfYUV && d.vi.format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, "Mask: input clip must be GRAY8, YUV420P8, YUV422P8, YUV440P8, or YUV444P8, with constant dimensions.");
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.vectors);
        return;
    }

    if (d.vi.format.colorFamily == cfGray)
        vsapi->getVideoFormatByID(&d.vi.format, pfYUV444P8, core);

    simpleInit(&d.upsizer, d.nWidthB, d.nHeightB, d.vectors_data.nBlkX, d.vectors_data.nBlkY, d.vectors_data.nWidth, d.vectors_data.nHeight, d.vectors_data.nPel, d.opt);
    simpleInit(&d.upsizerUV, d.nWidthBUV, d.nHeightBUV, d.vectors_data.nBlkX, d.vectors_data.nBlkY, d.nWidthUV, d.nHeightUV, d.vectors_data.nPel, d.opt);

    d.time256 = (int)(time * 256 / 100);


    data = (MVMaskData *)malloc(sizeof(d));
    *data = d;

    VSFilterDependency deps[2] = { 
        {data->node, rpStrictSpatial}, 
        {data->vectors, rpStrictSpatial},
    };

    vsapi->createVideoFilter(out, "Mask", &data->vi, mvmaskGetFrame, mvmaskFree, fmParallel, deps, ARRAY_SIZE(deps), data, core);
}


void mvmaskRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Mask",
                 "clip:vnode;"
                 "vectors:vnode;"
                 "ml:float:opt;"
                 "gamma:float:opt;"
                 "kind:int:opt;"
                 "time:float:opt;"
                 "ysc:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "opt:int:opt;",
                 "clip:vnode;",
                 mvmaskCreate, 0, plugin);
}
