#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "MVFrame.h"


typedef struct MVSuperData {
    VSNode *node;
    VSVideoInfo vi;

    VSNode *pelclip; // upsized source clip with doubled frame width and heigth (used for pel=2)

    int nHPad;
    int nVPad;
    int nPel;
    int nLevels;
    int sharp;
    int rfilter; // frame reduce filter mode
    int opt;

    int nWidth;
    int nHeight;

    int yRatioUV;
    int xRatioUV;
    int chroma;
    int usePelClip;
    int nSuperWidth;
    int nSuperHeight;

    MVPlaneSet nModeYUV;

    int isPelClipPadded;
} MVSuperData;


static const VSFrame *VS_CC mvsuperGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVSuperData *d = (MVSuperData *)instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node, frameCtx);
        if (d->usePelClip)
            vsapi->requestFrameFilter(n, d->pelclip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);

        const uint8_t *pSrc[3] = { NULL };
        uint8_t *pDst[3] = { NULL };
        const uint8_t *pSrcPel[3] = { NULL };
        int nSrcPitch[3] = { 0 };
        int nDstPitch[3] = { 0 };
        int nSrcPelPitch[3] = { 0 };

        const VSFrame *srcPel = NULL;
        if (d->usePelClip)
            srcPel = vsapi->getFrameFilter(n, d->pelclip, frameCtx);

        VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, src, core);

        for (int plane = 0; plane < d->vi.format.numPlanes; plane++) {
            pSrc[plane] = vsapi->getReadPtr(src, plane);
            nSrcPitch[plane] = vsapi->getStride(src, plane);

            pDst[plane] = vsapi->getWritePtr(dst, plane);
            nDstPitch[plane] = vsapi->getStride(dst, plane);

            memset(pDst[plane], 0, nDstPitch[plane] * vsapi->getFrameHeight(dst, plane));
        }

        MVGroupOfFrames pSrcGOF;
        mvgofInit(&pSrcGOF, d->nLevels, d->nWidth, d->nHeight, d->nPel, d->nHPad, d->nVPad, d->nModeYUV, d->opt, d->xRatioUV, d->yRatioUV, d->vi.format.bitsPerSample);

        mvgofUpdate(&pSrcGOF, pDst, nDstPitch);

        MVPlaneSet planes[3] = { YPLANE, UPLANE, VPLANE };

        for (int plane = 0; plane < d->vi.format.numPlanes; plane++)
            mvfFillPlane(pSrcGOF.frames[0], pSrc[plane], nSrcPitch[plane], plane);

        mvgofReduce(&pSrcGOF, d->nModeYUV, d->rfilter);
        mvgofPad(&pSrcGOF, d->nModeYUV);

        if (d->usePelClip) {
            MVFrame *srcFrames = pSrcGOF.frames[0];

            for (int plane = 0; plane < d->vi.format.numPlanes; plane++) {
                pSrcPel[plane] = vsapi->getReadPtr(srcPel, plane);
                nSrcPelPitch[plane] = vsapi->getStride(srcPel, plane);

                MVPlane *srcPlane = srcFrames->planes[plane];
                if (d->nModeYUV & planes[plane])
                    mvpRefineExt(srcPlane, pSrcPel[plane], nSrcPelPitch[plane], d->isPelClipPadded);
            }
        } else
            mvgofRefine(&pSrcGOF, d->nModeYUV, d->sharp);

        vsapi->freeFrame(src);
        if (d->usePelClip)
            vsapi->freeFrame(srcPel);

        mvgofDeinit(&pSrcGOF);

        if (n == 0) {
            VSMap *props = vsapi->getFramePropertiesRW(dst);

            vsapi->mapSetInt(props, "Super_height", d->nHeight, maReplace);
            vsapi->mapSetInt(props, "Super_hpad", d->nHPad, maReplace);
            vsapi->mapSetInt(props, "Super_vpad", d->nVPad, maReplace);
            vsapi->mapSetInt(props, "Super_pel", d->nPel, maReplace);
            vsapi->mapSetInt(props, "Super_modeyuv", d->nModeYUV, maReplace);
            vsapi->mapSetInt(props, "Super_levels", d->nLevels, maReplace);
        }

        return dst;
    }

    return 0;
}


static void VS_CC mvsuperFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVSuperData *d = (MVSuperData *)instanceData;

    vsapi->freeNode(d->node);
    vsapi->freeNode(d->pelclip);
    free(d);
}


static void VS_CC mvsuperCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVSuperData d;
    MVSuperData *data;

    int err;

    d.nHPad = vsapi->mapGetIntSaturated(in, "hpad", 0, &err);
    if (err)
        d.nHPad = 16;

    d.nVPad = vsapi->mapGetIntSaturated(in, "vpad", 0, &err);
    if (err)
        d.nVPad = 16;

    d.nPel = vsapi->mapGetIntSaturated(in, "pel", 0, &err);
    if (err)
        d.nPel = 2;

    d.nLevels = vsapi->mapGetIntSaturated(in, "levels", 0, &err);

    d.chroma = !!vsapi->mapGetInt(in, "chroma", 0, &err);
    if (err)
        d.chroma = 1;

    d.sharp = vsapi->mapGetIntSaturated(in, "sharp", 0, &err); // pel2 interpolation type
    if (err)
        d.sharp = SharpWiener;

    d.rfilter = vsapi->mapGetIntSaturated(in, "rfilter", 0, &err);
    if (err)
        d.rfilter = RfilterBilinear;

    d.opt = !!vsapi->mapGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;


    if ((d.nPel != 1) && (d.nPel != 2) && (d.nPel != 4)) {
        vsapi->mapSetError(out, "Super: pel must be 1, 2, or 4.");
        return;
    }

    if (d.sharp < SharpBilinear || d.sharp > SharpWiener) {
        vsapi->mapSetError(out, "Super: sharp must be between 0 and 2 (inclusive).");
        return;
    }

    if (d.rfilter < RfilterSimple || d.rfilter > RfilterCubic) {
        vsapi->mapSetError(out, "Super: rfilter must be between 0 and 4 (inclusive).");
        return;
    }


    d.node = vsapi->mapGetNode(in, "clip", 0, 0);

    // Make a copy of the video info, so we can reference
    // it and modify it below.
    d.vi = *vsapi->getVideoInfo(d.node);

    d.nWidth = d.vi.width;
    d.nHeight = d.vi.height;

    if (!vsh_isConstantVideoFormat(&d.vi) || d.vi.format.bitsPerSample > 16 || d.vi.format.sampleType != stInteger ||
            d.vi.format.subSamplingW > 1 || d.vi.format.subSamplingH > 1 || (d.vi.format.colorFamily != cfYUV && d.vi.format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, "Super: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.vi.format.colorFamily == cfGray)
        d.chroma = 0;

    d.nModeYUV = d.chroma ? YUVPLANES : YPLANE;


    d.xRatioUV = 1 << d.vi.format.subSamplingW;
    d.yRatioUV = 1 << d.vi.format.subSamplingH;

    int nLevelsMax = 0;
    while (PlaneHeightLuma(d.vi.height, nLevelsMax, d.yRatioUV, d.nVPad) >= d.yRatioUV * 2 &&
           PlaneWidthLuma(d.vi.width, nLevelsMax, d.xRatioUV, d.nHPad) >= d.xRatioUV * 2) // at last two pixels width and height of chroma
    {
        nLevelsMax++;
    }
    if (d.nLevels <= 0 || d.nLevels > nLevelsMax)
        d.nLevels = nLevelsMax;

    d.pelclip = vsapi->mapGetNode(in, "pelclip", 0, &err);
    const VSVideoInfo *pelvi = d.pelclip ? vsapi->getVideoInfo(d.pelclip) : NULL;

    if (d.pelclip && (!vsh_isConstantVideoFormat(pelvi) || !vsh_isSameVideoFormat(&pelvi->format, &d.vi.format))) {
        vsapi->mapSetError(out, "Super: pelclip must have the same format as the input clip, and it must have constant dimensions.");
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.pelclip);
        return;
    }

    d.usePelClip = 0;
    if (d.pelclip && (d.nPel >= 2)) {
        if ((pelvi->width == d.vi.width * d.nPel) &&
            (pelvi->height == d.vi.height * d.nPel)) {
            d.usePelClip = 1;
            d.isPelClipPadded = 0;
        } else if ((pelvi->width == (d.vi.width + d.nHPad * 2) * d.nPel) &&
                   (pelvi->height == (d.vi.height + d.nVPad * 2) * d.nPel)) {
            d.usePelClip = 1;
            d.isPelClipPadded = 1;
        } else {
            vsapi->mapSetError(out, "Super: pelclip's dimensions must be multiples of the input clip's dimensions.");
            vsapi->freeNode(d.pelclip);
            vsapi->freeNode(d.node);
            return;
        }
    }

    d.nSuperWidth = d.nWidth + 2 * d.nHPad;
    d.nSuperHeight = PlaneSuperOffset(0, d.nHeight, d.nLevels, d.nPel, d.nVPad, d.nSuperWidth, d.yRatioUV) / d.nSuperWidth;
    if (d.yRatioUV == 2 && d.nSuperHeight & 1)
        d.nSuperHeight++; // even
    if (d.xRatioUV == 2 && d.nSuperWidth & 1)
        d.nSuperWidth++;
    d.vi.width = d.nSuperWidth;
    d.vi.height = d.nSuperHeight;


    data = (MVSuperData *)malloc(sizeof(d));
    *data = d;

    VSFilterDependency deps[1] = { 
        {data->node, rpStrictSpatial}
    };

    vsapi->createVideoFilter(out, "Super", &data->vi, mvsuperGetFrame, mvsuperFree, fmParallel, deps, 1, data, core);
}


void mvsuperRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Super", 
                 "clip:vnode;"
                 "hpad:int:opt;"
                 "vpad:int:opt;"
                 "pel:int:opt;"
                 "levels:int:opt;"
                 "chroma:int:opt;"
                 "sharp:int:opt;"
                 "rfilter:int:opt;"
                 "pelclip:vnode:opt;"
                 "opt:int:opt;",
                 "clip:vnode;",
                 mvsuperCreate, 0, plugin);
}
