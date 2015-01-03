#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "MVSuper.h"
#include "MVInterface.h"


typedef struct {
    VSNodeRef *node;
    VSVideoInfo vi;

    VSNodeRef *pelclip; // upsized source clip with doubled frame width and heigth (used for pel=2)

    int nHPad;
    int nVPad;
    int nPel;
    int nLevels;
    int sharp;
    int rfilter; // frame reduce filter mode
    bool isse;

    int nWidth;
    int nHeight;

    int yRatioUV;
    int xRatioUV;
    bool chroma;
    bool usePelClip;
    int nSuperWidth;
    int nSuperHeight;

    MVPlaneSet nModeYUV;

    bool isPelClipPadded;
} MVSuperData;


static void VS_CC mvsuperInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVSuperData *d = (MVSuperData *) * instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvsuperGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVSuperData *d = (MVSuperData *) * instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node, frameCtx);
        if (d->usePelClip)
            vsapi->requestFrameFilter(n, d->pelclip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);

        const uint8_t *pSrcY, *pSrcU, *pSrcV;
        uint8_t *pDstY, *pDstU, *pDstV;
        const uint8_t *pSrcPelY, *pSrcPelU, *pSrcPelV;
        int nSrcPitchY, nSrcPitchUV;
        int nDstPitchY, nDstPitchUV;
        int nSrcPelPitchY, nSrcPelPitchUV;

        const VSFrameRef *srcPel = NULL;
        if (d->usePelClip)
            srcPel = vsapi->getFrameFilter(n, d->pelclip, frameCtx);

        VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

        pSrcY = vsapi->getReadPtr(src, 0);
        pSrcU = vsapi->getReadPtr(src, 1);
        pSrcV = vsapi->getReadPtr(src, 2);
        nSrcPitchY = vsapi->getStride(src, 0);
        nSrcPitchUV = vsapi->getStride(src, 1);

        pDstY = vsapi->getWritePtr(dst, 0);
        pDstU = vsapi->getWritePtr(dst, 1);
        pDstV = vsapi->getWritePtr(dst, 2);
        nDstPitchY  = vsapi->getStride(dst, 0);
        nDstPitchUV  = vsapi->getStride(dst, 1);

        memset(pDstY, 0, nDstPitchY * d->vi.height);
        memset(pDstU, 0, nDstPitchUV * (d->vi.height >> d->vi.format->subSamplingH));
        memset(pDstV, 0, nDstPitchUV * (d->vi.height >> d->vi.format->subSamplingH));

        MVGroupOfFrames *pSrcGOF = new MVGroupOfFrames(d->nLevels, d->nWidth, d->nHeight, d->nPel, d->nHPad, d->nVPad, YUVPLANES, d->isse, d->xRatioUV, d->yRatioUV);

        pSrcGOF->Update(YUVPLANES, pDstY, nDstPitchY, pDstU, nDstPitchUV, pDstV, nDstPitchUV);

        pSrcGOF->SetPlane(pSrcY, nSrcPitchY, YPLANE);
        pSrcGOF->SetPlane(pSrcU, nSrcPitchUV, UPLANE);
        pSrcGOF->SetPlane(pSrcV, nSrcPitchUV, VPLANE);

        pSrcGOF->Reduce(d->nModeYUV, d->rfilter);
        pSrcGOF->Pad(d->nModeYUV);

        if (d->usePelClip)
        {
            pSrcPelY = vsapi->getReadPtr(srcPel, 0);
            pSrcPelU = vsapi->getReadPtr(srcPel, 1);
            pSrcPelV = vsapi->getReadPtr(srcPel, 2);
            nSrcPelPitchY = vsapi->getStride(srcPel, 0);
            nSrcPelPitchUV = vsapi->getStride(srcPel, 1);

            MVFrame *srcFrames = pSrcGOF->GetFrame(0);

            MVPlane *srcPlaneY = srcFrames->GetPlane(YPLANE);
            if (d->nModeYUV & YPLANE)
                srcPlaneY->RefineExt(pSrcPelY, nSrcPelPitchY, d->isPelClipPadded);
            MVPlane *srcPlaneU = srcFrames->GetPlane(UPLANE);
            if (d->nModeYUV & UPLANE)
                srcPlaneU->RefineExt(pSrcPelU, nSrcPelPitchUV, d->isPelClipPadded);
            MVPlane *srcPlaneV = srcFrames->GetPlane(VPLANE);
            if (d->nModeYUV & VPLANE)
                srcPlaneV->RefineExt(pSrcPelV, nSrcPelPitchUV, d->isPelClipPadded);
        }
        else
            pSrcGOF->Refine(d->nModeYUV, d->sharp);

        vsapi->freeFrame(src);
        if (d->usePelClip)
            vsapi->freeFrame(srcPel);

        delete pSrcGOF;

        if (n == 0) {
            VSMap *props = vsapi->getFramePropsRW(dst);

            vsapi->propSetInt(props, "Super height", d->nHeight, paReplace);
            vsapi->propSetInt(props, "Super hpad", d->nHPad, paReplace);
            vsapi->propSetInt(props, "Super vpad", d->nVPad, paReplace);
            vsapi->propSetInt(props, "Super pel", d->nPel, paReplace);
            vsapi->propSetInt(props, "Super modeyuv", d->nModeYUV, paReplace);
            vsapi->propSetInt(props, "Super levels", d->nLevels, paReplace);
        }

        return dst;
    }

    return 0;
}


static void VS_CC mvsuperFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVSuperData *d = (MVSuperData *)instanceData;

    vsapi->freeNode(d->node);
    free(d);
}


static void VS_CC mvsuperCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVSuperData d;
    MVSuperData *data;

    int err;

    d.nHPad = vsapi->propGetInt(in, "hpad", 0, &err);
    if (err)
        d.nHPad = 8;

    d.nVPad = vsapi->propGetInt(in, "vpad", 0, &err);
    if (err)
        d.nVPad = 8;

    d.nPel = vsapi->propGetInt(in, "pel", 0, &err);
    if (err)
        d.nPel = 2;

    d.nLevels = vsapi->propGetInt(in, "levels", 0, &err);

    d.chroma = !!vsapi->propGetInt(in, "chroma", 0, &err);
    if (err)
        d.chroma = 1;

    d.sharp = vsapi->propGetInt(in, "sharp", 0, &err); // pel2 interpolation type
    if (err)
        d.sharp = 2;

    d.rfilter = vsapi->propGetInt(in, "rfilter", 0, &err);
    if (err)
        d.rfilter = 2;

    d.isse = !!vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;


    if (( d.nPel != 1 ) && ( d.nPel != 2 ) && ( d.nPel != 4 )) {
        vsapi->setError(out, "Super: pel must be 1, 2, or 4.");
        return;
    }

    if (d.sharp < 0 || d.sharp > 2) {
        vsapi->setError(out, "Super: sharp must be between 0 and 2 (inclusive).");
        return;
    }

    if (d.rfilter < 0 || d.rfilter > 4) {
        vsapi->setError(out, "Super: rfilter must be between 0 and 4 (inclusive).");
        return;
    }


    d.nModeYUV = d.chroma ? YUVPLANES : YPLANE;


    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = *vsapi->getVideoInfo(d.node);

    d.nWidth = d.vi.width;
    d.nHeight = d.vi.height;

    if (!isConstantFormat(&d.vi) || d.vi.format->bitsPerSample > 8 || d.vi.format->subSamplingW > 1 || d.vi.format->subSamplingH > 1 || d.vi.format->colorFamily != cmYUV) {
        vsapi->setError(out, "Super: input clip must be YUV420P8, YUV422P8, YUV440P8, or YUV444P8, with constant dimensions.");
        vsapi->freeNode(d.node);
        return;
    }

    d.xRatioUV = 1 << d.vi.format->subSamplingW;
    d.yRatioUV = 1 << d.vi.format->subSamplingH;

    int nLevelsMax = 0;
    while (PlaneHeightLuma(d.vi.height, nLevelsMax, d.yRatioUV, d.nVPad) >= d.yRatioUV*2 &&
            PlaneWidthLuma(d.vi.width, nLevelsMax, d.xRatioUV, d.nHPad) >= d.xRatioUV*2) // at last two pixels width and height of chroma
    {
        nLevelsMax++;
    }
    if (d.nLevels <= 0 || d.nLevels > nLevelsMax)
        d.nLevels = nLevelsMax;

    d.pelclip = vsapi->propGetNode(in, "pelclip", 0, &err);
    const VSVideoInfo *pelvi = d.pelclip ? vsapi->getVideoInfo(d.pelclip) : NULL;

    if (d.pelclip && (!isConstantFormat(pelvi) || pelvi->format != d.vi.format)) {
        vsapi->setError(out, "Super: pelclip must have the same format as the input clip, and it must have constant dimensions.");
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.pelclip);
        return;
    }

    d.usePelClip = false;
    if (d.pelclip && (d.nPel >= 2))
    {
        if ((pelvi->width == d.vi.width * d.nPel) &&
                (pelvi->height == d.vi.height * d.nPel)) {
            d.usePelClip = true;
            d.isPelClipPadded = false;
        } else if ((pelvi->width == (d.vi.width + d.nHPad * 2) * d.nPel) &&
                (pelvi->height == (d.vi.height + d.nVPad * 2) * d.nPel)) {
            d.usePelClip = true;
            d.isPelClipPadded = true;
        } else {
            vsapi->setError(out, "Super: pelclip's dimensions must be multiples of the input clip's dimensions.");
            vsapi->freeNode(d.pelclip);
            vsapi->freeNode(d.node);
            return;
        }
    }

    d.nSuperWidth = d.nWidth + 2 * d.nHPad;
    d.nSuperHeight = PlaneSuperOffset(false, d.nHeight, d.nLevels, d.nPel, d.nVPad, d.nSuperWidth, d.yRatioUV) / d.nSuperWidth;
    if (d.yRatioUV == 2 && d.nSuperHeight & 1)
        d.nSuperHeight++; // even
    if (d.xRatioUV == 2 && d.nSuperWidth & 1)
        d.nSuperWidth++;
    d.vi.width = d.nSuperWidth;
    d.vi.height = d.nSuperHeight;


    data = (MVSuperData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Super", mvsuperInit, mvsuperGetFrame, mvsuperFree, fmParallel, 0, data, core);
}


void mvsuperRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Super",
            "clip:clip;"
            "hpad:int:opt;"
            "vpad:int:opt;"
            "pel:int:opt;"
            "levels:int:opt;"
            "chroma:int:opt;"
            "sharp:int:opt;"
            "rfilter:int:opt;"
            "pelclip:clip:opt;"
            "isse:int:opt;"
            , mvsuperCreate, 0, plugin);
}
