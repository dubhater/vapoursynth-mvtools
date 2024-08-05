#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "CPU.h"
#include "DCTFFTW.h"
#include "Fakery.h"
#include "GroupOfPlanes.h"
#include "MVAnalysisData.h"


typedef struct MVRecalculateData {
    VSNode *node;
    const VSVideoInfo *vi;

    MVAnalysisData vectors_data;

    MVAnalysisData analysisData;
    MVAnalysisData analysisDataDivided;

    /*! \brief optimisations enabled */
    int opt;

    /*! \brief motion vecteur cost factor */
    int nLambda;

    /*! \brief search type chosen for refinement in the EPZ */
    SearchType searchType;

    /*! \brief additionnal parameter for this search */
    int nSearchParam; // usually search radius


    int pnew;        // penalty to cost for new canditate - added by Fizick
    int plen;        // penalty factor (similar to lambda) for vector length - added by Fizick
    int divideExtra; // divide blocks on sublocks with median motion
    int meander;     //meander (alternate) scan blocks (even row left to right, odd row right to left

    int dctmode;

    int nModeYUV;

    int nSuperLevels;
    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;

    int searchparam;
    int chroma;
    int truemotion;
    int smooth;
    int64_t thSAD;
    VSNode *vectors;

    int fields;
    int tff;
    int tff_exists;
} MVRecalculateData;


static const VSFrame *VS_CC mvrecalculateGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVRecalculateData *d = (MVRecalculateData *)instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        int nref;
        int offset = d->analysisData.nDeltaFrame;

        if (offset > 0) {
            nref = d->analysisData.isBackward ? n + offset : n - offset;
        } else {
            nref = -offset;
        }

        if (nref >= 0 && nref < d->vi->numFrames) {
            if (n < nref) {
                vsapi->requestFrameFilter(n, d->node, frameCtx);
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
                vsapi->requestFrameFilter(n, d->node, frameCtx);
            }
        } else { // too close to beginning/end of clip
            vsapi->requestFrameFilter(n, d->node, frameCtx);
        }
    } else if (activationReason == arAllFramesReady) {

        GroupOfPlanes vectorFields;

        gopInit(&vectorFields, d->analysisData.nBlkSizeX, d->analysisData.nBlkSizeY, d->analysisData.nLvCount, d->analysisData.nPel, d->analysisData.nMotionFlags, d->analysisData.nCPUFlags, d->analysisData.nOverlapX, d->analysisData.nOverlapY, d->analysisData.nBlkX, d->analysisData.nBlkY, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->divideExtra, d->vi->format.bitsPerSample);


        const uint8_t *pSrc[3] = { NULL };
        const uint8_t *pRef[3] = { NULL };
        int nSrcPitch[3] = { 0 };
        int nRefPitch[3] = { 0 };

        int nref;
        int offset = d->analysisData.nDeltaFrame;

        if (offset > 0) {
            nref = d->analysisData.isBackward ? n + offset : n - offset;
        } else {
            nref = -offset;
        }

        const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        const VSMap *srcprops = vsapi->getFramePropertiesRO(src);
        int err;

        int src_top_field = !!vsapi->mapGetInt(srcprops, "_Field", 0, &err);
        if (err && d->fields && !d->tff_exists) {
            vsapi->setFilterError("Recalculate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
            gopDeinit(&vectorFields);
            vsapi->freeFrame(src);
            return NULL;
        }

        // if tff was passed, it overrides _Field.
        if (d->tff_exists)
            src_top_field = d->tff ^ (n % 2);


        for (int plane = 0; plane < d->vi->format.numPlanes; plane++) {
            pSrc[plane] = vsapi->getReadPtr(src, plane);
            nSrcPitch[plane] = vsapi->getStride(src, plane);
        }


        FakeGroupOfPlanes fgop;
        fgopInit(&fgop, &d->vectors_data);

        const VSFrame *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        const VSMap *mvprops = vsapi->getFramePropertiesRO(mvn);

        fgopUpdate(&fgop, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));
        vsapi->freeFrame(mvn);

        MVArraySizeType vectors_size = gopGetArraySize(&vectorFields);
        uint8_t *vectors = (uint8_t *)malloc(vectors_size);

        if (fgopIsValid(&fgop) && nref >= 0 && nref < d->vi->numFrames) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->node, frameCtx);
            const VSMap *refprops = vsapi->getFramePropertiesRO(ref);

            int ref_top_field = !!vsapi->mapGetInt(refprops, "_Field", 0, &err);
            if (err && d->fields && !d->tff_exists) {
                vsapi->setFilterError("Recalculate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                gopDeinit(&vectorFields);
                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);
                free(vectors);
                fgopDeinit(&fgop);
                return NULL;
            }

            // if tff was passed, it overrides _Field.
            if (d->tff_exists)
                ref_top_field = d->tff ^ (nref % 2);

            int fieldShift = 0;
            if (d->fields && d->analysisData.nPel > 1 && (d->analysisData.nDeltaFrame % 2)) {
                fieldShift = (src_top_field && !ref_top_field) ? d->analysisData.nPel / 2 : ((ref_top_field && !src_top_field) ? -(d->analysisData.nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }


            for (int plane = 0; plane < d->vi->format.numPlanes; plane++) {
                pRef[plane] = vsapi->getReadPtr(ref, plane);
                nRefPitch[plane] = vsapi->getStride(ref, plane);
            }


            MVGroupOfFrames pSrcGOF, pRefGOF;

            mvgofInit(&pSrcGOF, d->nSuperLevels, d->analysisData.nWidth, d->analysisData.nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->opt, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->vi->format.bitsPerSample);
            mvgofInit(&pRefGOF, d->nSuperLevels, d->analysisData.nWidth, d->analysisData.nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->opt, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->vi->format.bitsPerSample);

            // cast away the const, because why not.
            mvgofUpdate(&pSrcGOF, (uint8_t **)pSrc, nSrcPitch);
            mvgofUpdate(&pRefGOF, (uint8_t **)pRef, nRefPitch);


            DCTFFTW *DCTc = NULL;
            if (d->dctmode >= 1 && d->dctmode <= 4) {
                DCTc = (DCTFFTW *)malloc(sizeof(DCTFFTW));
                dctInit(DCTc, d->analysisData.nBlkSizeX, d->analysisData.nBlkSizeY, d->vi->format.bitsPerSample, d->opt);
            }


            gopRecalculateMVs(&vectorFields, &fgop, &pSrcGOF, &pRefGOF, d->searchType, d->nSearchParam, d->nLambda, d->pnew, vectors, fieldShift, d->thSAD, DCTc, d->dctmode, d->smooth, d->meander);

            if (d->divideExtra) {
                // make extra level with divided sublocks with median (not estimated) motion
                gopExtraDivide(&vectorFields, vectors);
            }

            gopDeinit(&vectorFields);
            if (DCTc) {
                dctDeinit(DCTc);
                free(DCTc);
            }
            mvgofDeinit(&pSrcGOF);
            mvgofDeinit(&pRefGOF);
            vsapi->freeFrame(ref);
        } else {// too close to the beginning or end to do anything
            gopWriteDefaultToArray(&vectorFields, vectors);
            gopDeinit(&vectorFields);
        }

        VSFrame *dst = vsapi->copyFrame(src, core);
        VSMap *dstprops = vsapi->getFramePropertiesRW(dst);

        vsapi->mapSetData(dstprops,
                           prop_MVTools_MVAnalysisData,
                           (const char *)(d->divideExtra ? &d->analysisDataDivided : &d->analysisData),
                           sizeof(MVAnalysisData),
                           dtBinary,
                           maReplace);

        vsapi->mapSetData(dstprops,
                           prop_MVTools_vectors,
                           (const char *)vectors,
                           vectors_size,
                           dtBinary,
                           maReplace);

        free(vectors);

#if defined(MVTOOLS_X86)
        mvtools_cpu_emms();
#endif

        vsapi->freeFrame(src);

        fgopDeinit(&fgop);

        return dst;
    }

    return 0;
}


static void VS_CC mvrecalculateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVRecalculateData *d = (MVRecalculateData *)instanceData;

    vsapi->freeNode(d->node);
    vsapi->freeNode(d->vectors);
    free(d);
}


static void VS_CC mvrecalculateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVRecalculateData d;
    MVRecalculateData *data;

    int err;

    d.thSAD = vsapi->mapGetInt(in, "thsad", 0, &err);
    if (err)
        d.thSAD = 200;

    d.smooth = vsapi->mapGetIntSaturated(in, "smooth", 0, &err);
    if (err)
        d.smooth = 1;

    d.analysisData.nBlkSizeX = vsapi->mapGetIntSaturated(in, "blksize", 0, &err);
    if (err)
        d.analysisData.nBlkSizeX = 8;

    d.analysisData.nBlkSizeY = vsapi->mapGetIntSaturated(in, "blksizev", 0, &err);
    if (err)
        d.analysisData.nBlkSizeY = d.analysisData.nBlkSizeX;

    d.searchType = (SearchType)vsapi->mapGetIntSaturated(in, "search", 0, &err);
    if (err)
        d.searchType = SearchHex2;

    d.searchparam = vsapi->mapGetIntSaturated(in, "searchparam", 0, &err);
    if (err)
        d.searchparam = 2;

    d.chroma = !!vsapi->mapGetInt(in, "chroma", 0, &err);
    if (err)
        d.chroma = 1;

    d.truemotion = !!vsapi->mapGetInt(in, "truemotion", 0, &err);
    if (err)
        d.truemotion = 1;

    d.nLambda = vsapi->mapGetIntSaturated(in, "lambda", 0, &err);
    if (err)
        d.nLambda = d.truemotion ? (1000 * d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY / 64) : 0;

    d.pnew = vsapi->mapGetIntSaturated(in, "pnew", 0, &err);
    if (err)
        d.pnew = d.truemotion ? 50 : 0; // relative to 256

    d.analysisData.nOverlapX = vsapi->mapGetIntSaturated(in, "overlap", 0, &err);

    d.analysisData.nOverlapY = vsapi->mapGetIntSaturated(in, "overlapv", 0, &err);
    if (err)
        d.analysisData.nOverlapY = d.analysisData.nOverlapX;

    d.dctmode = vsapi->mapGetIntSaturated(in, "dct", 0, &err);

    d.divideExtra = vsapi->mapGetIntSaturated(in, "divide", 0, &err);

    d.opt = !!vsapi->mapGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;

    d.meander = !!vsapi->mapGetInt(in, "meander", 0, &err);
    if (err)
        d.meander = 1;

    d.fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    d.tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d.tff_exists = !err;


    if (d.searchType < 0 || d.searchType > 7) {
        vsapi->mapSetError(out, "Recalculate: search must be between 0 and 7 (inclusive).");
        return;
    }

    if (d.dctmode < 0 || d.dctmode > 10) {
        vsapi->mapSetError(out, "Recalculate: dct must be between 0 and 10 (inclusive).");
        return;
    }

    if (d.dctmode >= 5 && d.analysisData.nBlkSizeX == 16 && d.analysisData.nBlkSizeY == 2) {
        vsapi->mapSetError(out, "Recalculate: dct 5..10 cannot work with 16x2 blocks.");
        return;
    }

    if (d.divideExtra < 0 || d.divideExtra > 2) {
        vsapi->mapSetError(out, "Recalculate: divide must be between 0 and 2 (inclusive).");
        return;
    }


    if ((d.analysisData.nBlkSizeX != 4 || d.analysisData.nBlkSizeY != 4) &&
        (d.analysisData.nBlkSizeX != 8 || d.analysisData.nBlkSizeY != 4) &&
        (d.analysisData.nBlkSizeX != 8 || d.analysisData.nBlkSizeY != 8) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 2) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 8) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 16) &&
        (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 16) &&
        (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 32) &&
        (d.analysisData.nBlkSizeX != 64 || d.analysisData.nBlkSizeY != 32) &&
        (d.analysisData.nBlkSizeX != 64 || d.analysisData.nBlkSizeY != 64) &&
        (d.analysisData.nBlkSizeX != 128 || d.analysisData.nBlkSizeY != 64) &&
        (d.analysisData.nBlkSizeX != 128 || d.analysisData.nBlkSizeY != 128)) {

        vsapi->mapSetError(out, "Recalculate: the block size must be 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, 32x32, 64x32, 64x64, 128x64, or 128x128.");
        return;
    }


    if (d.pnew < 0 || d.pnew > 256) {
        vsapi->mapSetError(out, "Recalculate: pnew must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.analysisData.nOverlapX < 0 || d.analysisData.nOverlapX > d.analysisData.nBlkSizeX / 2 ||
        d.analysisData.nOverlapY < 0 || d.analysisData.nOverlapY > d.analysisData.nBlkSizeY / 2) {
        vsapi->mapSetError(out, "Recalculate: overlap must be at most half of blksize, overlapv must be at most half of blksizev, and they both need to be at least 0.");
        return;
    }

    if (d.divideExtra && (d.analysisData.nBlkSizeX < 8 || d.analysisData.nBlkSizeY < 8)) {
        vsapi->mapSetError(out, "Recalculate: blksize and blksizev must be at least 8 when divide=True.");
        return;
    }


    if (d.searchType == SearchNstep)
        d.nSearchParam = (d.searchparam < 0) ? 0 : d.searchparam;
    else
        d.nSearchParam = (d.searchparam < 1) ? 1 : d.searchparam;


    d.node = vsapi->mapGetNode(in, "super", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    if (d.analysisData.nOverlapX % (1 << d.vi->format.subSamplingW) ||
        d.analysisData.nOverlapY % (1 << d.vi->format.subSamplingH)) {
        vsapi->mapSetError(out, "Recalculate: The requested overlap is incompatible with the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.divideExtra && (d.analysisData.nOverlapX % (2 << d.vi->format.subSamplingW) ||
                          d.analysisData.nOverlapY % (2 << d.vi->format.subSamplingH))) { // subsampling times 2
        vsapi->mapSetError(out, "Recalculate: overlap and overlapv must be multiples of 2 or 4 when divide=True, depending on the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }


#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Recalculate: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrame *evil = vsapi->getFrame(0, d.node, errorMsg + errorLen, ERROR_SIZE - errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->mapSetError(out, errorMsg);
        vsapi->freeNode(d.node);
        return;
    }
    const VSMap *props = vsapi->getFramePropertiesRO(evil);
    int evil_err[6];
    int nHeight = vsapi->mapGetIntSaturated(props, "Super_height", 0, &evil_err[0]);
    d.nSuperHPad = vsapi->mapGetIntSaturated(props, "Super_hpad", 0, &evil_err[1]);
    d.nSuperVPad = vsapi->mapGetIntSaturated(props, "Super_vpad", 0, &evil_err[2]);
    d.nSuperPel = vsapi->mapGetIntSaturated(props, "Super_pel", 0, &evil_err[3]);
    d.nSuperModeYUV = vsapi->mapGetIntSaturated(props, "Super_modeyuv", 0, &evil_err[4]);
    d.nSuperLevels = vsapi->mapGetIntSaturated(props, "Super_levels", 0, &evil_err[5]);
    vsapi->freeFrame(evil);

    for (int i = 0; i < 6; i++)
        if (evil_err[i]) {
            vsapi->mapSetError(out, "Recalculate: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.node);
            return;
        }


    if (d.vi->format.colorFamily == cfGray)
        d.chroma = 0;

    d.nModeYUV = d.chroma ? YUVPLANES : YPLANE;

    if ((d.nModeYUV & d.nSuperModeYUV) != d.nModeYUV) { //x
        vsapi->mapSetError(out, "Recalculate: super clip does not contain needed colour data.");
        vsapi->freeNode(d.node);
        return;
    }

    d.vectors = vsapi->mapGetNode(in, "vectors", 0, NULL);

#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };
    const char *filter_name = "Recalculate";

    adataFromVectorClip(&d.vectors_data, d.vectors, filter_name, "vectors", vsapi, error, ERROR_SIZE);
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->mapSetError(out, error);

        vsapi->freeNode(d.node);
        vsapi->freeNode(d.vectors);
        return;
    }


    d.analysisData.yRatioUV = d.vectors_data.yRatioUV;
    d.analysisData.xRatioUV = d.vectors_data.xRatioUV;

    d.analysisData.nWidth = d.vectors_data.nWidth;
    d.analysisData.nHeight = d.vectors_data.nHeight;

    d.analysisData.nDeltaFrame = d.vectors_data.nDeltaFrame;
    d.analysisData.isBackward = d.vectors_data.isBackward;


    d.analysisData.bitsPerSample = d.vi->format.bitsPerSample;

    int pixelMax = (1 << d.vi->format.bitsPerSample) - 1;
    d.thSAD = (int64_t)((double)d.thSAD * pixelMax / 255.0 + 0.5);
    d.nLambda = (int)((double)d.nLambda * pixelMax / 255.0 + 0.5);

    // normalize threshold to block size
    int referenceBlockSize = 8 * 8;
    d.thSAD = d.thSAD * (d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY) / referenceBlockSize;
    if (d.chroma)
        d.thSAD += d.thSAD / (d.analysisData.xRatioUV * d.analysisData.yRatioUV) * 2;


    d.analysisData.nMotionFlags = 0;
    d.analysisData.nMotionFlags |= d.opt ? MOTION_USE_SIMD : 0;
    d.analysisData.nMotionFlags |= d.analysisData.isBackward ? MOTION_IS_BACKWARD : 0;
    d.analysisData.nMotionFlags |= d.chroma ? MOTION_USE_CHROMA_MOTION : 0;


    if (d.opt) {
        d.analysisData.nCPUFlags = g_cpuinfo;
    }

    d.analysisData.nPel = d.nSuperPel; //x

    int nSuperWidth = d.vi->width;
    if (nHeight != d.analysisData.nHeight || nSuperWidth - 2 * d.nSuperHPad != d.analysisData.nWidth) {
        vsapi->mapSetError(out, "Recalculate: wrong frame size.");
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.vectors);
        return;
    }

    d.analysisData.nHPadding = d.nSuperHPad; //v2.0    //x
    d.analysisData.nVPadding = d.nSuperVPad;


    int nBlkX = (d.analysisData.nWidth - d.analysisData.nOverlapX) / (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX); //x
    int nBlkY = (d.analysisData.nHeight - d.analysisData.nOverlapY) / (d.analysisData.nBlkSizeY - d.analysisData.nOverlapY);

    d.analysisData.nBlkX = nBlkX;
    d.analysisData.nBlkY = nBlkY;

    d.analysisData.nLvCount = 1;


    if (d.divideExtra) { //v1.8.1
        memcpy(&d.analysisDataDivided, &d.analysisData, sizeof(d.analysisData));
        d.analysisDataDivided.nBlkX = d.analysisData.nBlkX * 2;
        d.analysisDataDivided.nBlkY = d.analysisData.nBlkY * 2;
        d.analysisDataDivided.nBlkSizeX = d.analysisData.nBlkSizeX / 2;
        d.analysisDataDivided.nBlkSizeY = d.analysisData.nBlkSizeY / 2;
        d.analysisDataDivided.nOverlapX = d.analysisData.nOverlapX / 2;
        d.analysisDataDivided.nOverlapY = d.analysisData.nOverlapY / 2;
        d.analysisDataDivided.nLvCount = d.analysisData.nLvCount + 1;
    }


    data = (MVRecalculateData *)malloc(sizeof(d));
    *data = d;

    VSFilterDependency deps[2] = { 
        {data->node, rpGeneral}, // super
        {data->vectors, rpStrictSpatial},
    };

    vsapi->createVideoFilter(out, "Recalculate", data->vi, mvrecalculateGetFrame, mvrecalculateFree, fmParallel, deps, 2, data, core);
}


void mvrecalculateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Recalculate",
                 "super:vnode;"
                 "vectors:vnode;"
                 "thsad:int:opt;"
                 "smooth:int:opt;"
                 "blksize:int:opt;"
                 "blksizev:int:opt;"
                 "search:int:opt;"
                 "searchparam:int:opt;"
                 "lambda:int:opt;"
                 "chroma:int:opt;"
                 "truemotion:int:opt;"
                 "pnew:int:opt;"
                 "overlap:int:opt;"
                 "overlapv:int:opt;"
                 "divide:int:opt;"
                 "opt:int:opt;"
                 "meander:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 "dct:int:opt;",
                 "clip:vnode;",
                 mvrecalculateCreate, 0, plugin);
}
