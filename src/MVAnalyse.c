#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "Bullshit.h"
#include "CPU.h"
#include "DCTFFTW.h"
#include "GroupOfPlanes.h"
#include "MVAnalysisData.h"


typedef struct MVAnalyseData {
    VSNodeRef *node;
    const VSVideoInfo *vi;
    const VSVideoInfo *supervi;

    MVAnalysisData analysisData;
    MVAnalysisData analysisDataDivided;

    /*! \brief optimisations enabled */
    int opt;

    /*! \brief motion vecteur cost factor */
    int nLambda;

    /*! \brief search type chosen for refinement in the EPZ */
    SearchType searchType;

    SearchType searchTypeCoarse;

    /*! \brief additionnal parameter for this search */
    int nSearchParam; // usually search radius

    int nPelSearch; // search radius at finest level

    int lsad;        // SAD limit for lambda using - added by Fizick
    int pnew;        // penalty to cost for new canditate - added by Fizick
    int plen;        // penalty factor (similar to lambda) for vector length - added by Fizick
    int plevel;      // penalty factors (lambda, plen) level scaling - added by Fizick
    int global;     // use global motion predictor
    int pglobal;     // penalty factor for global motion predictor
    int pzero;       // penalty factor for zero vector
    int divideExtra; // divide blocks on sublocks with median motion
    int64_t badSAD;  //  SAD threshold to make more wide search for bad vectors
    int badrange;    // range (radius) of wide search
    int meander;    //meander (alternate) scan blocks (even row left to right, odd row right to left
    int tryMany;    // try refine around many predictors

    int dctmode;

    int nModeYUV;

    int nSuperLevels;
    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;

    int levels;
    int searchparam;
    int chroma;
    int truemotion;

    int fields;
    int tff;
    int tff_exists;
} MVAnalyseData;


static void VS_CC mvanalyseInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVAnalyseData *d = (MVAnalyseData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvanalyseGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVAnalyseData *d = (MVAnalyseData *)*instanceData;

    if (activationReason == arInitial) {
        int nref;

        if (d->analysisData.nDeltaFrame > 0) {
            int offset = (d->analysisData.isBackward) ? d->analysisData.nDeltaFrame : -d->analysisData.nDeltaFrame;
            nref = n + offset;

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
        } else {                                 // special static mode
            nref = -d->analysisData.nDeltaFrame; // positive fixed frame number

            if (n < nref) {
                vsapi->requestFrameFilter(n, d->node, frameCtx);
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->node, frameCtx);
                vsapi->requestFrameFilter(n, d->node, frameCtx);
            }
        }
    } else if (activationReason == arAllFramesReady) {

        GroupOfPlanes vectorFields;

        gopInit(&vectorFields, d->analysisData.nBlkSizeX, d->analysisData.nBlkSizeY, d->analysisData.nLvCount, d->analysisData.nPel, d->analysisData.nMotionFlags, d->analysisData.nCPUFlags, d->analysisData.nOverlapX, d->analysisData.nOverlapY, d->analysisData.nBlkX, d->analysisData.nBlkY, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->divideExtra, d->supervi->format->bitsPerSample);


        const uint8_t *pSrc[3] = { NULL };
        const uint8_t *pRef[3] = { NULL };
        int nSrcPitch[3] = { 0 };
        int nRefPitch[3] = { 0 };

        int nref;

        if (d->analysisData.nDeltaFrame > 0) {
            int offset = (d->analysisData.isBackward) ? d->analysisData.nDeltaFrame : -d->analysisData.nDeltaFrame;
            nref = n + offset;
        } else {                                 // special static mode
            nref = -d->analysisData.nDeltaFrame; // positive fixed frame number
        }

        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        const VSMap *srcprops = vsapi->getFramePropsRO(src);
        int err;

        int src_top_field = !!vsapi->propGetInt(srcprops, "_Field", 0, &err);
        if (err && d->fields && !d->tff_exists) {
            vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
            gopDeinit(&vectorFields);
            vsapi->freeFrame(src);
            return NULL;
        }

        // if tff was passed, it overrides _Field.
        if (d->tff_exists)
            src_top_field = d->tff ^ (n % 2);

        for (int plane = 0; plane < d->supervi->format->numPlanes; plane++) {
            pSrc[plane] = vsapi->getReadPtr(src, plane);
            nSrcPitch[plane] = vsapi->getStride(src, plane);
        }


        int vectors_size = gopGetArraySize(&vectorFields) * sizeof(int);
        int *vectors = (int *)malloc(vectors_size);


        if (nref >= 0 && nref < d->vi->numFrames) {
            const VSFrameRef *ref = vsapi->getFrameFilter(nref, d->node, frameCtx);
            const VSMap *refprops = vsapi->getFramePropsRO(ref);

            int ref_top_field = !!vsapi->propGetInt(refprops, "_Field", 0, &err);
            if (err && d->fields && !d->tff_exists) {
                vsapi->setFilterError("Analyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                gopDeinit(&vectorFields);
                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);
                free(vectors);
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

            for (int plane = 0; plane < d->supervi->format->numPlanes; plane++) {
                pRef[plane] = vsapi->getReadPtr(ref, plane);
                nRefPitch[plane] = vsapi->getStride(ref, plane);
            }


            MVGroupOfFrames pSrcGOF, pRefGOF;

            mvgofInit(&pSrcGOF, d->nSuperLevels, d->analysisData.nWidth, d->analysisData.nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->opt, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->supervi->format->bitsPerSample);
            mvgofInit(&pRefGOF, d->nSuperLevels, d->analysisData.nWidth, d->analysisData.nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->opt, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->supervi->format->bitsPerSample);

            // cast away the const, because why not.
            mvgofUpdate(&pSrcGOF, (uint8_t **)pSrc, nSrcPitch);
            mvgofUpdate(&pRefGOF, (uint8_t **)pRef, nRefPitch);


            DCTFFTW *DCTc = NULL;
            if (d->dctmode != 0) {
                DCTc = (DCTFFTW *)malloc(sizeof(DCTFFTW));
                dctInit(DCTc, d->analysisData.nBlkSizeX, d->analysisData.nBlkSizeY, d->dctmode, d->supervi->format->bitsPerSample);
            }


            gopSearchMVs(&vectorFields, &pSrcGOF, &pRefGOF, d->searchType, d->nSearchParam, d->nPelSearch, d->nLambda, d->lsad, d->pnew, d->plevel, d->global, vectors, fieldShift, DCTc, d->pzero, d->pglobal, d->badSAD, d->badrange, d->meander, d->tryMany, d->searchTypeCoarse);

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
        } else { // too close to the beginning or end to do anything
            gopWriteDefaultToArray(&vectorFields, vectors);
            gopDeinit(&vectorFields);
        }

        VSFrameRef *dst = vsapi->copyFrame(src, core);
        VSMap *dstprops = vsapi->getFramePropsRW(dst);

        vsapi->propSetData(dstprops,
                           prop_MVTools_MVAnalysisData,
                           (const char *)(d->divideExtra ? &d->analysisDataDivided : &d->analysisData),
                           sizeof(MVAnalysisData),
                           paReplace);

        vsapi->propSetData(dstprops,
                           prop_MVTools_vectors,
                           (const char *)vectors,
                           vectors_size,
                           paReplace);

        free(vectors);

        // FIXME: Get rid of all mmx shit.
        mvtools_cpu_emms();

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvanalyseFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVAnalyseData *d = (MVAnalyseData *)instanceData;

    vsapi->freeNode(d->node);
    free(d);
}


static void VS_CC mvanalyseCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVAnalyseData d;
    MVAnalyseData *data;

    int err;

    d.analysisData.nBlkSizeX = int64ToIntS(vsapi->propGetInt(in, "blksize", 0, &err));
    if (err)
        d.analysisData.nBlkSizeX = 8;

    d.analysisData.nBlkSizeY = int64ToIntS(vsapi->propGetInt(in, "blksizev", 0, &err));
    if (err)
        d.analysisData.nBlkSizeY = d.analysisData.nBlkSizeX;

    d.levels = int64ToIntS(vsapi->propGetInt(in, "levels", 0, &err));

    d.searchType = (SearchType)int64ToIntS(vsapi->propGetInt(in, "search", 0, &err));
    if (err)
        d.searchType = SearchHex2;

    d.searchTypeCoarse = (SearchType)int64ToIntS(vsapi->propGetInt(in, "search_coarse", 0, &err));
    if (err)
        d.searchTypeCoarse = SearchExhaustive;

    d.searchparam = int64ToIntS(vsapi->propGetInt(in, "searchparam", 0, &err));
    if (err)
        d.searchparam = 2;

    d.nPelSearch = int64ToIntS(vsapi->propGetInt(in, "pelsearch", 0, &err));

    d.analysisData.isBackward = !!vsapi->propGetInt(in, "isb", 0, &err);

    d.chroma = !!vsapi->propGetInt(in, "chroma", 0, &err);
    if (err)
        d.chroma = 1;

    d.analysisData.nDeltaFrame = int64ToIntS(vsapi->propGetInt(in, "delta", 0, &err));
    if (err)
        d.analysisData.nDeltaFrame = 1;

    d.truemotion = !!vsapi->propGetInt(in, "truemotion", 0, &err);
    if (err)
        d.truemotion = 1;

    d.nLambda = int64ToIntS(vsapi->propGetInt(in, "lambda", 0, &err));
    if (err)
        d.nLambda = d.truemotion ? (1000 * d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY / 64) : 0;

    d.lsad = int64ToIntS(vsapi->propGetInt(in, "lsad", 0, &err));
    if (err)
        d.lsad = d.truemotion ? 1200 : 400;

    d.plevel = int64ToIntS(vsapi->propGetInt(in, "plevel", 0, &err));
    if (err)
        d.plevel = d.truemotion ? 1 : 0;

    d.global = !!vsapi->propGetInt(in, "global", 0, &err);
    if (err)
        d.global = d.truemotion ? 1 : 0;

    d.pnew = int64ToIntS(vsapi->propGetInt(in, "pnew", 0, &err));
    if (err)
        d.pnew = d.truemotion ? 50 : 0; // relative to 256

    d.pzero = int64ToIntS(vsapi->propGetInt(in, "pzero", 0, &err));
    if (err)
        d.pzero = d.pnew;

    d.pglobal = int64ToIntS(vsapi->propGetInt(in, "pglobal", 0, &err));

    d.analysisData.nOverlapX = int64ToIntS(vsapi->propGetInt(in, "overlap", 0, &err));

    d.analysisData.nOverlapY = int64ToIntS(vsapi->propGetInt(in, "overlapv", 0, &err));
    if (err)
        d.analysisData.nOverlapY = d.analysisData.nOverlapX;

    d.dctmode = int64ToIntS(vsapi->propGetInt(in, "dct", 0, &err));

    d.divideExtra = int64ToIntS(vsapi->propGetInt(in, "divide", 0, &err));

    d.badSAD = int64ToIntS(vsapi->propGetInt(in, "badsad", 0, &err));
    if (err)
        d.badSAD = 10000;

    d.badrange = int64ToIntS(vsapi->propGetInt(in, "badrange", 0, &err));
    if (err)
        d.badrange = 24;

    d.opt = !!vsapi->propGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;

    d.meander = !!vsapi->propGetInt(in, "meander", 0, &err);
    if (err)
        d.meander = 1;

    d.tryMany = !!vsapi->propGetInt(in, "trymany", 0, &err);

    d.fields = !!vsapi->propGetInt(in, "fields", 0, &err);

    d.tff = !!vsapi->propGetInt(in, "tff", 0, &err);
    d.tff_exists = !err;


    if (d.searchType < 0 || d.searchType > 7) {
        vsapi->setError(out, "Analyse: search must be between 0 and 7 (inclusive).");
        return;
    }

    if (d.searchTypeCoarse < 0 || d.searchTypeCoarse > 7) {
        vsapi->setError(out, "Analyse: search_coarse must be between 0 and 7 (inclusive).");
        return;
    }

    if (d.dctmode < 0 || d.dctmode > 10) {
        vsapi->setError(out, "Analyse: dct must be between 0 and 10 (inclusive).");
        return;
    }

    if (d.dctmode >= 5 &&
        !((d.analysisData.nBlkSizeX == 4 && d.analysisData.nBlkSizeY == 4) ||
          (d.analysisData.nBlkSizeX == 8 && d.analysisData.nBlkSizeY == 4) ||
          (d.analysisData.nBlkSizeX == 8 && d.analysisData.nBlkSizeY == 8) ||
          (d.analysisData.nBlkSizeX == 16 && d.analysisData.nBlkSizeY == 8) ||
          (d.analysisData.nBlkSizeX == 16 && d.analysisData.nBlkSizeY == 16))) {
        vsapi->setError(out, "Analyse: dct 5..10 can only work with 4x4, 8x4, 8x8, 16x8, and 16x16 blocks.");
        return;
    }

    if (d.divideExtra < 0 || d.divideExtra > 2) {
        vsapi->setError(out, "Analyse: divide must be between 0 and 2 (inclusive).");
        return;
    }


    if ((d.analysisData.nBlkSizeX != 4 || d.analysisData.nBlkSizeY != 4) &&
        (d.analysisData.nBlkSizeX != 8 || d.analysisData.nBlkSizeY != 4) &&
        (d.analysisData.nBlkSizeX != 8 || d.analysisData.nBlkSizeY != 8) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 2) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 8) &&
        (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 16) &&
        (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 32) &&
        (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 16)) {

        vsapi->setError(out, "Analyse: the block size must be 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, or 32x32.");
        return;
    }


    if (d.plevel < 0 || d.plevel > 2) {
        vsapi->setError(out, "Analyse: plevel must be between 0 and 2 (inclusive).");
        return;
    }


    if (d.pnew < 0 || d.pnew > 256) {
        vsapi->setError(out, "Analyse: pnew must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.pzero < 0 || d.pzero > 256) {
        vsapi->setError(out, "Analyse: pzero must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.pglobal < 0 || d.pglobal > 256) {
        vsapi->setError(out, "Analyse: pglobal must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.analysisData.nOverlapX < 0 || d.analysisData.nOverlapX > d.analysisData.nBlkSizeX / 2 ||
        d.analysisData.nOverlapY < 0 || d.analysisData.nOverlapY > d.analysisData.nBlkSizeY / 2) {
        vsapi->setError(out, "Analyse: overlap must be at most half of blksize, overlapv must be at most half of blksizev, and they both need to be at least 0.");
        return;
    }

    if (d.divideExtra && (d.analysisData.nBlkSizeX < 8 || d.analysisData.nBlkSizeY < 8)) {
        vsapi->setError(out, "Analyse: blksize and blksizev must be at least 8 when divide=True.");
        return;
    }


    if (d.searchType == SearchNstep)
        d.nSearchParam = (d.searchparam < 0) ? 0 : d.searchparam;
    else
        d.nSearchParam = (d.searchparam < 1) ? 1 : d.searchparam;


    d.node = vsapi->propGetNode(in, "super", 0, 0);
    d.supervi = vsapi->getVideoInfo(d.node);
    d.vi = d.supervi;

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, "Analyse: Input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant format and dimensions.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.vi->format->colorFamily == cmGray)
        d.chroma = 0;

    d.nModeYUV = d.chroma ? YUVPLANES : YPLANE;


    d.analysisData.bitsPerSample = d.vi->format->bitsPerSample;

    int pixelMax = (1 << d.vi->format->bitsPerSample) - 1;
    d.lsad = (int)((double)d.lsad * pixelMax / 255.0 + 0.5);
    d.badSAD = (int)((double)d.badSAD * pixelMax / 255.0 + 0.5);

    d.lsad = d.lsad * (d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY) / 64;
    d.badSAD = d.badSAD * (d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY) / 64;


    d.analysisData.nMotionFlags = 0;
    d.analysisData.nMotionFlags |= d.opt ? MOTION_USE_SIMD : 0;
    d.analysisData.nMotionFlags |= d.analysisData.isBackward ? MOTION_IS_BACKWARD : 0;
    d.analysisData.nMotionFlags |= d.chroma ? MOTION_USE_CHROMA_MOTION : 0;


    if (d.opt) {
        d.analysisData.nCPUFlags = cpu_detect();
    }

    if (d.vi->format->bitsPerSample > 8)
        d.opt = 0; // needed here because MVPlane can't have opt=1 with more than 8 bits

    if (d.analysisData.nOverlapX % (1 << d.vi->format->subSamplingW) ||
        d.analysisData.nOverlapY % (1 << d.vi->format->subSamplingH)) {
        vsapi->setError(out, "Analyse: The requested overlap is incompatible with the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.divideExtra && (d.analysisData.nOverlapX % (2 << d.vi->format->subSamplingW) ||
                          d.analysisData.nOverlapY % (2 << d.vi->format->subSamplingH))) { // subsampling times 2
        vsapi->setError(out, "Analyse: overlap and overlapv must be multiples of 2 or 4 when divide=True, depending on the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.analysisData.nDeltaFrame <= 0 && (-d.analysisData.nDeltaFrame) >= d.vi->numFrames) {
        vsapi->setError(out, "Analyse: delta points to frame past the input clip's end.");
        vsapi->freeNode(d.node);
        return;
    }

    d.analysisData.yRatioUV = 1 << d.vi->format->subSamplingH;
    d.analysisData.xRatioUV = 1 << d.vi->format->subSamplingW;


#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Analyse: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrameRef *evil = vsapi->getFrame(0, d.node, errorMsg + errorLen, ERROR_SIZE - errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->setError(out, errorMsg);
        vsapi->freeNode(d.node);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[6];
    int nHeight = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    d.nSuperVPad = int64ToIntS(vsapi->propGetInt(props, "Super_vpad", 0, &evil_err[2]));
    d.nSuperPel = int64ToIntS(vsapi->propGetInt(props, "Super_pel", 0, &evil_err[3]));
    d.nSuperModeYUV = int64ToIntS(vsapi->propGetInt(props, "Super_modeyuv", 0, &evil_err[4]));
    d.nSuperLevels = int64ToIntS(vsapi->propGetInt(props, "Super_levels", 0, &evil_err[5]));
    vsapi->freeFrame(evil);

    for (int i = 0; i < 6; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "Analyse: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.node);
            return;
        }

    // check sanity
    if (nHeight <= 0 || d.nSuperHPad < 0 || d.nSuperHPad >= d.vi->width / 2 ||
        d.nSuperVPad < 0 || d.nSuperPel < 1 || d.nSuperPel > 4 ||
        d.nSuperModeYUV < 0 || d.nSuperModeYUV > YUVPLANES || d.nSuperLevels < 1) {
        vsapi->setError(out, "Analyse: parameters from super clip appear to be wrong.");
        vsapi->freeNode(d.node);
        return;
    }

    if ((d.nModeYUV & d.nSuperModeYUV) != d.nModeYUV) { //x
        vsapi->setError(out, "Analyse: super clip does not contain needed colour data.");
        vsapi->freeNode(d.node);
        return;
    }


    // fill in missing fields
    d.analysisData.nWidth = d.vi->width - d.nSuperHPad * 2; //x

    d.analysisData.nHeight = nHeight; //x

    d.analysisData.nPel = d.nSuperPel; //x

    d.analysisData.nHPadding = d.nSuperHPad; //v2.0    //x
    d.analysisData.nVPadding = d.nSuperVPad;


    int nBlkX = (d.analysisData.nWidth - d.analysisData.nOverlapX) / (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX); //x

    int nBlkY = (d.analysisData.nHeight - d.analysisData.nOverlapY) / (d.analysisData.nBlkSizeY - d.analysisData.nOverlapY);

    d.analysisData.nBlkX = nBlkX;
    d.analysisData.nBlkY = nBlkY;

    int nWidth_B = (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX) * nBlkX + d.analysisData.nOverlapX; // covered by blocks
    int nHeight_B = (d.analysisData.nBlkSizeY - d.analysisData.nOverlapY) * nBlkY + d.analysisData.nOverlapY;

    // calculate valid levels
    int nLevelsMax = 0;
    while (((nWidth_B >> nLevelsMax) - d.analysisData.nOverlapX) / (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX) > 0 &&
           ((nHeight_B >> nLevelsMax) - d.analysisData.nOverlapY) / (d.analysisData.nBlkSizeY - d.analysisData.nOverlapY) > 0) // at last one block
    {
        nLevelsMax++;
    }

    d.analysisData.nLvCount = d.levels > 0 ? d.levels : nLevelsMax + d.levels;

    if (d.analysisData.nLvCount < 1 || d.analysisData.nLvCount > nLevelsMax) {
        vsapi->setError(out, "Analyse: invalid number of levels.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.analysisData.nLvCount > d.nSuperLevels) { //x
#define ERROR_SIZE 512
        char error_msg[ERROR_SIZE + 1] = { 0 };
        snprintf(error_msg, ERROR_SIZE, "Analyse: super clip has %d levels. Analyse needs %d levels.", d.nSuperLevels, d.analysisData.nLvCount);
#undef ERROR_SIZE
        vsapi->setError(out, error_msg);
        vsapi->freeNode(d.node);
        return;
    }


    if (d.nPelSearch <= 0)
        d.nPelSearch = d.analysisData.nPel; // not below value of 0 at finest level //x


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


    data = (MVAnalyseData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Analyse", mvanalyseInit, mvanalyseGetFrame, mvanalyseFree, fmParallel, 0, data, core);
}


void mvanalyseRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Analyse",
                 "super:clip;"
                 "blksize:int:opt;"
                 "blksizev:int:opt;"
                 "levels:int:opt;"
                 "search:int:opt;"
                 "searchparam:int:opt;"
                 "pelsearch:int:opt;"
                 "isb:int:opt;"
                 "lambda:int:opt;"
                 "chroma:int:opt;"
                 "delta:int:opt;"
                 "truemotion:int:opt;"
                 "lsad:int:opt;"
                 "plevel:int:opt;"
                 "global:int:opt;"
                 "pnew:int:opt;"
                 "pzero:int:opt;"
                 "pglobal:int:opt;"
                 "overlap:int:opt;"
                 "overlapv:int:opt;"
                 "divide:int:opt;"
                 "badsad:int:opt;"
                 "badrange:int:opt;"
                 "opt:int:opt;"
                 "meander:int:opt;"
                 "trymany:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 "search_coarse:int:opt;"
                 "dct:int:opt;",
                 mvanalyseCreate, 0, plugin);
}
