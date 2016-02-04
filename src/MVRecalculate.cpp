#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "CPU.h"
#include "DCTFFTW.h"
#include "GroupOfPlanes.h"
#include "MVInterface.h"


// FIXME: Redundant members. A few can go straight in analysisData.
typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;
    const VSVideoInfo *supervi;

    MVClipDicks *mvClip;

    MVAnalysisData analysisData;
    MVAnalysisData analysisDataDivided;

    /*! \brief isse optimisations enabled */
    bool isse;

    /*! \brief motion vecteur cost factor */
    int nLambda;

    /*! \brief search type chosen for refinement in the EPZ */
    SearchType searchType;

    /*! \brief additionnal parameter for this search */
    int nSearchParam; // usually search radius


    int pnew; // penalty to cost for new canditate - added by Fizick
    int plen; // penalty factor (similar to lambda) for vector length - added by Fizick
    int divideExtra; // divide blocks on sublocks with median motion
    bool meander; //meander (alternate) scan blocks (even row left to right, odd row right to left

    int dctmode;

    int nModeYUV;

    int headerSize;

    int nSuperLevels;
    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;

    int blksize;
    int blksizev;
    int search;
    int searchparam;
    bool chroma;
    bool truemotion;
    int overlap;
    int overlapv;
    int smooth;
    int thSAD;
    VSNodeRef *vectors;

    bool fields;
    bool tff;
    int tffexists;
} MVRecalculateData;


static void VS_CC mvrecalculateInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVRecalculateData *d = (MVRecalculateData *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvrecalculateGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVRecalculateData *d = (MVRecalculateData *) * instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        int offset = ( d->analysisData.isBackward ) ? d->analysisData.nDeltaFrame : -d->analysisData.nDeltaFrame;
        int nref = n + offset;

        if (nref >= 0 && (nref < d->vi->numFrames || !d->vi->numFrames)) {
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

        GroupOfPlanes *vectorFields = new GroupOfPlanes(d->analysisData.nBlkSizeX, d->analysisData.nBlkSizeY, d->analysisData.nLvCount, d->analysisData.nPel, d->analysisData.nMotionFlags, d->analysisData.nCPUFlags, d->analysisData.nOverlapX, d->analysisData.nOverlapY, d->analysisData.nBlkX, d->analysisData.nBlkY, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->divideExtra, d->supervi->format->bitsPerSample);


        const uint8_t *pSrc[3] = { nullptr };
        const uint8_t *pRef[3] = { nullptr };
        uint8_t *pDst = { nullptr };
        int nSrcPitch[3] = { 0 };
        int nRefPitch[3] = { 0 };

        int offset = ( d->analysisData.isBackward ) ? d->analysisData.nDeltaFrame : -d->analysisData.nDeltaFrame;
        int nref = n + offset;

        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        const VSMap *srcprops = vsapi->getFramePropsRO(src);
        int err;

        bool srctff = !!vsapi->propGetInt(srcprops, "_Field", 0, &err);
        if (err && d->fields && !d->tffexists) {
            vsapi->setFilterError("Recalculate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
            delete vectorFields;
            vsapi->freeFrame(src);
            return NULL;
        }

        // if tff was passed, it overrides _Field.
        if (d->tffexists)
            srctff = d->tff && (n % 2 == 0); //child->GetParity(n); // bool tff;

        for (int plane = 0; plane < d->supervi->format->numPlanes; plane++) {
            pSrc[plane] = vsapi->getReadPtr(src, plane);
            nSrcPitch[plane] = vsapi->getStride(src, plane);
        }

        int dst_height = 1;
        int dst_width = d->headerSize / sizeof(int) + vectorFields->GetArraySize(); //v1.8.1
        // In Avisynth the frame was packed BGR32, which has 4 bytes per pixel.
        // It's GRAY8 here.
        dst_width *= 4;
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, dst_width, dst_height, src, core);

        pDst = vsapi->getWritePtr(dst, 0);

        // write analysis parameters as a header to frame
        memcpy(pDst, &d->headerSize, sizeof(int));

        if (d->divideExtra)
            memcpy(pDst+sizeof(int), &d->analysisDataDivided, sizeof(d->analysisData));
        else
            memcpy(pDst+sizeof(int), &d->analysisData, sizeof(d->analysisData));

        pDst += d->headerSize;

        const VSFrameRef *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        MVClipBalls balls(d->mvClip, vsapi);
        balls.Update(mvn);
        vsapi->freeFrame(mvn);

        if (balls.IsUsable() && nref >= 0 && (nref < d->vi->numFrames || !d->vi->numFrames))
        {
            const VSFrameRef *ref = vsapi->getFrameFilter(nref, d->node, frameCtx);
            const VSMap *refprops = vsapi->getFramePropsRO(ref);

            bool reftff = !!vsapi->propGetInt(refprops, "_Field", 0, &err);
            if (err && d->fields && !d->tffexists) {
                vsapi->setFilterError("Recalculate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                delete vectorFields;
                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);
                vsapi->freeFrame(dst);
                return NULL;
            }

            // if tff was passed, it overrides _Field.
            if (d->tffexists)
                reftff = d->tff && (nref % 2 == 0); //child->GetParity(n); // bool tff;

            int fieldShift = 0;
            if (d->fields && d->analysisData.nPel > 1 && (d->analysisData.nDeltaFrame % 2))
            {
                fieldShift = (srctff && !reftff) ? d->analysisData.nPel/2 : ( (reftff && !srctff) ? -(d->analysisData.nPel/2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }


            for (int plane = 0; plane < d->supervi->format->numPlanes; plane++) {
                pRef[plane] = vsapi->getReadPtr(ref, plane);
                nRefPitch[plane] = vsapi->getStride(ref, plane);
            }


            MVGroupOfFrames *pSrcGOF = new MVGroupOfFrames(d->nSuperLevels, d->analysisData.nWidth, d->analysisData.nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->isse, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->supervi->format->bitsPerSample);
            MVGroupOfFrames *pRefGOF = new MVGroupOfFrames(d->nSuperLevels, d->analysisData.nWidth, d->analysisData.nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->isse, d->analysisData.xRatioUV, d->analysisData.yRatioUV, d->supervi->format->bitsPerSample);

            // cast away the const, because why not.
            pSrcGOF->Update(d->nModeYUV, (uint8_t *)pSrc[0], nSrcPitch[0], (uint8_t *)pSrc[1], nSrcPitch[1], (uint8_t *)pSrc[2], nSrcPitch[2]); // v2.0
            pRefGOF->Update(d->nModeYUV, (uint8_t *)pRef[0], nRefPitch[0], (uint8_t *)pRef[1], nRefPitch[1], (uint8_t *)pRef[2], nRefPitch[2]); // v2.0


            DCTClass *DCTc = NULL;
            if (d->dctmode != 0) {
                /*
                // FIXME: deal with this inline asm shit
                if (d->isse && (d->blksize == 8) && d->blksizev == 8)
                DCTc = new DCTINT(d->blksize, d->blksizev, d->dctmode);
                else
                */
                DCTc = new DCTFFTW(d->blksize, d->blksizev, d->dctmode, d->supervi->format->bitsPerSample);
            }


            vectorFields->RecalculateMVs(balls, pSrcGOF, pRefGOF, d->searchType, d->nSearchParam, d->nLambda, d->pnew, reinterpret_cast<int*>(pDst), NULL, fieldShift, d->thSAD, DCTc, d->smooth, d->meander);

            if (d->divideExtra) {
                // make extra level with divided sublocks with median (not estimated) motion
                vectorFields->ExtraDivide(reinterpret_cast<int*>(pDst));
            }

            delete vectorFields;
            if (DCTc)
                delete DCTc;
            delete pSrcGOF;
            delete pRefGOF;
            vsapi->freeFrame(ref);
        }
        else // too close to the beginning or end to do anything
        {
            vectorFields->WriteDefaultToArray(reinterpret_cast<int*>(pDst));
            delete vectorFields;
        }

        // FIXME: Get rid of all mmx shit.
        mvtools_cpu_emms();

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvrecalculateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVRecalculateData *d = (MVRecalculateData *)instanceData;

    vsapi->freeNode(d->node);
    vsapi->freeNode(d->vectors);
    delete d->mvClip;
    free(d);
}


static void VS_CC mvrecalculateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVRecalculateData d;
    MVRecalculateData *data;

    int err;

    d.thSAD = int64ToIntS(vsapi->propGetInt(in, "thsad", 0, &err));
    if (err)
        d.thSAD = 200;

    d.smooth = int64ToIntS(vsapi->propGetInt(in, "smooth", 0, &err));
    if (err)
        d.smooth = 1;

    d.blksize = int64ToIntS(vsapi->propGetInt(in, "blksize", 0, &err));
    if (err)
        d.blksize = 8;

    d.blksizev = int64ToIntS(vsapi->propGetInt(in, "blksizev", 0, &err));
    if (err)
        d.blksizev = d.blksize;

    d.search = int64ToIntS(vsapi->propGetInt(in, "search", 0, &err));
    if (err)
        d.search = 4;

    d.searchparam = int64ToIntS(vsapi->propGetInt(in, "searchparam", 0, &err));
    if (err)
        d.searchparam = 2;

    d.chroma = !!vsapi->propGetInt(in, "chroma", 0, &err);
    if (err)
        d.chroma = 1;

    d.truemotion = !!vsapi->propGetInt(in, "truemotion", 0, &err);
    if (err)
        d.truemotion = 1;

    d.nLambda = int64ToIntS(vsapi->propGetInt(in, "lambda", 0, &err));
    if (err)
        d.nLambda = d.truemotion ? (1000 * d.blksize * d.blksizev / 64) : 0;

    d.pnew = int64ToIntS(vsapi->propGetInt(in, "pnew", 0, &err));
    if (err)
        d.pnew = d.truemotion ? 50 : 0; // relative to 256

    d.overlap = int64ToIntS(vsapi->propGetInt(in, "overlap", 0, &err));

    d.overlapv = int64ToIntS(vsapi->propGetInt(in, "overlapv", 0, &err));
    if (err)
        d.overlapv = d.overlap;

    d.dctmode = int64ToIntS(vsapi->propGetInt(in, "dct", 0, &err));

    d.divideExtra = int64ToIntS(vsapi->propGetInt(in, "divide", 0, &err));

    d.isse = !!vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;

    d.meander = !!vsapi->propGetInt(in, "meander", 0, &err);
    if (err)
        d.meander = 1;

    d.fields = !!vsapi->propGetInt(in, "fields", 0, &err);

    d.tff = !!vsapi->propGetInt(in, "tff", 0, &err);
    d.tffexists = err;


    if (d.search < 0 || d.search > 7) {
        vsapi->setError(out, "Recalculate: search must be between 0 and 7 (inclusive).");
        return;
    }

    if (d.dctmode < 0 || d.dctmode > 10) {
        vsapi->setError(out, "Recalculate: dct must be between 0 and 10 (inclusive).");
        return;
    }

    if (d.dctmode >= 5 && !((d.blksize == 4 && d.blksizev == 4) ||
                (d.blksize == 8 && d.blksizev == 4) ||
                (d.blksize == 8 && d.blksizev == 8) ||
                (d.blksize == 16 && d.blksizev == 8) ||
                (d.blksize == 16 && d.blksizev == 16))) {
        vsapi->setError(out, "Recalculate: dct 5..10 can only work with 4x4, 8x4, 8x8, 16x8, and 16x16 blocks.");
        return;
    }

    if (d.divideExtra < 0 || d.divideExtra > 2) {
        vsapi->setError(out, "Recalculate: divide must be between 0 and 2 (inclusive).");
        return;
    }


    d.analysisData.nBlkSizeX = d.blksize;
    d.analysisData.nBlkSizeY = d.blksizev;
    if ((d.analysisData.nBlkSizeX != 4  || d.analysisData.nBlkSizeY != 4) &&
            (d.analysisData.nBlkSizeX != 8  || d.analysisData.nBlkSizeY != 4) &&
            (d.analysisData.nBlkSizeX != 8  || d.analysisData.nBlkSizeY != 8) &&
            (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 2) &&
            (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 8) &&
            (d.analysisData.nBlkSizeX != 16 || d.analysisData.nBlkSizeY != 16) &&
            (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 32) &&
            (d.analysisData.nBlkSizeX != 32 || d.analysisData.nBlkSizeY != 16)) {

        vsapi->setError(out, "Recalculate: the block size must be 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, or 32x32.");
        return;
    }


    if (d.pnew < 0 || d.pnew > 256) {
        vsapi->setError(out, "Recalculate: pnew must be between 0 and 256 (inclusive).");
        return;
    }


    if (d.overlap < 0 || d.overlap > d.blksize / 2 ||
        d.overlapv < 0 || d.overlapv > d.blksizev / 2) {
        vsapi->setError(out, "Recalculate: overlap must be at most half of blksize, overlapv must be at most half of blksizev, and they both need to be at least 0.");
        return;
    }

    if (d.divideExtra && (d.blksize < 8 && d.blksizev < 8) ) {
        vsapi->setError(out, "Recalculate: blksize and blksizev must be at least 8 when divide=True.");
        return;
    }

    d.analysisData.nOverlapX = d.overlap;
    d.analysisData.nOverlapY = d.overlapv;

    SearchType searchTypes[] = { ONETIME, NSTEP, LOGARITHMIC, EXHAUSTIVE, HEX2SEARCH, UMHSEARCH, HSEARCH, VSEARCH };
    d.searchType = searchTypes[d.search];

    if (d.searchType == NSTEP)
        d.nSearchParam = ( d.searchparam < 0 ) ? 0 : d.searchparam;
    else
        d.nSearchParam = ( d.searchparam < 1 ) ? 1 : d.searchparam;


    // XXX maybe get rid of these two
    // Bleh, they're checked by client filters. Though it's kind of pointless.
    d.analysisData.nMagicKey = MOTION_MAGIC_KEY;
    d.analysisData.nVersion = MVANALYSIS_DATA_VERSION; // MVAnalysisData and outfile format version: last update v1.8.1


    d.headerSize = VSMAX(4 + sizeof(d.analysisData), 256); // include itself, but usually equal to 256 :-)


    d.node = vsapi->propGetNode(in, "super", 0, 0);
    d.supervi = vsapi->getVideoInfo(d.node);

    if (d.overlap % (1 << d.supervi->format->subSamplingW) ||
        d.overlapv % (1 << d.supervi->format->subSamplingH)) {
        vsapi->setError(out, "Recalculate: The requested overlap is incompatible with the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }

    if (d.divideExtra && (d.overlap % (2 << d.supervi->format->subSamplingW) ||
                          d.overlapv % (2 << d.supervi->format->subSamplingH))) { // subsampling times 2
        vsapi->setError(out, "Recalculate: overlap and overlapv must be multiples of 2 or 4 when divide=True, depending on the super clip's subsampling.");
        vsapi->freeNode(d.node);
        return;
    }


    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.node, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("Recalculate: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
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
            vsapi->setError(out, "Recalculate: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.node);
            return;
        }


    if (d.supervi->format->colorFamily == cmGray)
        d.chroma = 0;

    d.nModeYUV = d.chroma ? YUVPLANES : YPLANE;

    if ((d.nModeYUV & d.nSuperModeYUV) != d.nModeYUV) { //x
        vsapi->setError(out, "Recalculate: super clip does not contain needed colour data.");
        vsapi->freeNode(d.node);
        return;
    }

    d.vectors = vsapi->propGetNode(in, "vectors", 0, NULL);
    d.vi = vsapi->getVideoInfo(d.vectors);

    evil = vsapi->getFrame(0, d.vectors, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("Recalculate: failed to retrieve first frame from vectors clip. Error message: ").append(errorMsg).c_str());
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.vectors);
        return;
    }

    // XXX This really should be passed as a frame property.
    const MVAnalysisData *pAnalyseFilter = reinterpret_cast<const MVAnalysisData *>(vsapi->getReadPtr(evil, 0) + sizeof(int));

    d.analysisData.yRatioUV = pAnalyseFilter->GetYRatioUV();
    d.analysisData.xRatioUV = pAnalyseFilter->GetXRatioUV();

    d.analysisData.nWidth = pAnalyseFilter->GetWidth();
    d.analysisData.nHeight = pAnalyseFilter->GetHeight();

    d.analysisData.nDeltaFrame = pAnalyseFilter->GetDeltaFrame();
    d.analysisData.isBackward = pAnalyseFilter->IsBackward();
    vsapi->freeFrame(evil);


    d.analysisData.bitsPerSample = d.supervi->format->bitsPerSample;

    int pixelMax = (1 << d.supervi->format->bitsPerSample) - 1;
    d.thSAD = int((double)d.thSAD * pixelMax / 255.0 + 0.5);

    // normalize threshold to block size
    int referenceBlockSize = 8 * 8;
    d.thSAD = d.thSAD * (d.analysisData.nBlkSizeX * d.analysisData.nBlkSizeY) / referenceBlockSize;
    if (d.chroma)
        d.thSAD += d.thSAD / (d.analysisData.xRatioUV * d.analysisData.yRatioUV) * 2;


    d.analysisData.nMotionFlags = 0;
    d.analysisData.nMotionFlags |= d.isse ? MOTION_USE_ISSE : 0;
    d.analysisData.nMotionFlags |= d.analysisData.isBackward ? MOTION_IS_BACKWARD : 0;
    d.analysisData.nMotionFlags |= d.chroma ? MOTION_USE_CHROMA_MOTION : 0;


    if (d.isse)
    {
        d.analysisData.nCPUFlags = cpu_detect();
    }

    if (d.supervi->format->bitsPerSample > 8)
        d.isse = 0; // needed here because MVPlane can't have isse=1 with more than 8 bits

    d.analysisData.nPel = d.nSuperPel;//x

    int nSuperWidth = d.supervi->width;
    if (nHeight != d.analysisData.nHeight || nSuperWidth - 2 * d.nSuperHPad != d.analysisData.nWidth) {
        vsapi->setError(out, "Recalculate: wrong frame size.");
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.vectors);
        return;
    }

    d.analysisData.nHPadding = d.nSuperHPad; //v2.0    //x
    d.analysisData.nVPadding = d.nSuperVPad;


    int nBlkX = (d.analysisData.nWidth - d.analysisData.nOverlapX) / (d.analysisData.nBlkSizeX - d.analysisData.nOverlapX);//x
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

    try {
        d.mvClip = new MVClipDicks(d.vectors, 8 * 8 * 255, 255, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("Recalculate: ").append(e.what()).c_str());
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.vectors);
        return;
    }


    data = (MVRecalculateData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Recalculate", mvrecalculateInit, mvrecalculateGetFrame, mvrecalculateFree, fmParallel, 0, data, core);
}


extern "C" void mvrecalculateRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Recalculate",
            "super:clip;"
            "vectors:clip;"
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
            "isse:int:opt;"
            "meander:int:opt;"
            "fields:int:opt;"
            "tff:int:opt;"
            "dct:int:opt;"
            , mvrecalculateCreate, 0, plugin);
}
