// Pixels flow motion function
// Copyright(c)2005 A.G.Balakhnin aka Fizick

// See legal notice in Copying.txt for more information

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; version 2 of the License.
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "Bullshit.h"
#include "CommonFunctions.h"
#include "Fakery.h"
#include "MaskFun.h"
#include "MVAnalysisData.h"
#include "SimpleResize.h"
#include "CommonMacros.h"



enum FlowModes {
    Fetch = 0,
    Shift
};


typedef void (*FlowFunction)(uint8_t *pdst, int dst_pitch, const uint8_t *pref, int ref_pitch, int16_t *VXFull, int VXPitch, int16_t *VYFull, int VYPitch, int width, int height, int time256, int nPel);

typedef void * (*MemsetFunction)(void *ptr, int value, size_t bytes);


typedef struct MVFlowData {
    VSNode *clip;
    const VSVideoInfo *vi;

    VSNode *finest;
    VSNode *super;
    VSNode *vectors;

    int time256;
    int mode;
    int fields;
    int64_t thscd1;
    int thscd2;
    int opt;
    int tff;
    int tff_exists;

    MVAnalysisData vectors_data;

    int nSuperHPad;

    int nBlkXP;
    int nBlkYP;
    int nWidthP;
    int nHeightP;
    int nWidthPUV;
    int nHeightPUV;
    int nWidthUV;
    int nHeightUV;
    int nVPaddingUV;
    int nHPaddingUV;
    int VPitchY;
    int VPitchUV;

    int pixel_max;

    SimpleResize upsizer;
    SimpleResize upsizerUV;

    FlowFunction flow_function;
    MemsetFunction memset_function;
} MVFlowData;


template <typename PixelType>
static void flowFetch(uint8_t *pdst8, int dst_pitch, const uint8_t *pref8, int ref_pitch, int16_t *VXFull, int VXPitch, int16_t *VYFull, int VYPitch, int width, int height, int time256, int nPel) {
    const PixelType *pref = (const PixelType *)pref8;
    PixelType *pdst = (PixelType *)pdst8;

    dst_pitch /= sizeof(PixelType);
    ref_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    // fetch mode
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            // use interpolated image
            int vx = (VXFull[w] * time256 + 128) >> 8;
            int vy = (VYFull[w] * time256 + 128) >> 8;
            pdst[w] = pref[vy * ref_pitch + vx + (w << nPelLog)];
        }
        pref += ref_pitch << nPelLog;
        pdst += dst_pitch;
        VXFull += VXPitch;
        VYFull += VYPitch;
    }
}


template <typename PixelType>
static void flowShift(uint8_t *pdst8, int dst_pitch, const uint8_t *pref8, int ref_pitch, int16_t *VXFull, int VXPitch, int16_t *VYFull, int VYPitch, int width, int height, int time256, int nPel) {
    const PixelType *pref = (const PixelType *)pref8;
    PixelType *pdst = (PixelType *)pdst8;

    dst_pitch /= sizeof(PixelType);
    ref_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    int rounding = 128 << nPelLog;
    int shift = 8 + nPelLog;

    // shift mode
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            // very simple half-pixel using,  must be by image interpolation really (later)
            int vx = (-VXFull[w] * time256 + rounding) >> shift;
            int vy = (-VYFull[w] * time256 + rounding) >> shift;
            int href = h + vy;
            int wref = w + vx;
            if (href >= 0 && href < height && wref >= 0 && wref < width) // bound check if not padded
                pdst[vy * dst_pitch + vx + w] = pref[w << nPelLog];
        }
        pref += ref_pitch << nPelLog;
        pdst += dst_pitch;
        VXFull += VXPitch;
        VYFull += VYPitch;
    }
}


template <typename PixelType>
static void *flowMemset(void *ptrv, int value, size_t bytes) {
    size_t num = bytes / sizeof(PixelType);
    PixelType *ptr = (PixelType *)ptrv;

    while (num-- > 0)
        *ptr++ = (PixelType)value;

    return ptrv;
}


static const VSFrame *VS_CC mvflowGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    const MVFlowData *d = (const MVFlowData *)instanceData;

    if (activationReason == arInitial) {
        int nref;
        int offset = d->vectors_data.nDeltaFrame; // integer offset of reference frame

        if (offset > 0) {
            nref = d->vectors_data.isBackward ? n + offset : n - offset;
        } else {
            nref = -offset;
        }

        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        if (nref >= 0 && nref < d->vi->numFrames) {
            if (n < nref) {
                vsapi->requestFrameFilter(n, d->clip, frameCtx);
                vsapi->requestFrameFilter(n, d->finest, frameCtx);
                vsapi->requestFrameFilter(nref, d->finest, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->finest, frameCtx);
                vsapi->requestFrameFilter(n, d->finest, frameCtx);
                vsapi->requestFrameFilter(n, d->clip, frameCtx);
            }
        }

        vsapi->requestFrameFilter(n, d->clip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        int nref;
        int offset = d->vectors_data.nDeltaFrame; // integer offset of reference frame

        if (offset > 0) {
            nref = d->vectors_data.isBackward ? n + offset : n - offset;
        } else {
            nref = -offset;
        }

        uint8_t *pDst[3];
        const uint8_t *pRef[3];
        int nDstPitches[3];
        int nRefPitches[3];

        FakeGroupOfPlanes fgop;
        fgopInit(&fgop, &d->vectors_data);

        const VSFrame *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        const VSMap *mvprops = vsapi->getFramePropertiesRO(mvn);
        fgopUpdate(&fgop, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));
        vsapi->freeFrame(mvn);


        if (fgopIsUsable(&fgop, d->thscd1, d->thscd2)) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->finest, frameCtx); //  ref for  compensation
            VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, ref, core);

            for (int i = 0; i < d->vi->format.numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
            }

            const int nWidth = d->vectors_data.nWidth;
            const int nHeight = d->vectors_data.nHeight;
            const int nWidthUV = d->nWidthUV;
            const int nHeightUV = d->nHeightUV;
            const int nHeightP = d->nHeightP;
            const int nHeightPUV = d->nHeightPUV;
            const int xRatioUV = d->vectors_data.xRatioUV;
            const int yRatioUV = d->vectors_data.yRatioUV;
            const int nBlkX = d->vectors_data.nBlkX;
            const int nBlkY = d->vectors_data.nBlkY;
            const int nBlkXP = d->nBlkXP;
            const int nBlkYP = d->nBlkYP;
            const int nVPadding = d->vectors_data.nVPadding;
            const int nHPadding = d->vectors_data.nHPadding;
            const int nVPaddingUV = d->nVPaddingUV;
            const int nHPaddingUV = d->nHPaddingUV;
            const int nPel = d->vectors_data.nPel;
            const int time256 = d->time256;
            const int VPitchY = d->VPitchY;
            const int VPitchUV = d->VPitchUV;

            int bytesPerSample = d->vi->format.bytesPerSample;


            size_t full_size = nHeightP * VPitchY * sizeof(int16_t);
            size_t small_size = nBlkYP * nBlkXP * sizeof(int16_t);

            int16_t *VXFullY = (int16_t *)malloc(full_size);
            int16_t *VYFullY = (int16_t *)malloc(full_size);
            int16_t *VXSmallY = (int16_t *)malloc(small_size);
            int16_t *VYSmallY = (int16_t *)malloc(small_size);

            MakeVectorSmallMasks(&fgop, nBlkX, nBlkY, VXSmallY, nBlkXP, VYSmallY, nBlkXP);

            CheckAndPadSmallY(VXSmallY, VYSmallY, nBlkXP, nBlkYP, nBlkX, nBlkY);

            int fieldShift = 0;
            if (d->fields && nPel > 1 && ((nref - n) % 2 != 0)) {
                const VSFrame *src = vsapi->getFrameFilter(n, d->finest, frameCtx);
                int err;
                const VSMap *props = vsapi->getFramePropertiesRO(src);
                int src_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                vsapi->freeFrame(src);
                if (err && !d->tff_exists) {
                    vsapi->setFilterError("Flow: _Field property not found in super frame. Therefore, you must pass tff argument.", frameCtx);
                    fgopDeinit(&fgop);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }

                if (d->tff_exists)
                    src_top_field = d->tff ^ (n % 2);

                props = vsapi->getFramePropertiesRO(ref);
                int ref_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                if (err && !d->tff_exists) {
                    vsapi->setFilterError("Flow: _Field property not found in super frame. Therefore, you must pass tff argument.", frameCtx);
                    fgopDeinit(&fgop);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }

                if (d->tff_exists)
                    ref_top_field = d->tff ^ (nref % 2);

                fieldShift = (src_top_field && !ref_top_field) ? nPel / 2 : ((ref_top_field && !src_top_field) ? -(nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }

            for (int j = 0; j < nBlkYP; j++) {
                for (int i = 0; i < nBlkXP; i++) {
                    VYSmallY[j * nBlkXP + i] += fieldShift;
                }
            }

            d->upsizer.simpleResize_int16_t(&d->upsizer, VXFullY, VPitchY, VXSmallY, nBlkXP, 1);
            d->upsizer.simpleResize_int16_t(&d->upsizer, VYFullY, VPitchY, VYSmallY, nBlkXP, 0);

            int nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * bytesPerSample * nPel;
            int nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * bytesPerSample * nPel;

            if (d->mode == Shift)
                d->memset_function(pDst[0], d->pixel_max, nHeight * nDstPitches[0]);

            d->flow_function(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, nRefPitches[0],
                    VXFullY, VPitchY, VYFullY, VPitchY,
                    nWidth, nHeight, time256, nPel);

            if (d->vi->format.colorFamily != cfGray) {
                size_t full_size_uv = nHeightPUV * VPitchUV * sizeof(int16_t);
                size_t small_size_uv = nBlkYP * nBlkXP * sizeof(int16_t);

                int16_t *VXFullUV = (int16_t *)malloc(full_size_uv);
                int16_t *VYFullUV = (int16_t *)malloc(full_size_uv);

                int16_t *VXSmallUV = (int16_t *)malloc(small_size_uv);
                int16_t *VYSmallUV = (int16_t *)malloc(small_size_uv);

                VectorSmallMaskYToHalfUV(VXSmallY, nBlkXP, nBlkYP, VXSmallUV, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallY, nBlkXP, nBlkYP, VYSmallUV, yRatioUV);

                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VXFullUV, VPitchUV, VXSmallUV, nBlkXP, 1);
                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VYFullUV, VPitchUV, VYSmallUV, nBlkXP, 0);


                if (d->mode == Shift) {
                    d->memset_function(pDst[1], d->pixel_max, nHeightUV * nDstPitches[1]);
                    d->memset_function(pDst[2], d->pixel_max, nHeightUV * nDstPitches[2]);
                }

                d->flow_function(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, nRefPitches[1],
                        VXFullUV, VPitchUV, VYFullUV, VPitchUV,
                        nWidthUV, nHeightUV, time256, nPel);
                d->flow_function(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, nRefPitches[2],
                        VXFullUV, VPitchUV, VYFullUV, VPitchUV,
                        nWidthUV, nHeightUV, time256, nPel);

                free(VXFullUV);
                free(VYFullUV);
                free(VXSmallUV);
                free(VYSmallUV);
            }

            free(VXFullY);
            free(VYFullY);
            free(VXSmallY);
            free(VYSmallY);

            vsapi->freeFrame(ref);

            fgopDeinit(&fgop);

            return dst;
        } else { // not usable
            fgopDeinit(&fgop);

            return vsapi->getFrameFilter(n, d->clip, frameCtx);
        }
    }

    return 0;
}


static void VS_CC mvflowFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVFlowData *d = (MVFlowData *)instanceData;

    simpleDeinit(&d->upsizer);
    if (d->vi->format.colorFamily != cfGray)
        simpleDeinit(&d->upsizerUV);

    vsapi->freeNode(d->finest);
    vsapi->freeNode(d->super);
    vsapi->freeNode(d->vectors);
    vsapi->freeNode(d->clip);
    free(d);
}


static void VS_CC mvflowCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVFlowData d;
    MVFlowData *data;

    int err;

    double time = vsapi->mapGetFloat(in, "time", 0, &err);
    if (err)
        time = 100.0;

    d.mode = vsapi->mapGetIntSaturated(in, "mode", 0, &err);

    d.fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    d.thscd1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.opt = !!vsapi->mapGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 1;

    d.tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d.tff_exists = !err;


    if (time < 0.0 || time > 100.0) {
        vsapi->mapSetError(out, "Flow: time must be between 0 and 100 % (inclusive).");
        return;
    }

    if (d.mode < Fetch || d.mode > Shift) {
        vsapi->mapSetError(out, "Flow: mode must be 0 or 1.");
        return;
    }


    d.time256 = (int)(time * 256.0 / 100.0);


    d.super = vsapi->mapGetNode(in, "super", 0, NULL);

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Flow: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrame *evil = vsapi->getFrame(0, d.super, errorMsg + errorLen, ERROR_SIZE - errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->mapSetError(out, errorMsg);
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropertiesRO(evil);
    int evil_err[3];
    int nHeightS = vsapi->mapGetIntSaturated(props, "Super_height", 0, &evil_err[0]);
    d.nSuperHPad = vsapi->mapGetIntSaturated(props, "Super_hpad", 0, &evil_err[1]);
    int nSuperPel = vsapi->mapGetIntSaturated(props, "Super_pel", 0, &evil_err[2]);
    vsapi->freeFrame(evil);

    for (int i = 0; i < 2; i++)
        if (evil_err[i]) {
            vsapi->mapSetError(out, "Flow: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.vectors = vsapi->mapGetNode(in, "vectors", 0, NULL);

#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };
    const char *filter_name = "Flow";

    adataFromVectorClip(&d.vectors_data, d.vectors, filter_name, "vectors", vsapi, error, ERROR_SIZE);

    scaleThSCD(&d.thscd1, &d.thscd2, &d.vectors_data, filter_name, error, ERROR_SIZE);
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->mapSetError(out, error);

        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        return;
    }


    if (d.vectors_data.nPel == 1)
        d.finest = vsapi->addNodeRef(d.super);
    else {
        VSPlugin *mvtoolsPlugin = vsapi->getPluginByID("com.nodame.mvtools", core);

        VSMap *args = vsapi->createMap();
        vsapi->mapSetNode(args, "super", d.super, maReplace);
        vsapi->mapSetInt(args, "opt", d.opt, maReplace);
        VSMap *ret = vsapi->invoke(mvtoolsPlugin, "Finest", args);
        if (vsapi->mapGetError(ret)) {
#define ERROR_SIZE 512
            char error_msg[ERROR_SIZE + 1] = { 0 };
            snprintf(error_msg, ERROR_SIZE, "Flow: %s", vsapi->mapGetError(ret));
#undef ERROR_SIZE
            vsapi->mapSetError(out, error_msg);

            vsapi->freeNode(d.super);
            vsapi->freeNode(d.vectors);
            vsapi->freeMap(args);
            vsapi->freeMap(ret);
            return;
        }
        d.finest = vsapi->mapGetNode(ret, "clip", 0, NULL);
        vsapi->freeMap(args);
        vsapi->freeMap(ret);
    }

    d.clip = vsapi->mapGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.clip);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.vectors_data.nHeight != nHeightS || d.vectors_data.nWidth != nSuperWidth - d.nSuperHPad * 2 || d.vectors_data.nPel != nSuperPel) {
        vsapi->mapSetError(out, "Flow: wrong source or super clip frame size.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.clip);
        return;
    }

    if (!vsh::isConstantVideoFormat(d.vi) || d.vi->format.bitsPerSample > 16 || d.vi->format.sampleType != stInteger || d.vi->format.subSamplingW > 1 || d.vi->format.subSamplingH > 1 || (d.vi->format.colorFamily != cfYUV && d.vi->format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, "Flow: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.clip);
        return;
    }

    d.nBlkXP = d.vectors_data.nBlkX;
    d.nBlkYP = d.vectors_data.nBlkY;
    while (d.nBlkXP * (d.vectors_data.nBlkSizeX - d.vectors_data.nOverlapX) + d.vectors_data.nOverlapX < d.vectors_data.nWidth)
        d.nBlkXP++;
    while (d.nBlkYP * (d.vectors_data.nBlkSizeY - d.vectors_data.nOverlapY) + d.vectors_data.nOverlapY < d.vectors_data.nHeight)
        d.nBlkYP++;

    d.nWidthP = d.nBlkXP * (d.vectors_data.nBlkSizeX - d.vectors_data.nOverlapX) + d.vectors_data.nOverlapX; /// can be a local
    d.nHeightP = d.nBlkYP * (d.vectors_data.nBlkSizeY - d.vectors_data.nOverlapY) + d.vectors_data.nOverlapY;

    d.nWidthPUV = d.nWidthP / d.vectors_data.xRatioUV; /// can be a local
    d.nHeightPUV = d.nHeightP / d.vectors_data.yRatioUV;
    d.nHeightUV = d.vectors_data.nHeight / d.vectors_data.yRatioUV;
    d.nWidthUV = d.vectors_data.nWidth / d.vectors_data.xRatioUV;

    d.nHPaddingUV = d.vectors_data.nHPadding / d.vectors_data.xRatioUV;
    d.nVPaddingUV = d.vectors_data.nVPadding / d.vectors_data.yRatioUV;

    d.VPitchY = (d.nWidthP + 15) & (~15);
    d.VPitchUV = (d.nWidthPUV + 15) & (~15);


    d.pixel_max = (1 << d.vi->format.bitsPerSample) - 1;


    simpleInit(&d.upsizer, d.nWidthP, d.nHeightP, d.nBlkXP, d.nBlkYP, d.vectors_data.nWidth, d.vectors_data.nHeight, d.vectors_data.nPel, d.opt);
    if (d.vi->format.colorFamily != cfGray)
        simpleInit(&d.upsizerUV, d.nWidthPUV, d.nHeightPUV, d.nBlkXP, d.nBlkYP, d.nWidthUV, d.nHeightUV, d.vectors_data.nPel, d.opt);


    if (d.vi->format.bitsPerSample == 8) {
        if (d.mode == Fetch)
            d.flow_function = flowFetch<uint8_t>;
        else if (d.mode == Shift)
            d.flow_function = flowShift<uint8_t>;

        d.memset_function = memset;
    } else {
        if (d.mode == Fetch)
            d.flow_function = flowFetch<uint16_t>;
        else if (d.mode == Shift)
            d.flow_function = flowShift<uint16_t>;

        d.memset_function = flowMemset<uint16_t>;
    }


    data = (MVFlowData *)malloc(sizeof(d));
    *data = d;

    VSFilterDependency deps[3] = { 
        {data->clip, rpStrictSpatial}, 
        // {data->super, rpStrictSpatial}, //MVFlow doesn't actually request any frames from the super.
        {data->finest, rpGeneral}, 
        {data->vectors, rpStrictSpatial}, 
    };

    vsapi->createVideoFilter(out, "Flow", data->vi, mvflowGetFrame, mvflowFree, fmParallel, deps, ARRAY_SIZE(deps), data, core);
}


extern "C" void mvflowRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Flow",
                 "clip:vnode;"
                 "super:vnode;"
                 "vectors:vnode;"
                 "time:float:opt;"
                 "mode:int:opt;"
                 "fields:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "opt:int:opt;"
                 "tff:int:opt;",
                 "clip:vnode;",
                 mvflowCreate, 0, plugin);
}
