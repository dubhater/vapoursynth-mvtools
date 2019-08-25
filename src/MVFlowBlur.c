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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "Bullshit.h"
#include "CommonFunctions.h"
#include "Fakery.h"
#include "MaskFun.h"
#include "MVAnalysisData.h"
#include "SimpleResize.h"


typedef struct MVFlowBlurData {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *finest;
    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    float blur;
    int prec;
    int64_t thscd1;
    int thscd2;
    int opt;

    MVAnalysisData mvbw_data;
    MVAnalysisData mvfw_data;

    int nSuperHPad;

    int nWidthUV;
    int nHeightUV;
    int nVPaddingUV;
    int nHPaddingUV;
    int VPitchY;
    int VPitchUV;

    int blur256;

    SimpleResize upsizer;
    SimpleResize upsizerUV;
} MVFlowBlurData;


static void VS_CC mvflowblurInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVFlowBlurData *d = (MVFlowBlurData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


#define RealFlowBlur(PixelType) \
static void RealFlowBlur_##PixelType(uint8_t *pdst8, int dst_pitch, const uint8_t *pref8, int ref_pitch, \
                         int16_t *VXFullB, int16_t *VXFullF, int16_t *VYFullB, int16_t *VYFullF, \
                         int VPitch, int width, int height, int blur256, int prec, int nPel) { \
    const PixelType *pref = (const PixelType *)pref8; \
    PixelType *pdst = (PixelType *)pdst8; \
 \
    ref_pitch /= sizeof(PixelType); \
    dst_pitch /= sizeof(PixelType); \
 \
    int nPelLog = ilog2(nPel); \
 \
    /* very slow, but precise motion blur */ \
    for (int h = 0; h < height; h++) { \
        for (int w = 0; w < width; w++) { \
            int bluredsum = pref[w << nPelLog]; \
            int vxF0 = VXFullF[w] * blur256; \
            int vyF0 = VYFullF[w] * blur256; \
            int mF = (VSMAX(abs(vxF0), abs(vyF0)) / prec) >> 8; \
            if (mF > 0) { \
                vxF0 /= mF; \
                vyF0 /= mF; \
                int vxF = vxF0; \
                int vyF = vyF0; \
                for (int i = 0; i < mF; i++) { \
                    int dstF = pref[(vyF >> 8) * ref_pitch + (vxF >> 8) + (w << nPelLog)]; \
                    bluredsum += dstF; \
                    vxF += vxF0; \
                    vyF += vyF0; \
                } \
            } \
            int vxB0 = VXFullB[w] * blur256; \
            int vyB0 = VYFullB[w] * blur256; \
            int mB = (VSMAX(abs(vxB0), abs(vyB0)) / prec) >> 8; \
            if (mB > 0) { \
                vxB0 /= mB; \
                vyB0 /= mB; \
                int vxB = vxB0; \
                int vyB = vyB0; \
                for (int i = 0; i < mB; i++) { \
                    int dstB = pref[(vyB >> 8) * ref_pitch + (vxB >> 8) + (w << nPelLog)]; \
                    bluredsum += dstB; \
                    vxB += vxB0; \
                    vyB += vyB0; \
                } \
            } \
            pdst[w] = bluredsum / (mF + mB + 1); \
        } \
        pdst += dst_pitch; \
        pref += (ref_pitch << nPelLog); \
        VXFullB += VPitch; \
        VYFullB += VPitch; \
        VXFullF += VPitch; \
        VYFullF += VPitch; \
    } \
}

RealFlowBlur(uint8_t)
RealFlowBlur(uint16_t)


static void FlowBlur(uint8_t *pdst, int dst_pitch, const uint8_t *pref, int ref_pitch,
                     int16_t *VXFullB, int16_t *VXFullF, int16_t *VYFullB, int16_t *VYFullF,
                     int VPitch, int width, int height, int blur256, int prec, int nPel, int bitsPerSample) {
    if (bitsPerSample == 8)
        RealFlowBlur_uint8_t(pdst, dst_pitch, pref, ref_pitch, VXFullB, VXFullF, VYFullB, VYFullF, VPitch, width, height, blur256, prec, nPel);
    else
        RealFlowBlur_uint16_t(pdst, dst_pitch, pref, ref_pitch, VXFullB, VXFullF, VYFullB, VYFullF, VPitch, width, height, blur256, prec, nPel);
}


static const VSFrameRef *VS_CC mvflowblurGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVFlowBlurData *d = (MVFlowBlurData *)*instanceData;

    if (activationReason == arInitial) {
        int off = d->mvbw_data.nDeltaFrame; // integer offset of reference frame

        if (n - off >= 0 && n + off < d->vi->numFrames) {
            vsapi->requestFrameFilter(n - off, d->mvbw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvfw, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->finest, frameCtx);
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        uint8_t *pDst[3];
        const uint8_t *pRef[3];
        int nDstPitches[3];
        int nRefPitches[3];

        FakeGroupOfPlanes fgopF, fgopB;

        fgopInit(&fgopF, &d->mvfw_data);
        fgopInit(&fgopB, &d->mvbw_data);

        int isUsableB = 0;
        int isUsableF = 0;

        int off = d->mvbw_data.nDeltaFrame; // integer offset of reference frame

        if (n - off >= 0 && n + off < d->vi->numFrames) {
            const VSFrameRef *mvF = vsapi->getFrameFilter(n + off, d->mvfw, frameCtx);
            const VSMap *mvprops = vsapi->getFramePropsRO(mvF);
            fgopUpdate(&fgopF, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);
            vsapi->freeFrame(mvF);

            const VSFrameRef *mvB = vsapi->getFrameFilter(n - off, d->mvbw, frameCtx);
            mvprops = vsapi->getFramePropsRO(mvB);
            fgopUpdate(&fgopB, (const uint8_t *)vsapi->propGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
            vsapi->freeFrame(mvB);
        }


        if (isUsableB && isUsableF) {
            const VSFrameRef *ref = vsapi->getFrameFilter(n, d->finest, frameCtx); //  ref for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, ref, core);

            for (int i = 0; i < d->vi->format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
            }

            const int nWidth = d->mvbw_data.nWidth;
            const int nHeight = d->mvbw_data.nHeight;
            const int nWidthUV = d->nWidthUV;
            const int nHeightUV = d->nHeightUV;
            const int xRatioUV = d->mvbw_data.xRatioUV;
            const int yRatioUV = d->mvbw_data.yRatioUV;
            const int nBlkX = d->mvbw_data.nBlkX;
            const int nBlkY = d->mvbw_data.nBlkY;
            const int nVPadding = d->mvbw_data.nVPadding;
            const int nHPadding = d->mvbw_data.nHPadding;
            const int nVPaddingUV = d->nVPaddingUV;
            const int nHPaddingUV = d->nHPaddingUV;
            const int nPel = d->mvbw_data.nPel;
            const int blur256 = d->blur256;
            const int prec = d->prec;
            const int VPitchY = d->VPitchY;
            const int VPitchUV = d->VPitchUV;

            int bitsPerSample = d->vi->format->bitsPerSample;
            int bytesPerSample = d->vi->format->bytesPerSample;

            int nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * bytesPerSample * nPel;
            int nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * bytesPerSample * nPel;


            size_t full_size = nHeight * VPitchY * sizeof(int16_t);
            size_t small_size = nBlkY * nBlkX * sizeof(int16_t);

            int16_t *VXFullYB = (int16_t *)malloc(full_size);
            int16_t *VYFullYB = (int16_t *)malloc(full_size);
            int16_t *VXFullYF = (int16_t *)malloc(full_size);
            int16_t *VYFullYF = (int16_t *)malloc(full_size);
            int16_t *VXSmallYB = (int16_t *)malloc(small_size);
            int16_t *VYSmallYB = (int16_t *)malloc(small_size);
            int16_t *VXSmallYF = (int16_t *)malloc(small_size);
            int16_t *VYSmallYF = (int16_t *)malloc(small_size);

            // make  vector vx and vy small masks
            MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYB, nBlkX, VYSmallYB, nBlkX);
            MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYF, nBlkX, VYSmallYF, nBlkX);

            // analyse vectors field to detect occlusion

            // upsize (bilinear interpolate) vector masks to fullframe size


            d->upsizer.simpleResize_int16_t(&d->upsizer, VXFullYB, VPitchY, VXSmallYB, nBlkX, 1);
            d->upsizer.simpleResize_int16_t(&d->upsizer, VYFullYB, VPitchY, VYSmallYB, nBlkX, 0);
            d->upsizer.simpleResize_int16_t(&d->upsizer, VXFullYF, VPitchY, VXSmallYF, nBlkX, 1);
            d->upsizer.simpleResize_int16_t(&d->upsizer, VYFullYF, VPitchY, VYSmallYF, nBlkX, 0);

            FlowBlur(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, nRefPitches[0],
                     VXFullYB, VXFullYF, VYFullYB, VYFullYF, VPitchY,
                     nWidth, nHeight, blur256, prec, nPel, bitsPerSample);

            if (d->vi->format->colorFamily != cmGray) {
                size_t full_size_uv = nHeightUV * VPitchUV * sizeof(int16_t);
                size_t small_size_uv = nBlkY * nBlkX * sizeof(int16_t);

                int16_t *VXFullUVB = (int16_t *)malloc(full_size_uv);
                int16_t *VYFullUVB = (int16_t *)malloc(full_size_uv);

                int16_t *VXFullUVF = (int16_t *)malloc(full_size_uv);
                int16_t *VYFullUVF = (int16_t *)malloc(full_size_uv);

                int16_t *VXSmallUVB = (int16_t *)malloc(small_size_uv);
                int16_t *VYSmallUVB = (int16_t *)malloc(small_size_uv);

                int16_t *VXSmallUVF = (int16_t *)malloc(small_size_uv);
                int16_t *VYSmallUVF = (int16_t *)malloc(small_size_uv);

                VectorSmallMaskYToHalfUV(VXSmallYB, nBlkX, nBlkY, VXSmallUVB, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYB, nBlkX, nBlkY, VYSmallUVB, yRatioUV);

                VectorSmallMaskYToHalfUV(VXSmallYF, nBlkX, nBlkY, VXSmallUVF, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYF, nBlkX, nBlkY, VYSmallUVF, yRatioUV);

                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VXFullUVB, VPitchUV, VXSmallUVB, nBlkX, 1);
                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VYFullUVB, VPitchUV, VYSmallUVB, nBlkX, 0);

                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VXFullUVF, VPitchUV, VXSmallUVF, nBlkX, 1);
                d->upsizerUV.simpleResize_int16_t(&d->upsizerUV, VYFullUVF, VPitchUV, VYSmallUVF, nBlkX, 0);


                FlowBlur(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, nRefPitches[1],
                         VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, VPitchUV,
                         nWidthUV, nHeightUV, blur256, prec, nPel, bitsPerSample);
                FlowBlur(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, nRefPitches[2],
                         VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, VPitchUV,
                         nWidthUV, nHeightUV, blur256, prec, nPel, bitsPerSample);

                free(VXFullUVB);
                free(VYFullUVB);
                free(VXSmallUVB);
                free(VYSmallUVB);
                free(VXFullUVF);
                free(VYFullUVF);
                free(VXSmallUVF);
                free(VYSmallUVF);
            }

            free(VXFullYB);
            free(VYFullYB);
            free(VXSmallYB);
            free(VYSmallYB);
            free(VXFullYF);
            free(VYFullYF);
            free(VXSmallYF);
            free(VYSmallYF);

            vsapi->freeFrame(ref);

            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            return dst;
        } else { // not usable
            fgopDeinit(&fgopF);
            fgopDeinit(&fgopB);

            return vsapi->getFrameFilter(n, d->node, frameCtx);
        }
    }

    return 0;
}


static void VS_CC mvflowblurFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVFlowBlurData *d = (MVFlowBlurData *)instanceData;

    simpleDeinit(&d->upsizer);
    if (d->vi->format->colorFamily != cmGray)
        simpleDeinit(&d->upsizerUV);

    vsapi->freeNode(d->finest);
    vsapi->freeNode(d->super);
    vsapi->freeNode(d->mvfw);
    vsapi->freeNode(d->mvbw);
    vsapi->freeNode(d->node);
    free(d);
}


static void VS_CC mvflowblurCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVFlowBlurData d;
    MVFlowBlurData *data;

    int err;

    d.blur = (float)vsapi->propGetFloat(in, "blur", 0, &err);
    if (err)
        d.blur = 50.0f;

    d.prec = int64ToIntS(vsapi->propGetInt(in, "prec", 0, &err));
    if (err)
        d.prec = 1;

    d.thscd1 = vsapi->propGetInt(in, "thscd1", 0, &err);
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.opt = !!vsapi->propGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = INT_MAX;


    if (d.blur < 0.0f || d.blur > 200.0f) {
        vsapi->setError(out, "FlowBlur: blur must be between 0 and 200 % (inclusive).");
        return;
    }

    if (d.prec < 1) {
        vsapi->setError(out, "FlowBlur: prec must be at least 1.");
        return;
    }

    d.blur256 = (int)(d.blur * 256.0f / 200.0f);


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "FlowBlur: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg + errorLen, ERROR_SIZE - errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->setError(out, errorMsg);
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[3];
    int nHeightS = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    int nSuperPel = int64ToIntS(vsapi->propGetInt(props, "Super_pel", 0, &evil_err[2]));
    vsapi->freeFrame(evil);

    for (int i = 0; i < 2; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "FlowBlur: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);

#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };
    const char *filter_name = "FlowBlur";

    adataFromVectorClip(&d.mvbw_data, d.mvbw, filter_name, "mvbw", vsapi, error, ERROR_SIZE);
    adataFromVectorClip(&d.mvfw_data, d.mvfw, filter_name, "mvfw", vsapi, error, ERROR_SIZE);

    scaleThSCD(&d.thscd1, &d.thscd2, &d.mvbw_data, filter_name, error, ERROR_SIZE);

    adataCheckSimilarity(&d.mvbw_data, &d.mvfw_data, filter_name, "mvbw", "mvfw", error, ERROR_SIZE);
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->setError(out, error);

        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }


    if (d.mvbw_data.nDeltaFrame <= 0 || d.mvfw_data.nDeltaFrame <= 0) {
        vsapi->setError(out, "FlowBlur: cannot use motion vectors with absolute frame references.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    // XXX Alternatively, use both clips' delta as offsets in GetFrame.
    if (d.mvfw_data.nDeltaFrame != d.mvbw_data.nDeltaFrame) {
        vsapi->setError(out, "FlowBlur: mvbw and mvfw must be generated with the same delta.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    // Make sure the motion vector clips are correct.
    if (!d.mvbw_data.isBackward || d.mvfw_data.isBackward) {
        if (!d.mvbw_data.isBackward)
            vsapi->setError(out, "FlowBlur: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "FlowBlur: mvfw must be generated with isb=False.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }


    if (d.mvbw_data.nPel == 1)
        d.finest = vsapi->cloneNodeRef(d.super); // v2.0.9.1
    else {
        VSPlugin *mvtoolsPlugin = vsapi->getPluginById("com.nodame.mvtools", core);
        VSPlugin *stdPlugin = vsapi->getPluginById("com.vapoursynth.std", core);

        VSMap *args = vsapi->createMap();
        vsapi->propSetNode(args, "super", d.super, paReplace);
        vsapi->propSetInt(args, "opt", d.opt, paReplace);
        VSMap *ret = vsapi->invoke(mvtoolsPlugin, "Finest", args);
        if (vsapi->getError(ret)) {
#define ERROR_SIZE 512
            char error_msg[ERROR_SIZE + 1] = { 0 };
            snprintf(error_msg, ERROR_SIZE, "FlowBlur: %s", vsapi->getError(ret));
#undef ERROR_SIZE
            vsapi->setError(out, error_msg);

            vsapi->freeNode(d.super);
            vsapi->freeNode(d.mvfw);
            vsapi->freeNode(d.mvbw);
            vsapi->freeMap(args);
            vsapi->freeMap(ret);
            return;
        }
        d.finest = vsapi->propGetNode(ret, "clip", 0, NULL);
        vsapi->freeMap(ret);

        vsapi->clearMap(args);
        vsapi->propSetNode(args, "clip", d.finest, paReplace);
        vsapi->freeNode(d.finest);
        ret = vsapi->invoke(stdPlugin, "Cache", args);
        vsapi->freeMap(args);
        if (vsapi->getError(ret)) {
#define ERROR_SIZE 512
            char error_msg[ERROR_SIZE + 1] = { 0 };
            snprintf(error_msg, ERROR_SIZE, "FlowBlur: %s", vsapi->getError(ret));
#undef ERROR_SIZE
            vsapi->setError(out, error_msg);

            vsapi->freeNode(d.super);
            vsapi->freeNode(d.mvfw);
            vsapi->freeNode(d.mvbw);
            vsapi->freeMap(ret);
            return;
        }
        d.finest = vsapi->propGetNode(ret, "clip", 0, NULL);
        vsapi->freeMap(ret);
    }

    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.mvbw_data.nHeight != nHeightS || d.mvbw_data.nWidth != nSuperWidth - d.nSuperHPad * 2 || d.mvbw_data.nPel != nSuperPel) {
        vsapi->setError(out, "FlowBlur: wrong source or super clip frame size.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, "FlowBlur: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        return;
    }


    d.nHeightUV = d.mvbw_data.nHeight / d.mvbw_data.yRatioUV;
    d.nWidthUV = d.mvbw_data.nWidth / d.mvbw_data.xRatioUV;
    d.nHPaddingUV = d.mvbw_data.nHPadding / d.mvbw_data.xRatioUV;
    //d.nVPaddingUV = d.mvbw_data.nHPadding / d.mvbw_data.yRatioUV; // original looks wrong
    d.nVPaddingUV = d.mvbw_data.nVPadding / d.mvbw_data.yRatioUV;

    d.VPitchY = d.mvbw_data.nWidth;
    d.VPitchUV = d.nWidthUV;

    simpleInit(&d.upsizer, d.mvbw_data.nWidth, d.mvbw_data.nHeight, d.mvbw_data.nBlkX, d.mvbw_data.nBlkY, d.mvbw_data.nWidth, d.mvbw_data.nHeight, d.mvbw_data.nPel, d.opt);
    if (d.vi->format->colorFamily != cmGray)
        simpleInit(&d.upsizerUV, d.nWidthUV, d.nHeightUV, d.mvbw_data.nBlkX, d.mvbw_data.nBlkY, d.nWidthUV, d.nHeightUV, d.mvbw_data.nPel, d.opt);


    data = (MVFlowBlurData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "FlowBlur", mvflowblurInit, mvflowblurGetFrame, mvflowblurFree, fmParallel, 0, data, core);
}


void mvflowblurRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("FlowBlur",
                 "clip:clip;"
                 "super:clip;"
                 "mvbw:clip;"
                 "mvfw:clip;"
                 "blur:float:opt;"
                 "prec:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "opt:int:opt;",
                 mvflowblurCreate, 0, plugin);
}
