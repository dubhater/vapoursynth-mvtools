#include <mutex>
#include <math.h>

#include <fftw3.h>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "Fakery.h"
#include "MVAnalysisData.h"


#define prop_DepanAnalyse_info "DepanAnalyse_info"
#define prop_DepanEstimate_info "DepanEstimate_info"
#define prop_DepanCompensate_info "DepanCompensate_info"
#define prop_DepanStabilise_info "DepanStabilise_info"

// stage 1 to stage 2
#define prop_DepanEstimateFFT "DepanEstimateFFT"
#define prop_DepanEstimateFFT2 "DepanEstimateFFT2"

// stage 2 to stage 3
#define prop_DepanEstimateX "DepanEstimateX"
#define prop_DepanEstimateY "DepanEstimateY"
#define prop_DepanEstimateZoom "DepanEstimateZoom"
#define prop_DepanEstimateTrust "DepanEstimateTrust"

#define prop_Depan_dx "Depan_dx"
#define prop_Depan_dy "Depan_dy"
#define prop_Depan_zoom "Depan_zoom"
#define prop_Depan_rot "Depan_rot"


#define MOTIONUNKNOWN 9999.0f
#define MOTIONBAD 0.0f


typedef struct DepanAnalyseData {
    VSNodeRef *clip;
    VSNodeRef *vectors;
    VSNodeRef *mask;
    int zoom;
    int rot;
    float pixaspect;
    float error;
    int info;
    float wrong;
    float zerow;
    int thscd1;
    int thscd2;
    int fields;
    int tff;
    int tff_exists;

    const VSVideoInfo *vi;
    MVAnalysisData vectors_data;
} DepanAnalyseData;


typedef struct transform {
// structure of global motion transform
//  which defines source (xsrc, ysrc)  for current destination (x,y)
//   xsrc = dxc + dxx*x + dxy*y
//   ysrc = dyc + dyx*x + dyy*y
// But really only 4  parameters (dxc, dxx, dxy, dyc) are independent in used model
    float dxc;
    float dxx;
    float dxy;
    float dyc;
    float dyx;
    float dyy;
} transform;


static void setNull(transform *tr) {
    tr->dxc = 0.0f;
    tr->dxx = 1.0f;
    tr->dxy = 0.0f;
    tr->dyc = 0.0f;
    tr->dyx = 0.0f;
    tr->dyy = 1.0f;
}


static void transform2motion(const transform *tr, int forward, float xcenter, float ycenter, float pixaspect, float *dx, float *dy, float *rot, float *zoom) {
    const float PI = 3.1415926535897932384626433832795f;
    float rotradian, sinus, cosinus;

    rotradian = -atanf(pixaspect * tr->dxy / tr->dxx);
    *rot = rotradian * 180 / PI;
    sinus = sinf(rotradian);
    cosinus = cosf(rotradian);
    *zoom = tr->dxx / cosinus;

    if (forward) { //  get motion for forward
        *dx = tr->dxc - xcenter - (-xcenter * cosinus + ycenter / pixaspect * sinus) * (*zoom);
        *dy = tr->dyc / pixaspect - ycenter / pixaspect - ((-ycenter) / pixaspect * cosinus + (-xcenter) * sinus) * (*zoom); // dyc

    } else { // coefficients for backward

        //        tr.dxc/(*zoom) = xcenter/(*zoom) + (-xcenter + dx)*cosinus - ((-ycenter)/pixaspect + dy)*sinus ;
        //        tr.dyc/(*zoom)/pixaspect = ycenter/(*zoom)/pixaspect +  ((-ycenter)/pixaspect +dy)*cosinus + (-xcenter + dx)*sinus ;
        // *cosinus:
        //        tr.dxc/(*zoom)*cosinus = xcenter/(*zoom)*cosinus + (-xcenter + dx)*cosinus*cosinus - ((-ycenter)/pixaspect + dy)*sinus*cosinus ;
        // *sinus:
        //        tr.dyc/(*zoom)/pixaspect*sinus = ycenter/(*zoom)/pixaspect*sinus +  ((-ycenter)/pixaspect +dy)*cosinus*sinus + (-xcenter + dx)*sinus*sinus ;
        // summa:
        //        tr.dxc/(*zoom)*cosinus + tr.dyc/(*zoom)/pixaspect*sinus = xcenter/(*zoom)*cosinus + (-xcenter + dx) + ycenter/(*zoom)/pixaspect*sinus   ;
        *dx = tr->dxc / (*zoom) * cosinus + tr->dyc / (*zoom) / pixaspect * sinus - xcenter / (*zoom) * cosinus + xcenter - ycenter / (*zoom) / pixaspect * sinus;

        // *sinus:
        //        tr.dxc/(*zoom)*sinus = xcenter/(*zoom)*sinus + (-xcenter + dx)*cosinus*sinus - ((-ycenter)/pixaspect + dy)*sinus*sinus ;
        // *cosinus:
        //        tr.dyc/(*zoom)/pixaspect*cosinus = ycenter/(*zoom)/pixaspect*cosinus +  ((-ycenter)/pixaspect +dy)*cosinus*cosinus + (-xcenter + dx)*sinus*cosinus ;
        // diff:
        //        tr.dxc/(*zoom)*sinus - tr.dyc/(*zoom)/pixaspect*cosinus = xcenter/(*zoom)*sinus - (-ycenter/pixaspect + dy) - ycenter/(*zoom)/pixaspect*cosinus   ;
        *dy = -tr->dxc / (*zoom) * sinus + tr->dyc / (*zoom) / pixaspect * cosinus + xcenter / (*zoom) * sinus - (-ycenter / pixaspect) - ycenter / (*zoom) / pixaspect * cosinus;
    }
}


//------------------------------------------------------------
//  get  coefficients for inverse coordinates transformation,
//  fransform_inv ( transform_A ) = null transform
static void inversetransform(const transform *ta, transform *tinv) {
    float pixaspect;

    if (ta->dxy != 0.0f)
        pixaspect = sqrtf(-ta->dyx / ta->dxy);
    else
        pixaspect = 1.0f;

    tinv->dxx = ta->dxx / ((ta->dxx) * ta->dxx + ta->dxy * ta->dxy * pixaspect * pixaspect);
    tinv->dyy = tinv->dxx;
    tinv->dxy = -tinv->dxx * ta->dxy / ta->dxx;
    tinv->dyx = -tinv->dxy * pixaspect * pixaspect;
    tinv->dxc = -tinv->dxx * ta->dxc - tinv->dxy * ta->dyc;
    tinv->dyc = -tinv->dyx * ta->dxc - tinv->dyy * ta->dyc;
}


static void TrasformUpdate(transform *tr, const float *blockDx, const float *blockDy, const int *blockX, const int *blockY, const float *blockWeight, int nBlkX, int nBlkY, float safety, int ifZoom1, int ifRot1, float *error1, float pixaspect) {
    transform trderiv;
    int n = nBlkX * nBlkY;
    trderiv.dxc = 0;
    trderiv.dxx = 0;
    trderiv.dxy = 0;
    trderiv.dyc = 0;
    trderiv.dyx = 0;
    trderiv.dyy = 0;
    float norm = 0.1f;
    float x2 = 0.1f;
    float y2 = 0.1f;
    float error2 = 0.1f;
    for (int i = 0; i < n; i++) {
        float bw = blockWeight[i];
        float xdif = (tr->dxc + tr->dxx * blockX[i] + tr->dxy * blockY[i] - blockX[i] - blockDx[i]);
        trderiv.dxc += 2 * xdif * bw;
        if (ifZoom1)
            trderiv.dxx += 2 * blockX[i] * xdif * bw;
        if (ifRot1)
            trderiv.dxy += 2 * blockY[i] * xdif * bw;
        float ydif = (tr->dyc + tr->dyx * blockX[i] + tr->dyy * blockY[i] - blockY[i] - blockDy[i]);
        trderiv.dyc += 2 * ydif * bw;
        if (ifRot1)
            trderiv.dyx += 2 * blockX[i] * ydif * bw;
        if (ifZoom1)
            trderiv.dyy += 2 * blockY[i] * ydif * bw;
        norm += bw;
        x2 += blockX[i] * blockX[i] * bw;
        y2 += blockY[i] * blockY[i] * bw;
        error2 += (xdif * xdif + ydif * ydif) * bw;
    }
    trderiv.dxc /= norm * 2;
    trderiv.dxx /= x2 * 2 * 1.5f; // with additional safety factors
    trderiv.dxy /= y2 * 2 * 3;
    trderiv.dyc /= norm * 2;
    trderiv.dyx /= x2 * 2 * 3;
    trderiv.dyy /= y2 * 2 * 1.5f;

    error2 /= norm;
    *error1 = sqrtf(error2);

    tr->dxc -= safety * trderiv.dxc;
    if (ifZoom1)
        tr->dxx -= safety * 0.5f * (trderiv.dxx + trderiv.dyy);

    tr->dxy -= safety * 0.5f * (trderiv.dxy - trderiv.dyx / (pixaspect * pixaspect));
    tr->dyc -= safety * trderiv.dyc;
    //    tr->dyx -= safety*trderiv.dyx;
    //    tr->dyy -= safety*trderiv.dyy;
    if (ifZoom1)
        tr->dyy = tr->dxx;
    //    float pixaspect=1; // was for test and forgot remove?! disabled in v1.2.5
    tr->dyx = -pixaspect * pixaspect * tr->dxy;
}
//----------------------------------------------------------------------------


static void RejectBadBlocks(const transform *tr, const float *blockDx, const float *blockDy, const int *blockSAD, const int *blockX, const int *blockY, float *blockWeight, int nBlkX, int nBlkY, float wrongDif, float globalDif, int thSCD1, float zeroWeight, const float *blockWeightMask, int ignoredBorder) {
    for (int j = 0; j < nBlkY; j++) {
        for (int i = 0; i < nBlkX; i++) {
            int n = j * nBlkX + i;
            if (i < ignoredBorder || i >= nBlkX - ignoredBorder || j < ignoredBorder || j >= nBlkY - ignoredBorder) {
                blockWeight[n] = 0; // disable  blocks near frame borders
            } else if (blockSAD[n] > thSCD1) {
                blockWeight[n] = 0; // disable bad block with big SAD
            } else if (i > 0 && i < (nBlkX - 1) && (fabsf((blockDx[n - 1 - nBlkX] + blockDx[n - nBlkX] + blockDx[n + 1 - nBlkX] +
                                                          blockDx[n - 1] + blockDx[n + 1] +
                                                          blockDx[n - 1 + nBlkX] + blockDx[n + nBlkX] + blockDx[n + 1 + nBlkX]) /
                                                             8 -
                                                         blockDx[n]) > wrongDif)) {
                blockWeight[n] = 0; // disable blocks very different from neighbours
            } else if (j > 0 && j < (nBlkY - 1) && (fabsf((blockDy[n - 1 - nBlkX] + blockDy[n - nBlkX] + blockDy[n + 1 - nBlkX] +
                                                          blockDy[n - 1] + blockDy[n + 1] +
                                                          blockDy[n - 1 + nBlkX] + blockDy[n + nBlkX] + blockDy[n + 1 + nBlkX]) /
                                                             8 -
                                                         blockDy[n]) > wrongDif)) {
                blockWeight[n] = 0; // disable blocks very different from neighbours
            } else if (fabsf(tr->dxc + tr->dxx * blockX[n] + tr->dxy * blockY[n] - blockX[n] - blockDx[n]) > globalDif) {
                blockWeight[n] = 0; // disable blocks very different from global
            } else if (fabsf(tr->dyc + tr->dyx * blockX[n] + tr->dyy * blockY[n] - blockY[n] - blockDy[n]) > globalDif) {
                blockWeight[n] = 0; // disable blocks very different from global
            } else if (blockDx[n] == 0.0f && blockDy[n] == 0.0f) {
                blockWeight[n] = zeroWeight * blockWeightMask[n];
            } else {
                blockWeight[n] = blockWeightMask[n]; // good block
            }
        }
    }
}


static void VS_CC depanAnalyseInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    DepanAnalyseData *d = (DepanAnalyseData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC depanAnalyseGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    DepanAnalyseData *d = (DepanAnalyseData *)*instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(d->vectors_data.isBackward ? VSMAX(0, n - 1) : n, d->vectors, frameCtx);
        vsapi->requestFrameFilter(n, d->clip, frameCtx);

        if (d->mask)
            vsapi->requestFrameFilter(n, d->mask, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->clip, frameCtx);
        const VSFrameRef *mask = NULL;
        if (d->mask)
            mask = vsapi->getFrameFilter(n, d->mask, frameCtx);

        VSFrameRef *dst = vsapi->copyFrame(src, core);
        vsapi->freeFrame(src);
        VSMap *dst_props = vsapi->getFramePropsRW(dst);

        const int nFields = d->fields ? 2 : 1;

        const uint8_t *maskp = NULL;
        int mask_pitch = 0;
        if (d->mask) {
            maskp = vsapi->getReadPtr(mask, 0);
            mask_pitch = vsapi->getStride(mask, 0);
        }

        const int backward = d->vectors_data.isBackward;

        const size_t num_blocks = (size_t)d->vectors_data.nBlkX * (size_t)d->vectors_data.nBlkY;

        float *blockDx =         (float *)malloc(num_blocks * sizeof(float)); // dx vector
        float *blockDy =         (float *)malloc(num_blocks * sizeof(float)); // dy
        int *blockSAD =            (int *)malloc(num_blocks * sizeof(int));
        int *blockX =              (int *)malloc(num_blocks * sizeof(int)); // block x position
        int *blockY =              (int *)malloc(num_blocks * sizeof(int));
        float *blockWeight =     (float *)malloc(num_blocks * sizeof(float));
        float *blockWeightMask = (float *)malloc(num_blocks * sizeof(float));

        transform tr;

        // init motion transform as null
        setNull(&tr);

        float errorcur = d->error * 2;
        int iter = 0; // start iteration

        int nframemv = backward ? VSMAX(0, n - 1) : n; // set prev frame number as data frame if backward
        FakeGroupOfPlanes fgop;
        fgopInit(&fgop, &d->vectors_data);

        const VSFrameRef *mvn = vsapi->getFrameFilter(nframemv, d->vectors, frameCtx);
        const VSMap *mvn_props = vsapi->getFramePropsRO(mvn);

        fgopUpdate(&fgop, (const int *)vsapi->propGetData(mvn_props, prop_MVTools_vectors, 0, NULL));
        vsapi->freeFrame(mvn);

        if (fgopIsUsable(&fgop, d->thscd1, d->thscd2)) {
            const float dPel = 1.0f / d->vectors_data.nPel; // subpixel precision value

            for (int j = 0; j < d->vectors_data.nBlkY; j++) {
                for (int i = 0; i < d->vectors_data.nBlkX; i++) {
                    int nb = j * d->vectors_data.nBlkX + i;
                    const FakeBlockData *fbd = fgopGetBlock(&fgop, 0, nb);
                    blockDx[nb] = fbd->vector.x * dPel;
                    blockDy[nb] = fbd->vector.y * dPel;
                    blockSAD[nb] = fbd->vector.sad;
                    blockX[nb] = fbd->x + d->vectors_data.nBlkSizeX / 2;
                    blockY[nb] = fbd->y + d->vectors_data.nBlkSizeY / 2;
                    if (d->mask && blockX[nb] < d->vi->width && blockY[nb] < d->vi->height)
                        blockWeightMask[nb] = maskp[blockX[nb] + blockY[nb] * mask_pitch];
                    else
                        blockWeightMask[nb] = 1.0f;
                    blockWeight[nb] = blockWeightMask[nb];
                }
            }

            // begin with translation only
            float safety = 0.3f; // begin with small safety factor
            int ifRot0 = 0;
            int ifZoom0 = 0;
            float globalDif0 = 1000.0f;
            int ignoredBorder = mask ? 0 : 4;


            for (; iter < 5; iter++) {
                TrasformUpdate(&tr, blockDx, blockDy, blockX, blockY, blockWeight, d->vectors_data.nBlkX, d->vectors_data.nBlkY, safety, ifZoom0, ifRot0, &errorcur, d->pixaspect / nFields);
                RejectBadBlocks(&tr, blockDx, blockDy, blockSAD, blockX, blockY, blockWeight, d->vectors_data.nBlkX, d->vectors_data.nBlkY, d->wrong, globalDif0, d->thscd1, d->zerow, blockWeightMask, ignoredBorder);
            }


            const float errordif = 0.01f;   // error difference to terminate iterations

            for (; iter < 100; iter++) {
                if (iter < 8)
                    safety = 0.3f; // use for safety
                else if (iter < 10)
                    safety = 0.6f;
                else
                    safety = 1.0f;
                float errorprev = errorcur;
                TrasformUpdate(&tr, blockDx, blockDy, blockX, blockY, blockWeight, d->vectors_data.nBlkX, d->vectors_data.nBlkY, safety, d->zoom, d->rot, &errorcur, d->pixaspect / nFields);
                if (((errorprev - errorcur) < errordif * 0.5f && iter > 9) || errorcur < errordif)
                    break; // check convergence, accuracy increased in v1.2.5
                float globalDif = errorcur * 2;
                RejectBadBlocks(&tr, blockDx, blockDy, blockSAD, blockX, blockY, blockWeight, d->vectors_data.nBlkX, d->vectors_data.nBlkY, d->wrong, globalDif, d->thscd1, d->zerow, blockWeightMask, ignoredBorder);
            }
        }

        fgopDeinit(&fgop);

        // we get transform (null if scenechange)

        float xcenter = (float)d->vi->width / 2;
        float ycenter = (float)d->vi->height / 2;

        float motionx = 0.0f;
        float motiony = 0.0f;
        float motionrot = 0.0f;
        float motionzoom = 1.0f;

        if (errorcur < d->error) { // if not bad result
            // convert transform data to ordinary motion format
            if (d->vectors_data.isBackward) {
                transform trinv;
                inversetransform(&tr, &trinv);
                transform2motion(&trinv, 0, xcenter, ycenter, d->pixaspect / nFields, &motionx, &motiony, &motionrot, &motionzoom);
            } else
                transform2motion(&tr, 1, xcenter, ycenter, d->pixaspect / nFields, &motionx, &motiony, &motionrot, &motionzoom);

            // fieldbased correction
            if (d->fields) {
                const VSFrameRef *temp = vsapi->getFrameFilter(n, d->clip, frameCtx);
                const VSMap *temp_props = vsapi->getFramePropsRO(temp);
                int err;
                int top_field = !!vsapi->propGetInt(temp_props, "_Field", 0, &err);
                vsapi->freeFrame(temp);

                if (err && !d->tff_exists) {
                    vsapi->setFilterError("DepanAnalyse: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    vsapi->freeFrame(mask);
                    vsapi->freeFrame(dst);
                    return NULL;
                }

                if (d->tff_exists)
                    top_field = d->tff ^ (n % 2);

                float yadd = top_field ? 0.5f : -0.5f;

                // scale dy for fieldbased frame by factor 2 (for compatibility)
                yadd = yadd * 2;
                motiony += yadd;
            }

            // if it is accidentally very small, reset it to small, but non-zero value,
            // to differ from pure 0, which be interpreted as bad value mark (scene change)
            if (fabsf(motionx) < 0.01f)
                motionx = (rand() > RAND_MAX / 2) ? 0.011f : -0.011f;
        }

        if (d->info) {
#define INFO_SIZE 128
            char info[INFO_SIZE + 1] = { 0 };

            snprintf(info, INFO_SIZE, "fn=%d iter=%d error=%.3f dx=%.2f dy=%.2f rot=%.3f zoom=%.5f", n, iter, errorcur, motionx, motiony, motionrot, motionzoom);
#undef INFO_SIZE

            vsapi->propSetData(dst_props, prop_DepanAnalyse_info, info, -1, paReplace);
        }

        free(blockDx);
        free(blockDy);
        free(blockSAD);
        free(blockX);
        free(blockY);
        free(blockWeight);
        free(blockWeightMask);

        vsapi->propSetFloat(dst_props, prop_Depan_dx, motionx, paReplace);
        vsapi->propSetFloat(dst_props, prop_Depan_dy, motiony, paReplace);
        vsapi->propSetFloat(dst_props, prop_Depan_zoom, motionzoom, paReplace);
        vsapi->propSetFloat(dst_props, prop_Depan_rot, motionrot, paReplace);

        vsapi->freeFrame(mask);

        return dst;
    }

    return NULL;
}


static void VS_CC depanAnalyseFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    DepanAnalyseData *d = (DepanAnalyseData *)instanceData;

    vsapi->freeNode(d->clip);
    vsapi->freeNode(d->vectors);
    vsapi->freeNode(d->mask);

    free(d);
}


static int invokeFrameProps(const char *prop, VSMap *out, VSCore *core, const VSAPI *vsapi) {
    VSPlugin *text_plugin = vsapi->getPluginById("com.vapoursynth.text", core);

    VSNodeRef *node = vsapi->propGetNode(out, "clip", 0, NULL);
    VSMap *args = vsapi->createMap();
    vsapi->propSetNode(args, "clip", node, paReplace);
    vsapi->freeNode(node);
    vsapi->propSetData(args, "props", prop, -1, paReplace);

    VSMap *ret = vsapi->invoke(text_plugin, "FrameProps", args);
    vsapi->freeMap(args);

    if (vsapi->getError(ret)) {
        vsapi->setError(out, vsapi->getError(ret));
        vsapi->freeMap(ret);
        return 0;
    }

    node = vsapi->propGetNode(ret, "clip", 0, NULL);
    vsapi->freeMap(ret);
    vsapi->propSetNode(out, "clip", node, paReplace);
    vsapi->freeNode(node);

    return 1;
}


static void VS_CC depanAnalyseCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    DepanAnalyseData d;
    memset(&d, 0, sizeof(d));

    int err;

    d.zoom = !!vsapi->propGetInt(in, "zoom", 0, &err);
    if (err)
        d.zoom = 1;

    d.rot = !!vsapi->propGetInt(in, "rot", 0, &err);
    if (err)
        d.rot = 1;

    d.pixaspect = (float)vsapi->propGetFloat(in, "pixaspect", 0, &err);
    if (err)
        d.pixaspect = 1.0f;

    d.error = (float)vsapi->propGetFloat(in, "error", 0, &err);
    if (err)
        d.error = 15.0f;

    d.info = !!vsapi->propGetInt(in, "info", 0, &err);

    d.wrong = (float)vsapi->propGetFloat(in, "wrong", 0, &err);
    if (err)
        d.wrong = 10.0f;

    d.zerow = (float)vsapi->propGetFloat(in, "zerow", 0, &err);
    if (err)
        d.zerow = 0.05f;

    d.thscd1 = int64ToIntS(vsapi->propGetInt(in, "thscd1", 0, &err));
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.fields = !!vsapi->propGetInt(in, "fields", 0, &err);

    d.tff = !!vsapi->propGetInt(in, "tff", 0, &d.tff_exists);
    d.tff_exists = !d.tff_exists;


    if (d.pixaspect <= 0.0f) {
        vsapi->setError(out, "DepanAnalyse: pixaspect must be positive.");
        return;
    }


    d.clip = vsapi->propGetNode(in, "clip", 0, NULL);
    d.vi = vsapi->getVideoInfo(d.clip);

    if (!d.vi->format) { // XXX etc
        vsapi->setError(out, "DepanAnalyse: clip must have constant format.");
        vsapi->freeNode(d.clip);
        return;
    }

    d.vectors = vsapi->propGetNode(in, "vectors", 0, NULL);

    if (d.vi->numFrames > vsapi->getVideoInfo(d.vectors)->numFrames) {
        vsapi->setError(out, "DepanAnalyse: vectors must have at least as many frames as clip.");
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.clip);
        return;
    }

    d.mask = vsapi->propGetNode(in, "mask", 0, &err);

    if (d.mask && d.vi->numFrames > vsapi->getVideoInfo(d.mask)->numFrames) {
        vsapi->setError(out, "DepanStabilise: mask must have at least as many frames as clip.");
        vsapi->freeNode(d.mask);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.clip);
        return;
    }


#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };
    const char *filter_name = "DepanAnalyse";

    adataFromVectorClip(&d.vectors_data, d.vectors, filter_name, "vectors", vsapi, error, ERROR_SIZE);

    scaleThSCD(&d.thscd1, &d.thscd2, &d.vectors_data, filter_name, error, ERROR_SIZE);

    if (d.vectors_data.nDeltaFrame != 1)
        snprintf(error, ERROR_SIZE, "DepanAnalyse: vectors clip must be created with delta=1.");
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->setError(out, error);

        vsapi->freeNode(d.clip);
        vsapi->freeNode(d.vectors);
        vsapi->freeNode(d.mask);
        return;
    }


    DepanAnalyseData *data = (DepanAnalyseData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "DepanAnalyse", depanAnalyseInit, depanAnalyseGetFrame, depanAnalyseFree, fmParallel, 0, data, core);

    if (vsapi->getError(out)) {
        depanAnalyseFree(data, core, vsapi);
        return;
    }

    if (d.info) {
        if (!invokeFrameProps(prop_DepanAnalyse_info, out, core, vsapi)) {
            vsapi->setError(out, std::string("DepanAnalyse: failed to invoke text.FrameProps: ").append(vsapi->getError(out)).c_str());
            depanAnalyseFree(data, core, vsapi);
            return;
        }
    }
}


typedef struct DepanEstimateData {
    VSNodeRef *clip;
    float trust_limit;
    int winx;
    int winy;
    int wleft;
    int wtop;
    int dxmax;
    int dymax;
    float zoommax;
    float stab;
    float pixaspect;
    int info;
    int show;
    int fields;
    int tff;
    int tff_exists;

    const VSVideoInfo *vi;

    int stage;

    int pixel_max;

    size_t fftsize;

    fftwf_complex *unused_array;

    fftwf_plan plan, planinv;
} DepanEstimateData;


static void VS_CC depanEstimateInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    DepanEstimateData *d = (DepanEstimateData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


// put source data to real array for FFT
static void frame_data2d(const uint8_t *srcp, int pitch, float *realdata, int winx, int winy, int winleft, int h0, int bytes_per_sample) {
    int i, j;
    int winxpadded = (winx / 2 + 1) * 2;

    srcp += pitch * h0 + winleft * bytes_per_sample; // offset of window data
    for (j = 0; j < winy; j++) {
        if (bytes_per_sample == 8) {
            for (i = 0; i < winx; i += 2) {
                realdata[i] = srcp[i];         // real part
                realdata[i + 1] = srcp[i + 1]; // real part
            }
        } else if (bytes_per_sample == 32) {
            for (i = 0; i < winx; i += 2) {
                const float *srcpf = (const float *)srcp;
                realdata[i] = srcpf[i];         // real part
                realdata[i + 1] = srcpf[i + 1]; // real part
            }
        } else {
            for (i = 0; i < winx; i += 2) {
                const uint16_t *srcp16 = (const uint16_t *)srcp;
                realdata[i] = srcp16[i];         // real part
                realdata[i + 1] = srcp16[i + 1]; // real part
            }
        }
        srcp += pitch;
        realdata += winxpadded;
    }
}


static void mult_conj_data2d(const fftwf_complex *fftnext, const fftwf_complex *fftsrc, fftwf_complex *mult, int winx, int winy) {
    // multiply complex conj. *next to src
    // (hermit)
    int nx = winx / 2 + 1; //padded, odd

    int total = winy * nx;                                                                            // even
    for (int k = 0; k < total; k += 2) { //paired for speed
        // real part
        mult[k][0] = fftnext[k][0] * fftsrc[k][0] + fftnext[k][1] * fftsrc[k][1];
        // imaginary part
        mult[k][1] = fftnext[k][0] * fftsrc[k][1] - fftnext[k][1] * fftsrc[k][0];
        // real part
        mult[k + 1][0] = fftnext[k + 1][0] * fftsrc[k + 1][0] + fftnext[k + 1][1] * fftsrc[k + 1][1];
        // imaginary part
        mult[k + 1][1] = fftnext[k + 1][0] * fftsrc[k + 1][1] - fftnext[k + 1][1] * fftsrc[k + 1][0];
    }
}


static void get_motion_vector(const float *correl, int winx, int winy, float trust_limit, int dxmax, int dymax, float stab, int fieldbased, int top_field, float pixaspect, float *fdx, float *fdy, float *trust) {
    float correlmax, cur, correlmean;
    float f1, f2;
    float xadd = 0.0f;
    float yadd = 0.0f;
    int i, j;
    int dx, dy;
    int imax = 0, jmax = 0;
    int imaxm1, imaxp1, jmaxm1, jmaxp1;
    int count;

    int winxpadded = (winx / 2 + 1) * 2;
    const float *correlp;


    // find global max on real part of correlation surface
    // new version: search only at 4 corners with ranges dxmax, dymax
    correlmax = correl[0];
    correlmean = 0.0f;
    count = 0;
    correlp = correl;
    for (j = 0; j <= dymax; j++) { // top
        for (i = 0; i <= dxmax; i++) { //left
            cur = correlp[i]; // real part
            correlmean += cur;
            count += 1;
            if (correlmax < cur) {
                correlmax = cur;
                imax = i;
                jmax = j;
            }
        }
        for (i = winx - dxmax; i < winx; i++) { //right
            cur = correlp[i]; // real part
            correlmean += cur;
            count += 1;
            if (correlmax < cur) {
                correlmax = cur;
                imax = i;
                jmax = j;
            }
        }
        correlp += winxpadded;
    }
    correlp = correl + (winy - dymax) * winxpadded;
    for (j = winy - dymax; j < winy; j++) { // bottom
        for (i = 0; i <= dxmax; i++) { //left
            cur = correlp[i]; // real part
            correlmean += cur;
            count += 1;
            if (correlmax < cur) {
                correlmax = cur;
                imax = i;
                jmax = j;
            }
        }
        for (i = winx - dxmax; i < winx; i++) { //right
            cur = correlp[i]; // real part
            correlmean += cur;
            count += 1;
            if (correlmax < cur) {
                correlmax = cur;
                imax = i;
                jmax = j;
            }
        }
        correlp += winxpadded;
    }

    correlmean = correlmean / count; // mean value

    correlmax = correlmax / (winx * winy);   // normalize value
    correlmean = correlmean / (winx * winy); // normalize value

    *trust = (correlmax - correlmean) * 100.0f / (correlmax + 0.1f); // +0.1 for safe divide


    if (imax * 2 < winx) {
        dx = imax;
    } else { // get correct shift values on periodic surface (adjusted borders)
        dx = (imax - winx);
    }

    if (jmax * 2 < winy) {
        dy = jmax;
    } else { // get correct shift values on periodic surface (adjusted borders)
        dy = (jmax - winy);
    }

    // some trust decreasing for large shifts

    *trust *= (dxmax + 1) / (dxmax + 1 + stab * abs(dx)) * (dymax + 1) / (dymax + 1 + stab * abs(dy));

    // reject if relative diffference correlmax from correlmean is small
    // probably due to scene change
    if (*trust < trust_limit) {
        dx = 0; // set value to pure 0, what will be interpreted as bad mark (scene change)
        dy = 0;
        xadd = 0.0f;
        yadd = 0.0f;
        *fdx = 0.0f;
        *fdy = 0.0f;

    } else {
        // normal, no scene change
        // get more precise float dx, dy by interpolation
        // get i, j, of left and right of max
        if (imax + 1 < winx)
            imaxp1 = imax + 1; // plus 1
        else
            imaxp1 = imax + 1 - winx; // over period

        if (imax - 1 >= 0)
            imaxm1 = imax - 1; // minus 1
        else
            imaxm1 = imax - 1 + winx;

        if (jmax + 1 < winy)
            jmaxp1 = jmax + 1;
        else
            jmaxp1 = jmax + 1 - winy;

        if (jmax - 1 >= 0)
            jmaxm1 = jmax - 1;
        else
            jmaxm1 = jmax - 1 + winy;

        // first and second differential
        f1 = (correl[jmax * winxpadded + imaxp1] - correl[jmax * winxpadded + imaxm1]) / 2.0f;
        f2 = correl[jmax * winxpadded + imaxp1] + correl[jmax * winxpadded + imaxm1] - correl[jmax * winxpadded + imax] * 2.0f;

        if (f2 == 0.0f)
            xadd = 0.0f;
        else {
            xadd = -f1 / f2;
            if (xadd > 1.0f)
                xadd = 1.0f;
            else if (xadd < -1.0f)
                xadd = -1.0f;
        }

        if (fabsf(dx + xadd) > dxmax)
            xadd = 0.0f;

        f1 = (correl[jmaxp1 * winxpadded + imax] - correl[jmaxm1 * winxpadded + imax]) / 2.0f;
        f2 = correl[jmaxp1 * winxpadded + imax] + (correl[jmaxm1 * winxpadded + imax]) - correl[jmax * winxpadded + imax] * 2.0f;

        if (f2 == 0.0f)
            yadd = 0.0f;
        else {
            yadd = -f1 / f2;
            if (yadd > 1.0f)
                yadd = 1.0f; // limit addition for stability
            else if (yadd < -1.0f)
                yadd = -1.0f;
        }

        if (fabsf(dy + yadd) > dymax)
            yadd = 0.0f;

        if (fieldbased) { // correct line shift for fields
            if (top_field)
                yadd += 0.5f;
            else
                yadd += -0.5f;
            // scale dy for fieldbased frame by factor 2
            yadd = yadd * 2.0f;
            dy = dy * 2;
        }


        *fdx = (float)dx + xadd;
        *fdy = (float)dy + yadd;

        *fdy = (*fdy) / pixaspect;

        // if it is accidentally very small, reset it to small, but non-zero value,
        // to differ from pure 0, which be interpreted as bad value mark (scene change)
        if (fabsf(*fdx) < 0.01f)
            *fdx = (rand() > RAND_MAX / 2) ? 0.011f : -0.011f;

        // if (fabs(*fdy) < 0.01f) *fdy = 0.011f; // disabled in 0.9.1 (only dx used)
    }
}


// get forward fft of src frame plane
static void get_plane_fft(const uint8_t *srcp, int src_pitch, fftwf_complex *fftsrc, int winx, int winy, int winleft, int wintop, fftwf_plan plan, int bytes_per_sample) {
    // prepare 2d data for fft
    frame_data2d(srcp, src_pitch, (float *)fftsrc, winx, winy, winleft, wintop, bytes_per_sample);
    // make forward fft of data
    fftwf_execute_dft_r2c(plan, (float *)fftsrc, fftsrc);
}


static void showcorrelation(const float *correl, int winx, int winy, uint8_t *dstp, int dst_pitch, int winleft, int wintop, int pixel_max) {
    float correlmax, correlmin, cur;
    int i, j;
    int winxpadded = (winx / 2 + 1) * 2;

    const float *correlp;

    // find max and min
    correlmax = correl[0];
    correlmin = correl[0];
    correlp = correl;
    for (j = 0; j < winy; j++) {
        for (i = 0; i < winx; i++) {
            cur = correlp[i];
            if (correlmax < cur) {
                correlmax = cur;
            }
            if (correlmin > cur) {
                correlmin = cur;
            }
        }
        correlp += winxpadded;
    }

    // normalize
    float norm = (float)pixel_max / (correlmax - correlmin);

    dstp += wintop * dst_pitch; // go to first line of window

    int bytes_per_sample;
    if (pixel_max == 255)
        bytes_per_sample = 1;
    else if (pixel_max == 1)
        bytes_per_sample = 4;
    else
        bytes_per_sample = 2;

    dstp += winleft * bytes_per_sample;
    correlp = correl;
    for (j = 0; j < winy; j++) {
        if (pixel_max == 255) {
            for (i = 0; i < winx; i++)
                dstp[i] = (int)((correlp[i] - correlmin) * norm); // real part
        } else if (pixel_max == 1) {
            for (i = 0; i < winx; i++) {
                float *dstpf = (float *)dstp;
                dstpf[i] = (correlp[i] - correlmin) * norm;
            }
        } else {
            for (i = 0; i < winx; i++) {
                uint16_t *dstp16 = (uint16_t *)dstp;
                dstp16[i] = (int)((correlp[i] - correlmin) * norm);
            }
        }

        dstp += dst_pitch;
        correlp += winxpadded;
    }
}


static const VSFrameRef *VS_CC depanEstimateStage1GetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    DepanEstimateData *d = (DepanEstimateData *)*instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->clip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->clip, frameCtx);
        VSFrameRef *dst = vsapi->copyFrame(src, core);
        vsapi->freeFrame(src);

        const uint8_t *dstp = vsapi->getReadPtr(dst, 0);
        int stride = vsapi->getStride(dst, 0);

        fftwf_complex *fft = (fftwf_complex *)fftwf_malloc(d->fftsize);

        int winleft = d->wleft; // left of fft window

        get_plane_fft(dstp, stride, fft, d->winx, d->winy, winleft, d->wtop, d->plan, d->vi->format->bytesPerSample);

        VSMap *dst_props = vsapi->getFramePropsRW(dst);

        vsapi->propSetData(dst_props, prop_DepanEstimateFFT, (const char *)fft, d->fftsize, paReplace);
        fftwf_free(fft);

        if (d->zoommax != 1.0f) {
            fftwf_complex *fft2 = (fftwf_complex *)fftwf_malloc(d->fftsize);

            int winleft2 = d->wleft + d->vi->width / 2; // left edge of right (2)fft window

            get_plane_fft(dstp, stride, fft2, d->winx, d->winy, winleft2, d->wtop, d->plan, d->vi->format->bytesPerSample);

            vsapi->propSetData(dst_props, prop_DepanEstimateFFT2, (const char *)fft2, d->fftsize, paReplace);
            fftwf_free(fft2);
        }

        return dst;
    }

    return NULL;
}


static const VSFrameRef *VS_CC depanEstimateStage2GetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    DepanEstimateData *d = (DepanEstimateData *)*instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(VSMAX(0, n - 1), d->clip, frameCtx);
        vsapi->requestFrameFilter(n, d->clip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *prev = vsapi->getFrameFilter(VSMAX(0, n - 1), d->clip, frameCtx);
        const VSFrameRef *cur = vsapi->getFrameFilter(n, d->clip, frameCtx);

        const VSMap *prev_props = vsapi->getFramePropsRO(prev);
        const VSMap *cur_props = vsapi->getFramePropsRO(cur);
        int err;

        int top_field = 0;
        if (d->fields) {
            top_field = !!vsapi->propGetInt(cur_props, "_Field", 0, &err);

            if (err && !d->tff_exists) {
                vsapi->setFilterError("DepanEstimate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                vsapi->freeFrame(prev);
                vsapi->freeFrame(cur);
                return NULL;
            }

            if (d->tff_exists)
                top_field = d->tff ^ (n % 2);
        }

        if (d->fftsize != (size_t)vsapi->propGetDataSize(prev_props, prop_DepanEstimateFFT, 0, &err) ||
            d->fftsize != (size_t)vsapi->propGetDataSize(cur_props, prop_DepanEstimateFFT, 0, &err)) {
            vsapi->setFilterError("DepanEstimate: temporary property '" prop_DepanEstimateFFT "' has the wrong size. This should never happen.", frameCtx);
            vsapi->freeFrame(prev);
            vsapi->freeFrame(cur);
            return NULL;
        }
        if (d->zoommax != 1.0f) {
            if (d->fftsize != (size_t)vsapi->propGetDataSize(prev_props, prop_DepanEstimateFFT2, 0, &err) ||
                d->fftsize != (size_t)vsapi->propGetDataSize(cur_props, prop_DepanEstimateFFT2, 0, &err)) {
                vsapi->setFilterError("DepanEstimate: temporary property '" prop_DepanEstimateFFT2 "' has the wrong size. This should never happen.", frameCtx);
                vsapi->freeFrame(prev);
                vsapi->freeFrame(cur);
                return NULL;
            }
        }

        const fftwf_complex *fftprev = (const fftwf_complex *)vsapi->propGetData(prev_props, prop_DepanEstimateFFT, 0, &err);
        const fftwf_complex *fftcur = (const fftwf_complex *)vsapi->propGetData(cur_props, prop_DepanEstimateFFT, 0, &err);

        // memory for correlation matrice
        fftwf_complex *correl = (fftwf_complex *)fftwf_malloc(d->fftsize);

        float dx1, dy1, trust1;

        // prepare correlation data = mult fftsrc* by fftprev
        mult_conj_data2d(fftcur, fftprev, correl, d->winx, d->winy);
        // make inverse fft of prepared correl data
        fftwf_execute_dft_c2r(d->planinv, correl, (float *)correl);
        // now correl is is true correlation surface
        // find global motion vector as maximum on correlation sufrace
        // save vector to motion table
        get_motion_vector((float *)correl, d->winx, d->winy, d->trust_limit, d->dxmax, d->dymax, d->stab, d->fields, top_field, d->pixaspect, &dx1, &dy1, &trust1);


        int winleft = d->wleft;

        VSFrameRef *dst = vsapi->copyFrame(cur, core);
        uint8_t *dstp = NULL;
        int dst_stride = 0;

        if (d->show) { // show correlation sufrace
            dstp = vsapi->getWritePtr(dst, 0);
            dst_stride = vsapi->getStride(dst, 0);

            showcorrelation((float *)correl, d->winx, d->winy, dstp, dst_stride, winleft, d->wtop, d->pixel_max);
        }

        fftwf_free(correl);

        float motionx, motiony, motionzoom, trust;

        if (d->zoommax == 1.0f) { // NO ZOOM
            motionzoom = 1.0f; //no zoom
            motionx = dx1;
            motiony = dy1;
            trust = trust1;
        } else { // ZOOM, calculate 2 data sets (left and right)
            int winleft2 = d->wleft + d->vi->width / 2; // left edge of right (2)fft window

            const fftwf_complex *fftprev2 = (const fftwf_complex *)vsapi->propGetData(prev_props, prop_DepanEstimateFFT2, 0, &err);
            const fftwf_complex *fftcur2 = (const fftwf_complex *)vsapi->propGetData(cur_props, prop_DepanEstimateFFT2, 0, &err);

            fftwf_complex *correl2 = (fftwf_complex *)fftwf_malloc(d->fftsize);

            float dx2, dy2, trust2;

            // right window
            // prepare correlation data = mult fftsrc* by fftprev
            mult_conj_data2d(fftcur2, fftprev2, correl2, d->winx, d->winy);
            // make inverse fft of prepared correl data
            fftwf_execute_dft_c2r(d->planinv, correl2, (float *)correl2);
            // now correl is is true correlation surface
            // find global motion vector as maximum on correlation sufrace
            // save vector to motion table
            get_motion_vector((float *)correl2, d->winx, d->winy, d->trust_limit, d->dxmax, d->dymax, d->stab, d->fields, top_field, d->pixaspect, &dx2, &dy2, &trust2);

            // now we have 2 motion data sets for left and right windows
            // estimate zoom factor
            float zoom = 1.0f + (dx2 - dx1) / (winleft2 - winleft);
            if ((dx1 != 0.0f) && (dx2 != 0.0f) && (fabsf(zoom - 1.0f) < (d->zoommax - 1.0f))) { // if motion data and zoom good
                motionx = (dx1 + dx2) / 2.0f;
                motiony = (dy1 + dy2) / 2.0f;
                motionzoom = zoom;
                trust = VSMIN(trust1, trust2);
            } else { // bad zoom,
                motionx = 0.0f;
                motiony = 0.0f;
                motionzoom = 1.0f;
                trust = VSMIN(trust1, trust2);
            }

            if (d->show) // show correlation sufrace
                showcorrelation((float *)correl2, d->winx, d->winy, dstp, dst_stride, winleft2, d->wtop, d->pixel_max);

            fftwf_free(correl2);
        }

        vsapi->freeFrame(prev);
        vsapi->freeFrame(cur);

        VSMap *dst_props = vsapi->getFramePropsRW(dst);

        vsapi->propDeleteKey(dst_props, prop_DepanEstimateFFT);
        vsapi->propDeleteKey(dst_props, prop_DepanEstimateFFT2);

        if (n == 0) {
            motionx = motiony = trust = 0.0f;
            motionzoom = 1.0f;
        }

        vsapi->propSetFloat(dst_props, prop_DepanEstimateX, motionx, paReplace);
        vsapi->propSetFloat(dst_props, prop_DepanEstimateY, motiony, paReplace);
        vsapi->propSetFloat(dst_props, prop_DepanEstimateZoom, motionzoom, paReplace);
        vsapi->propSetFloat(dst_props, prop_DepanEstimateTrust, trust, paReplace);

        return dst;
    }

    return NULL;
}


static const VSFrameRef *VS_CC depanEstimateStage3GetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    DepanEstimateData *d = (DepanEstimateData *)*instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(VSMAX(0, n - 1), d->clip, frameCtx);
        vsapi->requestFrameFilter(n, d->clip, frameCtx);
        vsapi->requestFrameFilter(VSMIN(n + 1, d->vi->numFrames - 1), d->clip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src[3];
        src[0] = vsapi->getFrameFilter(VSMAX(0, n - 1), d->clip, frameCtx);
        src[1] = vsapi->getFrameFilter(n, d->clip, frameCtx);
        src[2] = vsapi->getFrameFilter(VSMIN(n + 1, d->vi->numFrames - 1), d->clip, frameCtx);

        const VSMap *src_props[3];

        float trust[3];

        for (int i = 0; i < 3; i++) {
            src_props[i] = vsapi->getFramePropsRO(src[i]);

            int err;
            trust[i] = (float)vsapi->propGetFloat(src_props[i], prop_DepanEstimateTrust, 0, &err);
            if (err) {
                vsapi->setFilterError("DepanEstimate: temporary property '" prop_DepanEstimateTrust "' not found in input frame. This should never happen.", frameCtx);
                for (int j = 0; j < 3; j++)
                    vsapi->freeFrame(src[j]);
                return NULL;
            }
        }

        vsapi->freeFrame(src[0]);
        vsapi->freeFrame(src[2]);

        int err[3];
        float motionx = (float)vsapi->propGetFloat(src_props[1], prop_DepanEstimateX, 0, &err[0]);
        float motiony = (float)vsapi->propGetFloat(src_props[1], prop_DepanEstimateY, 0, &err[1]);
        float motionzoom = (float)vsapi->propGetFloat(src_props[1], prop_DepanEstimateZoom, 0, &err[2]);

        if (err[0] || err[1] || err[2]) {
            vsapi->setFilterError("DepanEstimate: some temporary property was not found in input frame. This should never happen.", frameCtx);
            vsapi->freeFrame(src[1]);
            return NULL;
        }

        // check scenechanges in range, as sharp decreasing of trust
        if (n - 1 >= 0 && n < d->vi->numFrames && trust[1] < d->trust_limit * 2.0f && trust[1] < 0.5f * trust[0]) {
            // very sharp decrease of not very big trust, probably due to scenechange
            motionx = 0.0f;
            motiony = 0.0f;
            motionzoom = 1.0f;
        }
        if (n >= 0 && n + 1 < d->vi->numFrames && trust[1] < d->trust_limit * 2.0f && trust[1] < 0.5f * trust[2]) {
            // very sharp decrease of not very big trust, probably due to scenechange
            motionx = 0.0f;
            motiony = 0.0f;
            motionzoom = 1.0f;
        }

        VSFrameRef *dst = vsapi->copyFrame(src[1], core);
        vsapi->freeFrame(src[1]);

        VSMap *dst_props = vsapi->getFramePropsRW(dst);

        vsapi->propDeleteKey(dst_props, prop_DepanEstimateX);
        vsapi->propDeleteKey(dst_props, prop_DepanEstimateY);
        vsapi->propDeleteKey(dst_props, prop_DepanEstimateZoom);
        vsapi->propDeleteKey(dst_props, prop_DepanEstimateTrust);

        vsapi->propSetFloat(dst_props, prop_Depan_dx, motionx, paReplace);
        vsapi->propSetFloat(dst_props, prop_Depan_dy, motiony, paReplace);
        vsapi->propSetFloat(dst_props, prop_Depan_zoom, motionzoom, paReplace);
        vsapi->propSetFloat(dst_props, prop_Depan_rot, 0, paReplace);

        if (d->info) {
#define INFO_SIZE 128
            char info[INFO_SIZE + 1] = { 0 };

            snprintf(info, INFO_SIZE, "fn=%d dx=%.2f dy=%.2f zoom=%.5f trust=%.2f", n, motionx, motiony, motionzoom, trust[1]);
#undef INFO_SIZE

            vsapi->propSetData(dst_props, prop_DepanEstimate_info, info, -1, paReplace);
        }

        return dst;
    }

    return NULL;
}


// Defined in DCTFFTW.cpp because that file was the first to use fftw.
extern std::mutex g_fftw_plans_mutex;


static void VS_CC depanEstimateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    DepanEstimateData *d = (DepanEstimateData *)instanceData;

    vsapi->freeNode(d->clip);

    {
        std::lock_guard<std::mutex> guard(g_fftw_plans_mutex);
        if (d->stage == 1)
            fftwf_destroy_plan(d->plan);
        else if (d->stage == 2)
            fftwf_destroy_plan(d->planinv);
    }
    if (d->stage == 1)
        fftwf_free(d->unused_array);

    free(d);
}


static void VS_CC depanEstimateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    DepanEstimateData d;
    memset(&d, 0, sizeof(d));

    int err;

    d.trust_limit = (float)vsapi->propGetFloat(in, "trust", 0, &err);
    if (err)
        d.trust_limit = 4.0f;

    d.winx = int64ToIntS(vsapi->propGetInt(in, "winx", 0, &err));

    d.winy = int64ToIntS(vsapi->propGetInt(in, "winy", 0, &err));

    d.wleft = int64ToIntS(vsapi->propGetInt(in, "wleft", 0, &err));
    if (err)
        d.wleft = -1;

    d.wtop = int64ToIntS(vsapi->propGetInt(in, "wtop", 0, &err));
    if (err)
        d.wtop = -1;

    d.dxmax = int64ToIntS(vsapi->propGetInt(in, "dxmax", 0, &err));
    if (err)
        d.dxmax = -1;

    d.dymax = int64ToIntS(vsapi->propGetInt(in, "dymax", 0, &err));
    if (err)
        d.dymax = -1;

    d.zoommax = (float)vsapi->propGetFloat(in, "zoommax", 0, &err);
    if (err)
        d.zoommax = 1.0f;

    d.stab = (float)vsapi->propGetFloat(in, "stab", 0, &err);
    if (err)
        d.stab = 1.0f;

    d.pixaspect = (float)vsapi->propGetFloat(in, "pixaspect", 0, &err);
    if (err)
        d.pixaspect = 1.0f;

    d.info = !!vsapi->propGetInt(in, "info", 0, &err);

    d.show = !!vsapi->propGetInt(in, "show", 0, &err);

    d.fields = !!vsapi->propGetInt(in, "fields", 0, &err);

    d.tff = !!vsapi->propGetInt(in, "tff", 0, &d.tff_exists);
    d.tff_exists = !d.tff_exists;


    if (d.trust_limit < 0.0f || d.trust_limit > 100.0f) {
        vsapi->setError(out, "DepanEstimate: trust must be between 0.0 and 100.0 (inclusive).");
        return;
    }

    if (d.pixaspect <= 0.0f) {
        vsapi->setError(out, "DepanEstimate: pixaspect must be positive.");
        return;
    }


    d.clip = vsapi->propGetNode(in, "clip", 0, NULL);
    d.vi = vsapi->getVideoInfo(d.clip);


    if (!isConstantFormat(d.vi) ||
        (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray) ||
        (d.vi->format->sampleType == stInteger && d.vi->format->bitsPerSample > 16) ||
        (d.vi->format->sampleType == stFloat && d.vi->format->bitsPerSample != 32)) {
        vsapi->setError(out, "DepanEstimate: clip must have constant format and dimensions, it must be YUV or Gray, and it must be 8..16 bit integer or 32 bit float.");
        vsapi->freeNode(d.clip);
        return;
    }

    // used only for luma
    if (d.vi->format->sampleType == stFloat)
        d.pixel_max = 1;
    else
        d.pixel_max = (1 << d.vi->format->bitsPerSample) - 1;


    int wleft0 = d.wleft; // save
    if (d.wleft < 0)
        d.wleft = 0; // auto

    // check and set fft window x size

    if (d.winx > d.vi->width - d.wleft) {
        vsapi->setError(out, "DepanEstimate: winx must not be greater than width-wleft.");
        vsapi->freeNode(d.clip);
        return;
    }
    int i;
    int wx;
    if (d.winx == 0) { // auto
        d.winx = d.vi->width - d.wleft;
        // find max fft window size (power of 2)
        wx = 1;
        for (i = 0; i < 13; i++) {
            if (wx * 2 <= d.winx)
                wx = wx * 2;
        }
        d.winx = wx;
    }

    if (d.zoommax != 1.0f) {
        d.winx = d.winx / 2; // devide window x by 2 part (left and right)
        if (wleft0 < 0)
            d.wleft = (d.vi->width - d.winx * 2) / 4;
    } else if (wleft0 < 0)
        d.wleft = (d.vi->width - d.winx) / 2;


    int wtop0 = d.wtop; // save
    if (d.wtop < 0)
        d.wtop = 0; // auto

    // check and set fft window y size
    if (d.winy > d.vi->height - d.wtop) {
        vsapi->setError(out, "DepanEstimate: winy must not be greater than height-wtop.");
        vsapi->freeNode(d.clip);
        return;
    }
    int wy;
    if (d.winy == 0) {
        d.winy = d.vi->height - d.wtop; // start value
        // find max fft window size (power of 2)
        wy = 1;
        for (i = 0; i < 13; i++) {
            if (wy * 2 <= d.winy)
                wy = wy * 2;
        }
        d.winy = wy;
    }

    if (wtop0 < 0)
        d.wtop = (d.vi->height - d.winy) / 2; // auto

    // max dx shift must be less than winx/2
    if (d.dxmax < 0)
        d.dxmax = d.winx / 4; // default
    if (d.dymax < 0)
        d.dymax = d.winy / 4; // default

    if (d.dxmax >= d.winx / 2) {
        vsapi->setError(out, "DepanEstimate: dxmax must be less than winx/2.");
        vsapi->freeNode(d.clip);
        return;
    }
    if (d.dymax >= d.winy / 2) {
        vsapi->setError(out, "DepanEstimate: dymax must be less than winy/2.");
        vsapi->freeNode(d.clip);
        return;
    }


    int winxpadded = (d.winx / 2 + 1) * 2;
    d.fftsize = d.winy * winxpadded / 2; //complex
    d.fftsize *= sizeof(fftwf_complex);


    d.unused_array = (fftwf_complex *)fftwf_malloc(d.fftsize);
    {
        std::lock_guard<std::mutex> guard(g_fftw_plans_mutex);
        // in-place transforms
        d.plan = fftwf_plan_dft_r2c_2d(d.winy, d.winx, (float *)d.unused_array, d.unused_array, FFTW_ESTIMATE);    // direct fft
        d.planinv = fftwf_plan_dft_c2r_2d(d.winy, d.winx, d.unused_array, (float *)d.unused_array, FFTW_ESTIMATE); // inverse fft
    }


    DepanEstimateData *data1 = (DepanEstimateData *)malloc(sizeof(d));
    DepanEstimateData *data2 = (DepanEstimateData *)malloc(sizeof(d));
    DepanEstimateData *data3 = (DepanEstimateData *)malloc(sizeof(d));

    *data1 = *data2 = *data3 = d;

    data1->stage = 1;
    data2->stage = 2;
    data3->stage = 3;

    VSPlugin *std_plugin = vsapi->getPluginById("com.vapoursynth.std", core);

    vsapi->createFilter(in, out, "DepanEstimateStage1", depanEstimateInit, depanEstimateStage1GetFrame, depanEstimateFree, fmParallel, 0, data1, core);

    if (vsapi->getError(out)) {
        depanEstimateFree(data1, core, vsapi);
        depanEstimateFree(data2, core, vsapi);
        depanEstimateFree(data3, core, vsapi);
        return;
    }

    VSMap *args = vsapi->createMap();
    VSNodeRef *node = vsapi->propGetNode(out, "clip", 0, NULL);
    vsapi->propSetNode(args, "clip", node, paReplace);
    vsapi->freeNode(node);
    vsapi->clearMap(out);

    VSMap *ret = vsapi->invoke(std_plugin, "Cache", args);
    vsapi->freeMap(args);
    if (vsapi->getError(ret)) {
        vsapi->setError(out, std::string("DepanEstimate: failed to invoke std.Cache: ").append(vsapi->getError(ret)).c_str());
        vsapi->freeMap(ret);
        depanEstimateFree(data1, core, vsapi);
        depanEstimateFree(data2, core, vsapi);
        depanEstimateFree(data3, core, vsapi);
        return;
    }

    data2->clip = vsapi->propGetNode(ret, "clip", 0, NULL);
    vsapi->freeMap(ret);

    vsapi->createFilter(in, out, "DepanEstimateStage2", depanEstimateInit, depanEstimateStage2GetFrame, depanEstimateFree, fmParallel, 0, data2, core);

    if (vsapi->getError(out)) {
        depanEstimateFree(data1, core, vsapi);
        depanEstimateFree(data2, core, vsapi);
        depanEstimateFree(data3, core, vsapi);
        return;
    }

    args = vsapi->createMap();
    node = vsapi->propGetNode(out, "clip", 0, NULL);
    vsapi->propSetNode(args, "clip", node, paReplace);
    vsapi->freeNode(node);
    vsapi->clearMap(out);

    ret = vsapi->invoke(std_plugin, "Cache", args);
    vsapi->freeMap(args);
    if (vsapi->getError(ret)) {
        vsapi->setError(out, std::string("DepanEstimate: failed to invoke std.Cache: ").append(vsapi->getError(ret)).c_str());
        vsapi->freeMap(ret);
        depanEstimateFree(data1, core, vsapi);
        depanEstimateFree(data2, core, vsapi);
        depanEstimateFree(data3, core, vsapi);
        return;
    }

    data3->clip = vsapi->propGetNode(ret, "clip", 0, NULL);
    vsapi->freeMap(ret);

    vsapi->createFilter(in, out, "DepanEstimateStage3", depanEstimateInit, depanEstimateStage3GetFrame, depanEstimateFree, fmParallel, 0, data3, core);

    if (vsapi->getError(out)) {
        depanEstimateFree(data1, core, vsapi);
        depanEstimateFree(data2, core, vsapi);
        depanEstimateFree(data3, core, vsapi);
        return;
    }

    if (d.info) {
        if (!invokeFrameProps(prop_DepanEstimate_info, out, core, vsapi)) {
            vsapi->setError(out, std::string("DepanEstimate: failed to invoke text.FrameProps: ").append(vsapi->getError(out)).c_str());
            depanEstimateFree(data1, core, vsapi);
            depanEstimateFree(data2, core, vsapi);
            depanEstimateFree(data3, core, vsapi);
            return;
        }
    }
}


typedef void (*CompensateFunction)(uint8_t *dstp, int dst_pitch, const uint8_t *srcp, int src_pitch, int row_size, int height, const transform *tr, int mirror, int border, int *work1row_size, int blurmax);


typedef struct DepanCompensateData {
    VSNodeRef *clip;
    VSNodeRef *data;
    float offset;
    int subpixel;
    float pixaspect;
    int matchfields;
    int mirror;
    int blur;
    int info;
    int fields;
    int tff;
    int tff_exists;

    const VSVideoInfo *vi;
    int intoffset;
    float xcenter;
    float ycenter;

    CompensateFunction compensate_plane;
} DepanCompensateData;


static void VS_CC depanCompensateInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    DepanCompensateData *d = (DepanCompensateData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


//****************************************************************************
//  motion to transform
//  get  coefficients for coordinates transformation,
//  which defines source (xsrc, ysrc)  for current destination (x,y)
//
//  delta is fracture of deformation (from 0 to 1 for forward,  from -1 to 0 for backward time direction)
//
//
//  return:
//  vector:
//   t[0] = dxc, t[1] = dxx, t[2] = dxy, t[3] = dyc, t[4] = dyx, t[5] = dyy
//
//
//   xsrc = dxc + dxx*x + dxy*y
//   ysrc = dyc + dyx*x + dyy*y
//
//  if no rotation, then dxy, dyx = 0,
//  if no rotation and zoom, then also dxx, dyy = 1.
//
static void motion2transform(float dx1, float dy1, float rot, float zoom1, float pixaspect, float xcenter, float ycenter, int forward, float fractoffset, transform *tr) {
    const float PI = 3.1415926535897932384626433832795f;
    float rotradian, sinus, cosinus, dx, dy, zoom;

    // fractoffset > 0 for forward, <0 for backward
    dx = fractoffset * dx1;
    dy = fractoffset * dy1;
    rotradian = fractoffset * rot * PI / 180; // from degree to radian
    if (fabsf(rotradian) < 1e-6f)
        rotradian = 0.0f; // for some stability of rounding precision
    zoom = expf(fractoffset * logf(zoom1)); // zoom**(fractoffset) = exp(fractoffset*ln(zoom))
    if (fabsf(zoom - 1.0f) < 1e-6f)
        zoom = 1.0f; // for some stability of rounding precision

    sinus = sinf(rotradian);
    cosinus = cosf(rotradian);

    //    xcenter = row_size_p*(1+uv)/2.0;      //  middle x
    //    ycenter = height_p*(1+uv)/2.0;       //  middle y

    if (forward) { //  get coefficients for forward
        tr->dxc = xcenter + (-xcenter * cosinus + ycenter / pixaspect * sinus) * zoom + dx; // dxc            /(1+uv);
        tr->dxx = cosinus * zoom;                                                           // dxx
        tr->dxy = -sinus / pixaspect * zoom;                                                // dxy

        tr->dyc = ycenter + (((-ycenter) / pixaspect * cosinus + (-xcenter) * sinus) * zoom + dy) * pixaspect; // dyc      /(1+uv);
        tr->dyx = sinus * zoom * pixaspect;                                                                    // dyx
        tr->dyy = cosinus * zoom;                                                                              // dyy
    } else { // coefficients for backward
        tr->dxc = xcenter + ((-xcenter + dx) * cosinus - ((-ycenter) / pixaspect + dy) * sinus) * zoom; //     /(1+uv);
        tr->dxx = cosinus * zoom;
        tr->dxy = -sinus / pixaspect * zoom;

        tr->dyc = ycenter + (((-ycenter) / pixaspect + dy) * cosinus + (-xcenter + dx) * sinus) * zoom * pixaspect; //      /(1+uv);
        tr->dyx = sinus * zoom * pixaspect;
        tr->dyy = cosinus * zoom;
    }
}


//****************************************************************************
//  get  summary coefficients for summary combined coordinates transformation,
//  transform_BA = fransform_B ( transform_A )
//   t[0] = dxc, t[1] = dxx, t[2] = dxy, t[3] = dyc, t[4] = dyx, t[5] = dyy
//void sumtransform(float ta[], float tb[], float tba[])
static void sumtransform(const transform *ta, const transform *tb, transform *tba) {
    transform temp;

    temp.dxc = tb->dxc + tb->dxx * ta->dxc + tb->dxy * ta->dyc;

    temp.dxx = tb->dxx * ta->dxx + tb->dxy * ta->dyx;

    temp.dxy = tb->dxx * ta->dxy + tb->dxy * ta->dyy;

    temp.dyc = tb->dyc + tb->dyx * ta->dxc + tb->dyy * ta->dyc;

    temp.dyx = tb->dyx * ta->dxx + tb->dyy * ta->dyx;

    temp.dyy = tb->dyx * ta->dxy + tb->dyy * ta->dyy;

    memcpy(tba, &temp, sizeof(temp));
}


#define MIRROR_TOP 1
#define MIRROR_BOTTOM 2
#define MIRROR_LEFT 4
#define MIRROR_RIGHT 8

//****************************************************************************
// move plane of nextp frame to dstp for motion compensation by trc, trm with NEAREST pixels
//
void compensate_plane_nearest(uint8_t *dstp, int dst_pitch, const uint8_t *srcp, int src_pitch, int row_size, int height, const transform *tr, int mirror, int border, int *work1row_size, int blurmax) {
    // if border >=0, then we fill empty edge (border) pixels by that value
    // work1row_size is work array, it must have size >= 1*row_size

    // if mirror > 0, than we fill empty edge (border) pixels by mirrored (reflected) pixels from border,
    // according to bit set of "mirror" parameter:                   (added in v.0.9)
    // mirror = 1 - only top
    // mirror = 2 - only bottom
    // mirror = 4 - only left
    // mirror = 8 - only right
    // any combination - sum of above

    int h, row;
    int rowleft, hlow;
    float xsrc, ysrc;
    int w0;
    int inttr0;
    int *rowleftwork = work1row_size;

    int smoothed;
    int blurlen;
    int i;

    // for mirror

    int mtop = mirror & MIRROR_TOP;
    int mbottom = mirror & MIRROR_BOTTOM;
    int mleft = mirror & MIRROR_LEFT;
    int mright = mirror & MIRROR_RIGHT;

    //    select if rotation, zoom?

    if (tr->dxy == 0.0f && tr->dyx == 0.0f && tr->dxx == 1.0f && tr->dyy == 1.0f) { // only translation - fast

        for (h = 0; h < height; h++) {

            ysrc = tr->dyc + h;
            hlow = (int)floorf(ysrc + 0.5f);

            inttr0 = (int)floorf(tr->dxc + 0.5f);
            rowleft = inttr0;
            //            xsrc = tr[0];
            //            rowleft = (int)floor(xsrc);  // low

            if (hlow < 0 && mtop)
                hlow = -hlow; // mirror borders
            if (hlow >= height && mbottom)
                hlow = height + height - hlow - 2;

            w0 = hlow * src_pitch;
            if ((hlow >= 0) && (hlow < height)) { // middle lines


                for (row = 0; row < row_size; row++) {
                    rowleft = inttr0 + row;

                    //                    xsrc = tr[0]+row;            // hided simle formulas,
                    //                    rowleft = (int)floor(xsrc);  // but slow

                    //  x,y point is in square: (rowleft,hlow) to (rowleft+1,hlow+1)

                    if ((rowleft >= 0) && (rowleft < row_size)) {
                        dstp[row] = srcp[w0 + rowleft];
                    } else if (rowleft < 0 && mleft) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, -rowleft);
                            smoothed = 0;
                            for (i = -rowleft - blurlen + 1; i <= -rowleft; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                        }
                    } else if (rowleft >= row_size && mright) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, rowleft - row_size + 1);
                            smoothed = 0;
                            for (i = row_size + row_size - rowleft - 2; i < row_size + row_size - rowleft - 2 + blurlen; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2]; // not very precise - may be bicubic?
                        }
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }


                } // end for
            } else if (border >= 0) { // out lines
                for (row = 0; row < row_size; row++) {
                    dstp[row] = border;
                }
            }

            dstp += dst_pitch; // next line
        }
    }
    //-----------------------------------------------------------------------------
    else if (tr->dxy == 0.0f && tr->dyx == 0.0f) { // no rotation, only zoom and translation  - fast

        // prepare positions   (they are not dependent from h) for fast processing
        for (row = 0; row < row_size; row++) {
            xsrc = tr->dxc + tr->dxx * row;
            rowleftwork[row] = (int)floorf(xsrc + 0.5f);
            rowleft = rowleftwork[row];
        }


        for (h = 0; h < height; h++) {

            ysrc = tr->dyc + tr->dyy * h;

            hlow = (int)floorf(ysrc + 0.5f);

            if (hlow < 0 && mtop)
                hlow = -hlow; // mirror borders
            if (hlow >= height && mbottom)
                hlow = height + height - hlow - 2;

            w0 = hlow * src_pitch;
            if ((hlow >= 0) && (hlow < height)) { // incide


                for (row = 0; row < row_size; row++) {

                    //                    xsrc = tr[0]+tr[1]*row;
                    //                    rowleft = floor(xsrc);
                    rowleft = rowleftwork[row]; //(int)(xsrc);

                    if ((rowleft >= 0) && (rowleft < row_size)) {
                        dstp[row] = srcp[w0 + rowleft];
                    } else if (rowleft < 0 && mleft) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, -rowleft);
                            smoothed = 0;
                            for (i = -rowleft - blurlen + 1; i <= -rowleft; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                        }
                    } else if (rowleft >= row_size && mright) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, rowleft - row_size + 1);
                            smoothed = 0;
                            for (i = row_size + row_size - rowleft - 2; i < row_size + row_size - rowleft - 2 + blurlen; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2]; // not very precise - may be bicubic?
                        }
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }
                } // end for
            } else if (border >= 0) { // out lines
                for (row = 0; row < row_size; row++) {
                    dstp[row] = border;
                }
            }

            dstp += dst_pitch; // next line
        }
    }
    //-----------------------------------------------------------------------------
    else { // rotation, zoom and translation - slow

        for (h = 0; h < height; h++) {

            xsrc = tr->dxc + tr->dxy * h; // part not dependent from row
            ysrc = tr->dyc + tr->dyy * h;

            for (row = 0; row < row_size; row++) {

                rowleft = (int)(xsrc + 0.5f); // use simply fast (int), not floor(), since followed check

                //                if (xsrc  < rowleft) {
                //                    rowleft -=1;
                //                }

                hlow = (int)(ysrc + 0.5f); // use simply fast  (int), not floor(), since followed check

                //                if (ysrc <  hlow) {
                //                    hlow -=1;
                //                }


                if ((rowleft >= 0) && (rowleft < row_size) && (hlow >= 0) && (hlow < height)) {
                    dstp[row] = srcp[hlow * src_pitch + rowleft];
                } else { // try fill by mirror. Probability of these cases is small
                    if (hlow < 0 && mtop)
                        hlow = -hlow; // mirror borders
                    if (hlow >= height && mbottom)
                        hlow = height + height - hlow - 2;
                    if (rowleft < 0 && mleft)
                        rowleft = -rowleft;
                    if (rowleft >= row_size && mright)
                        rowleft = row_size + row_size - rowleft - 2;
                    // check mirrowed
                    if ((rowleft >= 0) && (rowleft < row_size) && (hlow >= 0) && (hlow < height)) {
                        dstp[row] = srcp[hlow * src_pitch + rowleft];
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }
                }
                xsrc += tr->dxx; // next
                ysrc += tr->dyx;
            } // end for row

            dstp += dst_pitch; // next line
        } //end for h
    } // end if rotation
}


//****************************************************************************
// move plane of nextp frame to dstp for motion compensation by transform tr[]
// with BILINEAR interpolation of discrete neighbour source pixels
//   t[0] = dxc, t[1] = dxx, t[2] = dxy, t[3] = dyc, t[4] = dyx, t[5] = dyy
//
void compensate_plane_bilinear(uint8_t *dstp, int dst_pitch, const uint8_t *srcp, int src_pitch, int row_size, int height, const transform *tr, int mirror, int border, int *work2row_size4356, int blurmax) {
    // work2row_size is work array, it must have size >= 2*row_size

    int h, row;
    int pixel;
    int rowleft, hlow;
    float sx, sy;
    //    float t0, t1, t2, t3, c0,c1,c2,c3;
    float xsrc, ysrc;
    int w0;
    int intcoef[66];
    int intcoef2d[4];
    int ix2, iy2;
    int i, j;
    int inttr0;
    int *rowleftwork = work2row_size4356;
    int *ix2work = rowleftwork + row_size;
    int *intcoef2dzoom0 = ix2work + row_size; //[66][66]; // 4356
    int *intcoef2dzoom = intcoef2dzoom0;

    int w;

    int smoothed;
    int blurlen;

    // for mirror
    int mtop = mirror & MIRROR_TOP;
    int mbottom = mirror & MIRROR_BOTTOM;
    int mleft = mirror & MIRROR_LEFT;
    int mright = mirror & MIRROR_RIGHT;

    int rowgoodstart, rowgoodend, rowbadstart, rowbadend;

    // prepare interpolation coefficients tables
    // for position of xsrc in integer grid
    //        sx = (xsrc-rowleft);
    //        sy = (ysrc-hlow);
    //
    //            cx0 = (1-sx);
    //            cx1 = sx;
    //
    //            cy0 = (1-sy);
    //            cy1 = sy;
    //
    // now sx = i/32, sy = j/32  (discrete approximation)

    // float coeff. are changed by integer coeff. scaled by 32
    for (i = 0; i <= 32; i += 1) {
        intcoef[i * 2] = (32 - i);
        intcoef[i * 2 + 1] = i;
    }

    //    select if rotation, zoom?

    if (tr->dxy == 0.0f && tr->dyx == 0.0f && tr->dxx == 1.0f && tr->dyy == 1.0f) { // only translation - fast

        for (h = 0; h < height; h++) {

            ysrc = tr->dyc + h;
            hlow = (int)floorf(ysrc);
            iy2 = 2 * ((int)floorf((ysrc - hlow) * 32));

            inttr0 = (int)floorf(tr->dxc);
            rowleft = inttr0;
            //            xsrc = tr[0];
            //            rowleft = (int)floor(xsrc);  // low
            ix2 = 2 * ((int)floorf((tr->dxc - rowleft) * 32));

            for (j = 0; j < 2; j++) {
                for (i = 0; i < 2; i++) {
                    intcoef2d[j * 2 + i] = (intcoef[j + iy2] * intcoef[i + ix2]); // 4 coeff. for bilinear 2D
                }
            }

            if (hlow < 0 && mtop)
                hlow = -hlow; // mirror borders
            if (hlow >= height && mbottom)
                hlow = height + height - hlow - 2;

            w0 = hlow * src_pitch;

            if ((hlow >= 0) && (hlow < height - 1)) { // middle lines

                if (inttr0 >= 0) {
                    rowgoodstart = 0;
                    rowgoodend = row_size - 1 - inttr0;
                    rowbadstart = rowgoodend;
                    rowbadend = row_size;
                } else {
                    rowbadstart = 0;
                    rowbadend = -inttr0;
                    rowgoodstart = rowbadend;
                    rowgoodend = row_size;
                }
                //                int rowgoodendpaired = (rowgoodend/2)*2; //even - but it was a little not optimal
                int rowgoodendpaired = rowgoodstart + ((rowgoodend - rowgoodstart) / 2) * 2; //even length - small fix in v.1.8
                w = w0 + inttr0 + rowgoodstart;
                for (row = rowgoodstart; row < rowgoodendpaired; row += 2) { // paired unroll for speed
                    //                    xsrc = tr[0]+row;            // hided simle formulas,
                    //                    rowleft = (int)floor(xsrc);  // but slow
                    //                    rowleft = inttr0 + row;
                    //                    w = w0+rowleft;
                    //  x,y point is in square: (rowleft,hlow) to (rowleft+1,hlow+1)
                    //                    if ( (rowleft >= 0) && (rowleft<row_size-1)  )
                    dstp[row] = (intcoef2d[0] * srcp[w] + intcoef2d[1] * srcp[w + 1] + intcoef2d[2] * srcp[w + src_pitch] + intcoef2d[3] * srcp[w + src_pitch + 1]) >> 10; // i.e. divide by 32*32
                    dstp[row + 1] = (intcoef2d[0] * srcp[w + 1] + intcoef2d[1] * srcp[w + 2] + intcoef2d[2] * srcp[w + src_pitch + 1] + intcoef2d[3] * srcp[w + src_pitch + 2]) >> 10; // i.e. divide by 32*32
                    w += 2;
                }
                for (row = rowgoodendpaired - 1; row < rowgoodend; row++) { // if odd, process  very last
                    w = w0 + inttr0 + row;
                    dstp[row] = (intcoef2d[0] * srcp[w] + intcoef2d[1] * srcp[w + 1] +
                                 intcoef2d[2] * srcp[w + src_pitch] + intcoef2d[3] * srcp[w + src_pitch + 1]) >>
                                10; // i.e. divide by 32*32
                }
                for (row = rowbadstart; row < rowbadend; row++) {
                    rowleft = inttr0 + row;
                    w = w0 + rowleft;

                    if (rowleft < 0 && mleft) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, -rowleft);
                            smoothed = 0;
                            for (i = -rowleft - blurlen + 1; i <= -rowleft; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                        }
                    } else if (rowleft >= row_size - 1 && mright) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, rowleft - row_size + 2);
                            smoothed = 0;
                            for (i = row_size + row_size - rowleft - 2; i < row_size + row_size - rowleft - 2 + blurlen; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2]; // not very precise - may be bicubic?
                        }
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }

                } // end for
            } else if (hlow == height - 1) { // edge (top, bottom) lines
                for (row = 0; row < row_size; row++) {
                    rowleft = inttr0 + row;
                    if ((rowleft >= 0) && (rowleft < row_size)) {
                        dstp[row] = srcp[w0 + rowleft]; // nearest pixel, may be bilinear is better
                    } else if (rowleft < 0 && mleft) {
                        dstp[row] = srcp[w0 - rowleft];
                    } else if (rowleft >= row_size - 1 && mright) {
                        dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2];
                    } else if (border >= 0) { // left or right
                        dstp[row] = border;
                    }
                }
            } else if (border >= 0) { // out lines
                for (row = 0; row < row_size; row++) {
                    dstp[row] = border;
                }
            }

            dstp += dst_pitch; // next line
        }
    }
    //-----------------------------------------------------------------------------
    else if (tr->dxy == 0.0f && tr->dyx == 0.0f) { // no rotation, only zoom and translation  - fast

        // prepare positions   (they are not dependent from h) for fast processing
        for (row = 0; row < row_size; row++) {
            xsrc = tr->dxc + tr->dxx * row;
            rowleftwork[row] = (int)floorf(xsrc);
            rowleft = rowleftwork[row];
            ix2work[row] = 2 * ((int)floorf((xsrc - rowleft) * 32));
        }

        for (j = 0; j < 66; j++) {
            for (i = 0; i < 66; i++) {
                intcoef2dzoom[i] = (intcoef[j] * intcoef[i]); //  coeff. for bilinear 2D
            }
            intcoef2dzoom += 66;
        }
        intcoef2dzoom -= 66 * 66; //restore

        for (h = 0; h < height; h++) {

            ysrc = tr->dyc + tr->dyy * h;

            hlow = (int)floorf(ysrc);
            iy2 = 2 * ((int)floorf((ysrc - hlow) * 32));

            if (hlow < 0 && mtop)
                hlow = -hlow; // mirror borders
            if (hlow >= height && mbottom)
                hlow = height + height - hlow - 2;

            w0 = hlow * src_pitch;

            if ((hlow >= 0) && (hlow < height - 1)) { // incide

                intcoef2dzoom = intcoef2dzoom0;
                intcoef2dzoom += iy2 * 66;


                for (row = 0; row < row_size; row++) {

                    //                    xsrc = tr[0]+tr[1]*row;
                    rowleft = rowleftwork[row]; //(int)(xsrc);
                    //                    rowleft = floor(xsrc);

                    //  x,y point is in square: (rowleft,hlow) to (rowleft+1,hlow+1)

                    if ((rowleft >= 0) && (rowleft < row_size - 1)) {

                        //                        ix2 = 2*((int)((xsrc-rowleft)*32));
                        ix2 = ix2work[row];
                        w = w0 + rowleft;

                        //                        pixel = ( intcoef[iy2]*(intcoef[ix2]*srcp[w] + intcoef[ix2+1]*srcp[w+1] ) +
                        //                                intcoef[iy2+1]*(intcoef[ix2]*srcp[w+src_pitch] + intcoef[ix2+1]*srcp[w+src_pitch+1] ) )/1024;
                        pixel = (intcoef2dzoom[ix2] * srcp[w] + intcoef2dzoom[ix2 + 1] * srcp[w + 1] +
                                 intcoef2dzoom[ix2 + 66] * srcp[w + src_pitch] + intcoef2dzoom[ix2 + 67] * srcp[w + src_pitch + 1]) >>
                                10;

                        //                        dstp[row] = max(min(pixel,255),0);
                        dstp[row] = pixel; // maxmin disabled in v1.6
                    } else if (rowleft < 0 && mleft) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, -rowleft);
                            smoothed = 0;
                            for (i = -rowleft - blurlen + 1; i <= -rowleft; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 - rowleft];
                        }
                    } else if (rowleft >= row_size - 1 && mright) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, rowleft - row_size + 2);
                            smoothed = 0;
                            for (i = row_size + row_size - rowleft - 2; i < row_size + row_size - rowleft - 2 + blurlen; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2]; // not very precise - may be bicubic?
                        }
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }
                } // end for
            } else if (hlow == height - 1) { // edge ( bottom) lines
                for (row = 0; row < row_size; row++) {
                    rowleft = rowleftwork[row];
                    if ((rowleft >= 0) && (rowleft < row_size)) {
                        dstp[row] = srcp[rowleft + hlow * src_pitch]; // nearest pixel, may be bilinear is better
                    } else if (border >= 0) { // left or right
                        dstp[row] = border;
                    }
                }
            } else if (border >= 0) { // out lines
                for (row = 0; row < row_size; row++) {
                    dstp[row] = border;
                }
            }

            dstp += dst_pitch; // next line
        }
    }
    //-----------------------------------------------------------------------------
    else { // rotation, zoom and translation - slow

        for (h = 0; h < height; h++) {

            xsrc = tr->dxc + tr->dxy * h; // part not dependent from row
            ysrc = tr->dyc + tr->dyy * h;

            for (row = 0; row < row_size; row++) {

                rowleft = (int)(xsrc); // use simply fast (int), not floor(), since followed check >1
                sx = xsrc - rowleft;
                if (sx < 0) {
                    sx += 1;
                    rowleft -= 1;
                }

                hlow = (int)(ysrc); // use simply fast  (int), not floor(), since followed check >1
                sy = ysrc - hlow;
                if (sy < 0) {
                    sy += 1;
                    hlow -= 1;
                }

                //  x,y point is in square: (rowleft,hlow) to (rowleft+1,hlow+1)


                if ((rowleft >= 0) && (rowleft < row_size - 1) && (hlow >= 0) && (hlow < height - 1)) {

                    ix2 = ((int)(sx * 32)) << 1; // i.e. *2
                    iy2 = ((int)(sy * 32)) << 1; // i.e. *2
                    w0 = rowleft + hlow * src_pitch;

                    pixel = ((intcoef[ix2] * srcp[w0] + intcoef[ix2 + 1] * srcp[w0 + 1]) * intcoef[iy2] +
                             (intcoef[ix2] * srcp[w0 + src_pitch] + intcoef[ix2 + 1] * srcp[w0 + src_pitch + 1]) * intcoef[iy2 + 1]) >>
                            10;

                    //                    dstp[row] = max(min(pixel,255),0);
                    dstp[row] = pixel; //maxmin disabled in v1.6
                } else {
                    if (hlow < 0 && mtop)
                        hlow = -hlow; // mirror borders
                    if (hlow >= height && mbottom)
                        hlow = height + height - hlow - 2;
                    if (rowleft < 0 && mleft)
                        rowleft = -rowleft;
                    if (rowleft >= row_size && mright)
                        rowleft = row_size + row_size - rowleft - 2;
                    // check mirrowed
                    if ((rowleft >= 0) && (rowleft < row_size) && (hlow >= 0) && (hlow < height)) {
                        dstp[row] = srcp[hlow * src_pitch + rowleft];
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }
                }
                xsrc += tr->dxx; // next
                ysrc += tr->dyx;
            } // end for row

            dstp += dst_pitch; // next line
        } //end for h
    } // end if rotation
}


//****************************************************************************
// move plane of nextp frame to dstp for motion compensation by transform tr[]
// with BICUBIC interpolation of discrete neighbour source pixels
//
//   t[0] = dxc, t[1] = dxx, t[2] = dxy, t[3] = dyc, t[4] = dyx, t[5] = dyy
//
void compensate_plane_bicubic(uint8_t *dstp, int dst_pitch, const uint8_t *srcp, int src_pitch, int row_size, int height, const transform *tr, int mirror, int border, int *work2width1030, int blurmax) {
    // work2width1030 is integer work array, it must have size >= 2*row_size+1030

    int h, row;
    int pixel;
    int rowleft, hlow;
    float sx, sy;
    //    float t0, t1, t2, t3, c0,c1,c2,c3;
    float xsrc, ysrc;
    int w0;
    int ix4, iy4;
    int i, j;
    int inttr0, inttr3;
    int *rowleftwork = work2width1030;
    int *ix4work = work2width1030 + row_size;
    //    int intcoef[1030];
    int *intcoef = ix4work + row_size;
    int ts[4];
    int intcoef2d[16];

    int w;
    int smoothed;
    int blurlen;

    // for mirror
    int mtop = mirror & MIRROR_TOP;
    int mbottom = mirror & MIRROR_BOTTOM;
    int mleft = mirror & MIRROR_LEFT;
    int mright = mirror & MIRROR_RIGHT;

    // prepare interpolation coefficients tables
    // for position of xsrc in integer grid
    //        sx = (xsrc-rowleft);
    //        sy = (ysrc-hlow);
    //
    //            cx0 = -sx*(1-sx)*(1-sx);
    //            cx1 = (1-2*sx*sx+sx*sx*sx);
    //            cx2 =  sx*(1+sx-sx*sx);
    //            cx3 =  -sx*sx*(1-sx);
    //
    //            cy0 = -sy*(1-sy)*(1-sy);
    //            cy1 = (1-2*sy*sy+sy*sy*sy);
    //            cy2 =  sy*(1+sy-sy*sy);
    //            cy3 =  -sy*sy*(1-sy);
    //
    // now sx = i/256, sy = j/256  (discrete approximation)

    // float coeff. are changed by integer coeff. scaled by 256*256*256/8192 = 2048
    for (i = 0; i <= 256; i += 1) { // 257 steps, 1028 numbers
        intcoef[i * 4] = -((i * (256 - i) * (256 - i))) / 8192;
        intcoef[i * 4 + 1] = (256 * 256 * 256 - 2 * 256 * i * i + i * i * i) / 8192;
        intcoef[i * 4 + 2] = (i * (256 * 256 + 256 * i - i * i)) / 8192;
        intcoef[i * 4 + 3] = -(i * i * (256 - i)) / 8192;
    }

    //    select if rotation, zoom

    if (tr->dxy == 0.0f && tr->dyx == 0.0f && tr->dxx == 1.0f && tr->dyy == 1.0f) { // only translation - fast

        for (h = 0; h < height; h++) {

            ysrc = tr->dyc + h;
            inttr3 = (int)floorf(tr->dyc);
            hlow = (int)floorf(ysrc);
            iy4 = 4 * ((int)((ysrc - hlow) * 256));

            inttr0 = (int)floorf(tr->dxc);
            rowleft = inttr0;
            //            xsrc = tr[0];
            //            rowleft = (int)floor(xsrc);  // low
            ix4 = 4 * ((int)((tr->dxc - inttr0) * 256));

            for (j = 0; j < 4; j++) {
                for (i = 0; i < 4; i++) {
                    intcoef2d[j * 4 + i] = ((intcoef[j + iy4] * intcoef[i + ix4])) / 2048; // 16 coeff. for bicubic 2D, scaled by 2048
                }
            }

            if (hlow < 0 && mtop)
                hlow = -hlow; // mirror borders
            if (hlow >= height && mbottom)
                hlow = height + height - hlow - 2;

            w0 = hlow * src_pitch;

            if ((hlow >= 1) && (hlow < height - 2)) { // middle lines

                for (row = 0; row < row_size; row++) {

                    rowleft = inttr0 + row;

                    //                    xsrc = tr[0]+row;

                    //  x,y point is in square: (rowleft,hlow) to (rowleft+1,hlow+1)

                    if ((rowleft >= 1) && (rowleft < row_size - 2)) {
                        w = w0 + rowleft;

                        pixel = (intcoef2d[0] * srcp[w - src_pitch - 1] + intcoef2d[1] * srcp[w - src_pitch] + intcoef2d[2] * srcp[w - src_pitch + 1] + intcoef2d[3] * srcp[w - src_pitch + 2] +
                                 intcoef2d[4] * srcp[w - 1] + intcoef2d[5] * srcp[w] + intcoef2d[6] * srcp[w + 1] + intcoef2d[7] * srcp[w + 2] +
                                 intcoef2d[8] * srcp[w + src_pitch - 1] + intcoef2d[9] * srcp[w + src_pitch] + intcoef2d[10] * srcp[w + src_pitch + 1] + intcoef2d[11] * srcp[w + src_pitch + 2] +
                                 intcoef2d[12] * srcp[w + src_pitch * 2 - 1] + intcoef2d[13] * srcp[w + src_pitch * 2] + intcoef2d[14] * srcp[w + src_pitch * 2 + 1] + intcoef2d[15] * srcp[w + src_pitch * 2 + 2] + 1024) >>
                                11; // i.e. /2048

                        dstp[row] = VSMAX(VSMIN(pixel, 255), 0);

                    } else if (rowleft < 0 && mleft) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, -rowleft);
                            smoothed = 0;
                            for (i = -rowleft - blurlen + 1; i <= -rowleft; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                        }
                    } else if (rowleft >= row_size && mright) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, rowleft - row_size + 1);
                            smoothed = 0;
                            for (i = row_size + row_size - rowleft - 2; i < row_size + row_size - rowleft - 2 + blurlen; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2]; // not very precise - may be bicubic?
                        }
                    } else if (rowleft == 0 || rowleft == row_size - 1 || rowleft == row_size - 2) { // edges
                        dstp[row] = srcp[w0 + rowleft];
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }

                } // end for
            } else if (hlow == 0 || hlow == height - 2) { // near edge (top-1, bottom-1) lines
                for (row = 0; row < row_size; row++) {
                    rowleft = inttr0 + row;
                    sx = tr->dxc - inttr0;
                    sy = tr->dyc - inttr3;
                    if ((rowleft >= 0) && (rowleft < row_size - 1)) { // bug fixed for right edge in v.1.1.1
                        w = w0 + rowleft;
                        dstp[row] = (int)((1.0 - sy) * ((1.0 - sx) * srcp[w] + sx * srcp[w + 1]) +
                                          sy * ((1.0 - sx) * srcp[w + src_pitch] + sx * srcp[w + src_pitch + 1])); // bilinear
                    } else if (rowleft == row_size - 1) { // added in v.1.1.1
                        dstp[row] = srcp[rowleft + w0];
                    } else if (rowleft < 0 && mleft) {
                        dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                    } else if (rowleft >= row_size && mright) {
                        dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2];
                    } else if (border >= 0) { // left or right
                        dstp[row] = border;
                    }
                }
            } else if (hlow == height - 1) { // bottom line
                for (row = 0; row < row_size; row++) {
                    rowleft = inttr0 + row;
                    if (rowleft >= 0 && rowleft < row_size) {
                        dstp[row] = srcp[w0 + rowleft];
                    } else if (rowleft < 0 && mleft) {
                        dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                    } else if (rowleft >= row_size && mright) {
                        dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2];
                    } else if (border >= 0) { // left or right
                        dstp[row] = border;
                    }
                }
            } else if (border >= 0) { // out lines
                for (row = 0; row < row_size; row++) {
                    dstp[row] = border;
                }
            }

            dstp += dst_pitch; // next line
        }
    }
    //-----------------------------------------------------------------------------
    else if (tr->dxy == 0.0f && tr->dyx == 0.0f) { // no rotation, only zoom and translation  - fast

        // prepare positions   (they are not dependent from h) for fast processing
        for (row = 0; row < row_size; row++) {
            xsrc = tr->dxc + tr->dxx * row;
            rowleftwork[row] = (int)floorf(xsrc);
            rowleft = rowleftwork[row];
            ix4work[row] = 4 * ((int)((xsrc - rowleft) * 256));
        }


        for (h = 0; h < height; h++) {

            ysrc = tr->dyc + tr->dyy * h;

            hlow = (int)floorf(ysrc);
            iy4 = 4 * ((int)((ysrc - hlow) * 256));

            sy = ysrc - hlow;

            if (hlow < 0 && mtop)
                hlow = -hlow; // mirror borders
            if (hlow >= height && mbottom)
                hlow = height + height - hlow - 2;

            w0 = hlow * src_pitch;
            if ((hlow >= 1) && (hlow < height - 2)) { // incide

                for (row = 0; row < row_size; row++) {

                    //                    xsrc = tr[0]+tr[1]*row;
                    //                    rowleft = floor(xsrc);
                    rowleft = rowleftwork[row]; //(int)(xsrc);

                    //  x,y point is in square: (rowleft,hlow) to (rowleft+1,hlow+1)

                    if ((rowleft >= 1) && (rowleft < row_size - 2)) {

                        //                        ix4 = 4*((int)((xsrc-rowleft)*256));
                        ix4 = ix4work[row];
                        w = w0 + rowleft;


                        srcp -= src_pitch; // prev line
                        ts[0] = (intcoef[ix4] * srcp[w - 1] + intcoef[ix4 + 1] * srcp[w] + intcoef[ix4 + 2] * srcp[w + 1] + intcoef[ix4 + 3] * srcp[w + 2]);
                        srcp += src_pitch; // next line
                        ts[1] = (intcoef[ix4] * srcp[w - 1] + intcoef[ix4 + 1] * srcp[w] + intcoef[ix4 + 2] * srcp[w + 1] + intcoef[ix4 + 3] * srcp[w + 2]);
                        srcp += src_pitch; // next line
                        ts[2] = (intcoef[ix4] * srcp[w - 1] + intcoef[ix4 + 1] * srcp[w] + intcoef[ix4 + 2] * srcp[w + 1] + intcoef[ix4 + 3] * srcp[w + 2]);
                        srcp += src_pitch; // next line
                        ts[3] = (intcoef[ix4] * srcp[w - 1] + intcoef[ix4 + 1] * srcp[w] + intcoef[ix4 + 2] * srcp[w + 1] + intcoef[ix4 + 3] * srcp[w + 2]);

                        srcp -= (src_pitch << 1); // restore pointer, changed to shift in v 1.1.1

                        pixel = (intcoef[iy4] * ts[0] + intcoef[iy4 + 1] * ts[1] + intcoef[iy4 + 2] * ts[2] + intcoef[iy4 + 3] * ts[3]) >> 22;

                        dstp[row] = VSMAX(VSMIN(pixel, 255), 0);
                    } else if (rowleft < 0 && mleft) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, -rowleft);
                            smoothed = 0;
                            for (i = -rowleft - blurlen + 1; i <= -rowleft; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else {
                            dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                        }
                    } else if (rowleft >= row_size && mright) {
                        if (blurmax > 0) {
                            blurlen = VSMIN(blurmax, rowleft - row_size + 1);
                            smoothed = 0;
                            for (i = row_size + row_size - rowleft - 2; i < row_size + row_size - rowleft - 2 + blurlen; i++)
                                smoothed += srcp[w0 + i];
                            dstp[row] = smoothed / blurlen;
                        } else { // no blur
                            dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2]; // not very precise - may be bicubic?
                        }
                    } else if (rowleft == 0 || rowleft == row_size - 1 || rowleft == row_size - 2) { // edges
                        dstp[row] = srcp[w0 + rowleft];
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }
                } // end for
            } else if (hlow == 0 || hlow == height - 2) { // near edge (top-1, bottom-1) lines
                for (row = 0; row < row_size; row++) {
                    rowleft = rowleftwork[row];
                    if ((rowleft >= 0) && (rowleft < row_size - 1)) { // bug fixed for right bound in v.1.10.0
                        xsrc = tr->dxc + tr->dxx * row;
                        sx = xsrc - rowleft;
                        w = w0 + rowleft;
                        pixel = (int)((1.0 - sy) * ((1.0 - sx) * srcp[w] + sx * srcp[w + 1]) +
                                      sy * ((1.0 - sx) * srcp[w + src_pitch] + sx * srcp[w + src_pitch + 1])); // bilinear
                        dstp[row] = VSMAX(VSMIN(pixel, 255), 0);
                    } else if (rowleft == row_size - 1) { // added in v.1.1.1
                        dstp[row] = srcp[rowleft + w0];
                    } else if (rowleft < 0 && mleft) {
                        dstp[row] = srcp[w0 - rowleft]; // not very precise - may be bicubic?
                    } else if (rowleft >= row_size && mright) {
                        dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2];
                    } else if (border >= 0) { // left or right
                        dstp[row] = border;
                    }
                }
            } else if (hlow == height - 1) { // bottom line
                for (row = 0; row < row_size; row++) {
                    rowleft = rowleftwork[row];
                    if (rowleft >= 0 && rowleft < row_size) {
                        dstp[row] = (srcp[w0 + rowleft] + srcp[w0 + rowleft - src_pitch]) / 2; // for some smoothing
                    } else if (rowleft < 0 && mleft) {
                        dstp[row] = srcp[w0 - rowleft];
                    } else if (rowleft >= row_size && mright) {
                        dstp[row] = srcp[w0 + row_size + row_size - rowleft - 2];
                    } else if (border >= 0) { // left or right
                        dstp[row] = border;
                    }
                }
            } else if (border >= 0) { // out lines
                for (row = 0; row < row_size; row++) {
                    // bug fixed here in v. 0.9.1 (access violation - bad w0)
                    /*                    rowleft = rowleftwork[row];
                    if (rowleft >=0 && rowleft < row_size) {
                        dstp[row] = srcp[w0+rowleft];
                    }
                    else if ( rowleft < 0 && mleft) {
                        dstp[row] = srcp[w0-rowleft];       // not very precise - may be bicubic?
                    }
                    else if ( rowleft >= row_size && mright) {
                        dstp[row] = srcp[w0+row_size + row_size - rowleft -2];
                    }
                    else  if (border >=0 ){ // left or right
*/
                    dstp[row] = border;
                    //                    }
                }
            }

            dstp += dst_pitch; // next line
        }
    }
    //-----------------------------------------------------------------------------
    else { // rotation, zoom and translation - slow

        for (h = 0; h < height; h++) {

            for (row = 0; row < row_size; row++) {

                xsrc = tr->dxc + tr->dxx * row + tr->dxy * h;
                ysrc = tr->dyc + tr->dyx * row + tr->dyy * h;
                rowleft = (int)(xsrc); // use simply fast (int), not floor(), since followed check >1
                if (xsrc < rowleft) {
                    rowleft -= 1;
                }

                hlow = (int)(ysrc); // use simply fast  (int), not floor(), since followed check >1
                if (ysrc < hlow) {
                    hlow -= 1;
                }

                //  x,y point is in square: (rowleft,hlow) to (rowleft+1,hlow+1)

                if ((rowleft >= 1) && (rowleft < row_size - 2) && (hlow >= 1) && (hlow < height - 2)) {

                    ix4 = 4 * ((int)((xsrc - rowleft) * 256));

                    w0 = rowleft + hlow * src_pitch;

                    srcp -= src_pitch; // prev line
                    ts[0] = (intcoef[ix4] * srcp[w0 - 1] + intcoef[ix4 + 1] * srcp[w0] + intcoef[ix4 + 2] * srcp[w0 + 1] + intcoef[ix4 + 3] * srcp[w0 + 2]);
                    srcp += src_pitch; // next line
                    ts[1] = (intcoef[ix4] * srcp[w0 - 1] + intcoef[ix4 + 1] * srcp[w0] + intcoef[ix4 + 2] * srcp[w0 + 1] + intcoef[ix4 + 3] * srcp[w0 + 2]);
                    srcp += src_pitch; // next line
                    ts[2] = (intcoef[ix4] * srcp[w0 - 1] + intcoef[ix4 + 1] * srcp[w0] + intcoef[ix4 + 2] * srcp[w0 + 1] + intcoef[ix4 + 3] * srcp[w0 + 2]);
                    srcp += src_pitch; // next line
                    ts[3] = (intcoef[ix4] * srcp[w0 - 1] + intcoef[ix4 + 1] * srcp[w0] + intcoef[ix4 + 2] * srcp[w0 + 1] + intcoef[ix4 + 3] * srcp[w0 + 2]);

                    srcp -= (src_pitch << 1); // restore pointer, changed to shift in v.1.1.1


                    iy4 = ((int)((ysrc - hlow) * 256)) << 2; //changed to shift in v.1.1.1

                    pixel = (intcoef[iy4] * ts[0] + intcoef[iy4 + 1] * ts[1] + intcoef[iy4 + 2] * ts[2] + intcoef[iy4 + 3] * ts[3]) >> 22;
                    dstp[row] = VSMAX(VSMIN(pixel, 255), 0);
                } else {
                    if (hlow < 0 && mtop)
                        hlow = -hlow; // mirror borders
                    if (hlow >= height && mbottom)
                        hlow = height + height - hlow - 2;
                    if (rowleft < 0 && mleft)
                        rowleft = -rowleft;
                    if (rowleft >= row_size && mright)
                        rowleft = row_size + row_size - rowleft - 2;
                    // check mirrowed
                    if ((rowleft >= 0) && (rowleft < row_size) && (hlow >= 0) && (hlow < height)) {
                        dstp[row] = srcp[hlow * src_pitch + rowleft];
                    } else if (border >= 0) { // if shifted point is out of frame, fill using border value
                        dstp[row] = border;
                    }
                }
            } // end for row

            dstp += dst_pitch; // next line
        } //end for h
    } // end if rotation
}


static const VSFrameRef *VS_CC depanCompensateGetFrame(int ndest, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    const DepanCompensateData *d = (const DepanCompensateData *)*instanceData;

    if (activationReason == arInitial) {
        int nsrc = ndest - d->intoffset;

        if (d->intoffset == 0 || nsrc < 0 || nsrc > d->vi->numFrames - 1) {
            vsapi->requestFrameFilter(ndest, d->clip, frameCtx);
            return NULL;
        }

        int start = VSMIN(nsrc, ndest);
        int end = VSMAX(nsrc, ndest);

        vsapi->requestFrameFilter(start, d->clip, frameCtx);

        for (int n = start + 1; n <= end; n++)
            vsapi->requestFrameFilter(n, d->data, frameCtx);

        vsapi->requestFrameFilter(end, d->clip, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        int nsrc = ndest - d->intoffset;

        if (d->intoffset == 0 || nsrc < 0 || nsrc > d->vi->numFrames - 1) //  NULL transform, return source
            return vsapi->getFrameFilter(ndest, d->clip, frameCtx);

        int forward = d->intoffset > 0;

        float fractoffset = d->offset;
        fractoffset += forward ? 1 : -1;
        fractoffset -= d->intoffset;

        int start = VSMIN(nsrc, ndest);
        int end = VSMAX(nsrc, ndest);

        int nfields = d->fields ? 2 : 1;

        transform trsum;
        setNull(&trsum);

        for (int n = start + 1; n <= end; n++) {
            const VSFrameRef *dataframe = vsapi->getFrameFilter(n, d->data, frameCtx);
            const VSMap *data_props = vsapi->getFramePropsRO(dataframe);

            int err[4];
            float motionx = (float)vsapi->propGetFloat(data_props, prop_Depan_dx, 0, &err[0]);
            float motiony = (float)vsapi->propGetFloat(data_props, prop_Depan_dy, 0, &err[1]);
            float motionzoom = (float)vsapi->propGetFloat(data_props, prop_Depan_zoom, 0, &err[2]);
            float motionrot = (float)vsapi->propGetFloat(data_props, prop_Depan_rot, 0, &err[3]);

            vsapi->freeFrame(dataframe);

            if (err[0] || err[1] || err[2] || err[3]) {
                vsapi->setFilterError("DepanCompensate: required frame properties not found in data clip. Did data clip really come from DepanAnalyse or DepanEstimate?", frameCtx);
                return NULL;
            }

            if (motionx == MOTIONBAD) {
                setNull(&trsum);
                break;
            }

            transform tr;

            motion2transform(motionx, motiony, motionrot, motionzoom, d->pixaspect / nfields, d->xcenter, d->ycenter, forward, fractoffset, &tr);
            sumtransform(&trsum, &tr, &trsum);
        }

        if (d->fields && d->matchfields) {
            const VSFrameRef *temp = vsapi->getFrameFilter(ndest, d->clip, frameCtx);
            const VSMap *temp_props = vsapi->getFramePropsRO(temp);
            int err;
            int top_field = !!vsapi->propGetInt(temp_props, "_Field", 0, &err);
            vsapi->freeFrame(temp);

            if (err && !d->tff_exists) {
                vsapi->setFilterError("DepanCompensate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                return NULL;
            }

            if (d->tff_exists)
                top_field = d->tff ^ (ndest % 2);

            // reverse 1 line motion correction if matchfields mode
            trsum.dyc += top_field ? -0.5f : 0.5f;
        }


        const VSFrameRef *src = vsapi->getFrameFilter(nsrc, d->clip, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

        int *work2width4356 = (int *)malloc((2 * d->vi->width + 4356) * sizeof(int));

        int border[3] = { 0, 128, 128 };
        int blur[3] = { d->blur, d->blur / 2, d->blur / 2 };
        transform tr[3];

        tr[0] = tr[1] = trsum;
        tr[1].dxc /= 2;
        tr[1].dyc /= 2;
        tr[2] = tr[1];

        for (int plane = 0; plane < d->vi->format->numPlanes; plane++) {
            const uint8_t *srcp = vsapi->getReadPtr(src, plane);
            int src_width = vsapi->getFrameWidth(src, plane);
            int src_height = vsapi->getFrameHeight(src, plane);
            int src_pitch = vsapi->getStride(src, plane);

            uint8_t *dstp = vsapi->getWritePtr(dst, plane);
            int dst_pitch = vsapi->getStride(dst, plane);

            d->compensate_plane(dstp, dst_pitch, srcp, src_pitch, src_width, src_height, &tr[plane], d->mirror, border[plane], work2width4356, blur[plane]);
        }

        free(work2width4356);

        vsapi->freeFrame(src);

        if (d->info) {
            float dxsum = 0.0f, dysum = 0.0f, rotsum = 0.0f, zoomsum = 1.0f;
            transform2motion(&trsum, forward, d->xcenter, d->ycenter, d->pixaspect / nfields, &dxsum, &dysum, &rotsum, &zoomsum);

#define INFO_SIZE 128
            char info[INFO_SIZE + 1] = { 0 };

            snprintf(info, INFO_SIZE, "offset=%.2f, %d to %d, dx=%.2f, dy=%.2f, rot=%.3f zoom=%.5f", d->offset, ndest - d->intoffset, ndest, dxsum, dysum, rotsum, zoomsum);
#undef INFO_SIZE

            VSMap *dst_props = vsapi->getFramePropsRW(dst);
            vsapi->propSetData(dst_props, prop_DepanCompensate_info, info, -1, paReplace);
        }

        return dst;
    }

    return NULL;
}


static void VS_CC depanCompensateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    DepanCompensateData *d = (DepanCompensateData *)instanceData;

    vsapi->freeNode(d->clip);
    vsapi->freeNode(d->data);

    free(d);
}


static void VS_CC depanCompensateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    DepanCompensateData d;
    memset(&d, 0, sizeof(d));

    int err;

    d.offset = (float)vsapi->propGetFloat(in, "offset", 0, &err);

    d.subpixel = int64ToIntS(vsapi->propGetInt(in, "subpixel", 0, &err));
    if (err)
        d.subpixel = 2;

    d.pixaspect = (float)vsapi->propGetFloat(in, "pixaspect", 0, &err);
    if (err)
        d.pixaspect = 1.0f;

    d.matchfields = !!vsapi->propGetInt(in, "matchfields", 0, &err);
    if (err)
        d.matchfields = 1;

    d.mirror = int64ToIntS(vsapi->propGetInt(in, "mirror", 0, &err));

    d.blur = int64ToIntS(vsapi->propGetInt(in, "blur", 0, &err));

    d.info = !!vsapi->propGetInt(in, "info", 0, &err);

    d.fields = !!vsapi->propGetInt(in, "fields", 0, &err);

    d.tff = !!vsapi->propGetInt(in, "tff", 0, &d.tff_exists);
    d.tff_exists = !d.tff_exists;


    if (d.offset < -10.0f || d.offset > 10.0f) {
        vsapi->setError(out, "DepanCompensate: offset must be between -10.0 and 10.0 (inclusive).");
        return;
    }

    if (d.subpixel < 0 || d.subpixel > 2) {
        vsapi->setError(out, "DepanCompensate: subpixel must be between 0 and 2 (inclusive).");
        return;
    }

    if (d.pixaspect <= 0.0f) {
        vsapi->setError(out, "DepanCompensate: pixaspect must be greater than 0.");
        return;
    }

    if (d.mirror < 0 || d.mirror > 15) {
        vsapi->setError(out, "DepanCompensate: mirror must be between 0 and 15 (inclusive).");
        return;
    }

    if (d.blur < 0) {
        vsapi->setError(out, "DepanCompensate: blur must not be negative.");
        return;
    }


    d.clip = vsapi->propGetNode(in, "clip", 0, NULL);
    d.vi = vsapi->getVideoInfo(d.clip);

    if (!isConstantFormat(d.vi) || d.vi->format->id != pfYUV420P8) {
        vsapi->setError(out, "DepanCompensate: clip must have constant format and dimensions, and it must be YUV420P8.");
        vsapi->freeNode(d.clip);
        return;
    }

    d.data = vsapi->propGetNode(in, "data", 0, NULL);

    if (d.vi->numFrames > vsapi->getVideoInfo(d.data)->numFrames) {
        vsapi->setError(out, "DepanCompensate: data must have at least as many frames as clip.");
        vsapi->freeNode(d.data);
        vsapi->freeNode(d.clip);
        return;
    }


    if (d.offset > 0.0f)
        d.intoffset = (int)ceilf(d.offset);
    else
        d.intoffset = (int)floorf(d.offset);

    d.xcenter = d.vi->width / 2.0f; // center of frame
    d.ycenter = d.vi->height / 2.0f;


    CompensateFunction compensate_functions[3] = {
        compensate_plane_nearest,
        compensate_plane_bilinear,
        compensate_plane_bicubic
    };
    d.compensate_plane = compensate_functions[d.subpixel];


    DepanCompensateData *data = (DepanCompensateData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "DepanCompensate", depanCompensateInit, depanCompensateGetFrame, depanCompensateFree, fmParallel, 0, data, core);

    if (vsapi->getError(out)) {
        depanCompensateFree(data, core, vsapi);
        return;
    }

    if (d.info) {
        if (!invokeFrameProps(prop_DepanCompensate_info, out, core, vsapi)) {
            vsapi->setError(out, std::string("DepanCompensate: failed to invoke text.FrameProps: ").append(vsapi->getError(out)).c_str());
            depanCompensateFree(data, core, vsapi);
            return;
        }
    }
}


typedef struct DepanStabiliseData {
    VSNodeRef *clip;
    VSNodeRef *data;
    float cutoff;
    float damping;
    float initzoom;
    int addzoom;
    int prev;
    int next;
    int mirror;
    int blur;
    float dxmax;
    float dymax;
    float zoommax;
    float rotmax;
    int subpixel;
    float pixaspect;
    int fitlast;
    float tzoom;
    int info;
    int method;
    int fields;

    const VSVideoInfo *vi;

    int nfields;

    float *motionx;
    float *motiony;
    float *motionrot;
    float *motionzoom;

    int *work2width4356;

    transform *trcumul;
    transform *trsmoothed;
    transform nonlinfactor;

    float *azoom;
    float *azoomsmoothed;

    float fps;        // frame per second
    float mass;       // mass
    float pdamp;      // damping parameter
    float kstiff;     // stiffness
    float freqnative; // native frequency
    int radius;       // stabilization radius

    float *wint; // average window
    int wintsize;
    float *winrz; // rize zoom window
    float *winfz; // fall zoom window
    int winrzsize;
    int winfzsize;

    float xcenter; // center of frame
    float ycenter;

    CompensateFunction compensate_plane;
} DepanStabiliseData;


static void VS_CC depanStabiliseInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    DepanStabiliseData *d = (DepanStabiliseData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static void Inertial(DepanStabiliseData *d, int nbase, int ndest, transform *ptrdif) {
    transform *trsmoothed = d->trsmoothed;
    transform *trcumul = d->trcumul;
    const transform nonlinfactor = d->nonlinfactor;
    const float damping = d->damping;
    const float fps = d->fps;
    const float freqnative = d->freqnative;
    const float pixaspect = d->pixaspect;
    const int nfields = d->nfields;
    const int addzoom = d->addzoom;
    float *azoom = d->azoom;
    float *azoomsmoothed = d->azoomsmoothed;
    const float initzoom = d->initzoom;
    const float xcenter = d->xcenter;
    const float ycenter = d->ycenter;
    const int width = d->vi->width;
    const int height = d->vi->height;
    const float cutoff = d->cutoff;
    const float tzoom = d->tzoom;

    int n;
    transform trinv, trcur, trtemp;

    // set null as smoothed for base - v1.12
    // set null as smoothed for base+1 - v1.12
    setNull(&trsmoothed[nbase]);
    setNull(&trsmoothed[nbase + 1]);

    float cdamp = 12.56f * damping / fps;
    float cquad = 39.44f / (fps * fps);

    // recurrent calculation of smoothed cumulative transforms from base+2 to ndest frames
    for (n = nbase + 2; n <= ndest; n++) {

        // dxc predictor:
        trsmoothed[n].dxc = 2 * trsmoothed[n - 1].dxc - trsmoothed[n - 2].dxc -
                            cdamp * freqnative * (trsmoothed[n - 1].dxc - trsmoothed[n - 2].dxc - trcumul[n - 1].dxc + trcumul[n - 2].dxc) *
                                (1 + 0.5f * nonlinfactor.dxc / freqnative * fabsf(trsmoothed[n - 1].dxc - trsmoothed[n - 2].dxc - trcumul[n - 1].dxc + trcumul[n - 2].dxc)) -
                            cquad * freqnative * freqnative * (trsmoothed[n - 1].dxc - trcumul[n - 1].dxc) *
                                (1 + nonlinfactor.dxc * fabsf(trsmoothed[n - 1].dxc - trcumul[n - 1].dxc)); // predictor
        // corrector, one iteration must be enough:
        trsmoothed[n].dxc = 2 * trsmoothed[n - 1].dxc - trsmoothed[n - 2].dxc -
                            cdamp * freqnative * 0.5f * (trsmoothed[n].dxc - trsmoothed[n - 2].dxc - trcumul[n].dxc + trcumul[n - 2].dxc) *
                                (1 + 0.5f * nonlinfactor.dxc / freqnative * 0.5f * fabsf(trsmoothed[n].dxc - trsmoothed[n - 2].dxc - trcumul[n].dxc + trcumul[n - 2].dxc)) -
                            cquad * freqnative * freqnative * (trsmoothed[n - 1].dxc - trcumul[n - 1].dxc) *
                                (1 + nonlinfactor.dxc * fabsf(trsmoothed[n - 1].dxc - trcumul[n - 1].dxc));

        // very light (2 frames interval) stabilization of zoom
        trsmoothed[n].dxx = 0.5f * (trcumul[n].dxx + trsmoothed[n - 1].dxx);

        // dxy predictor:
        // double cutoff frequency for rotation
        trsmoothed[n].dxy = 2 * trsmoothed[n - 1].dxy - trsmoothed[n - 2].dxy -
                            cdamp * 2 * freqnative * (trsmoothed[n - 1].dxy - trsmoothed[n - 2].dxy - trcumul[n - 1].dxy + trcumul[n - 2].dxy) *
                                (1 + 0.5f * nonlinfactor.dxy / freqnative * fabsf(trsmoothed[n - 1].dxy - trsmoothed[n - 2].dxy - trcumul[n - 1].dxy + trcumul[n - 2].dxy)) -
                            cquad * 4 * freqnative * freqnative * (trsmoothed[n - 1].dxy - trcumul[n - 1].dxy) *
                                (1 + nonlinfactor.dxy * fabsf(trsmoothed[n - 1].dxy - trcumul[n - 1].dxy)); // predictor
        // corrector, one iteration must be enough:
        trsmoothed[n].dxy = 2 * trsmoothed[n - 1].dxy - trsmoothed[n - 2].dxy -
                            cdamp * 2 * freqnative * 0.5f * (trsmoothed[n].dxy - trsmoothed[n - 2].dxy - trcumul[n].dxy + trcumul[n - 2].dxy) *
                                (1 + 0.5f * nonlinfactor.dxy / freqnative * 0.5f * fabsf(trsmoothed[n].dxy - trsmoothed[n - 2].dxy - trcumul[n].dxy + trcumul[n - 2].dxy)) -
                            cquad * 4 * freqnative * freqnative * (trsmoothed[n - 1].dxy - trcumul[n - 1].dxy) *
                                (1 + nonlinfactor.dxy * fabsf(trsmoothed[n - 1].dxy - trcumul[n - 1].dxy)); // corrector, one iteration must be enough

        // dyx predictor:
        trsmoothed[n].dyx = -trsmoothed[n].dxy * (pixaspect / nfields) * (pixaspect / nfields); // must be consistent

        // dyc predictor:
        trsmoothed[n].dyc = 2 * trsmoothed[n - 1].dyc - trsmoothed[n - 2].dyc -
                            cdamp * freqnative * (trsmoothed[n - 1].dyc - trsmoothed[n - 2].dyc - trcumul[n - 1].dyc + trcumul[n - 2].dyc) *
                                (1 + 0.5f * nonlinfactor.dyc / freqnative * fabsf(trsmoothed[n - 1].dyc - trsmoothed[n - 2].dyc - trcumul[n - 1].dyc + trcumul[n - 2].dyc)) -
                            cquad * freqnative * freqnative * (trsmoothed[n - 1].dyc - trcumul[n - 1].dyc) *
                                (1 + nonlinfactor.dyc * fabsf(trsmoothed[n - 1].dyc - trcumul[n - 1].dyc)); // predictor
        // corrector, one iteration must be enough:
        trsmoothed[n].dyc = 2 * trsmoothed[n - 1].dyc - trsmoothed[n - 2].dyc -
                            cdamp * freqnative * 0.5f * (trsmoothed[n].dyc - trsmoothed[n - 2].dyc - trcumul[n].dyc + trcumul[n - 2].dyc) *
                                (1 + 0.5f * nonlinfactor.dyc / freqnative * 0.5f * fabsf(trsmoothed[n].dyc - trsmoothed[n - 2].dyc - trcumul[n].dyc + trcumul[n - 2].dyc)) -
                            cquad * freqnative * freqnative * (trsmoothed[n - 1].dyc - trcumul[n - 1].dyc) *
                                (1 + nonlinfactor.dyc * fabsf(trsmoothed[n - 1].dyc - trcumul[n - 1].dyc)); // corrector, one iteration must be enough


        // dyy
        trsmoothed[n].dyy = trsmoothed[n].dxx; //must be equal to dxx
    }


    if (addzoom) { // calculate and add adaptive zoom factor to fill borders (for all frames from base to ndest)

        azoom[nbase] = initzoom;
        azoom[nbase + 1] = initzoom;
        azoomsmoothed[nbase] = initzoom;
        azoomsmoothed[nbase + 1] = initzoom;
        for (n = nbase + 2; n <= ndest; n++) {
            // get inverse transform
            inversetransform(&trcumul[n], &trinv);
            // calculate difference between smoothed and original non-smoothed cumulative transform
            sumtransform(&trinv, &trsmoothed[n], &trcur);
            // find adaptive zoom factor
            //                transform2motion (trcur, 1, xcenter, ycenter, pixaspect/nfields, &dxdif, &dydif, &rotdif, &zoomdif);
            azoom[n] = initzoom;
            float azoomtest = 1 + (trcur.dxc + trcur.dxy * ycenter) / xcenter; // xleft
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;
            azoomtest = 1 - (trcur.dxc + trcur.dxx * width + trcur.dxy * ycenter - width) / xcenter; //xright
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;
            azoomtest = 1 + (trcur.dyc + trcur.dyx * xcenter) / ycenter; // ytop
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;
            azoomtest = 1 - (trcur.dyc + trcur.dyx * xcenter + trcur.dyy * height - height) / ycenter; //ybottom
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;

            // limit zoom to max - added in v.1.4.0
            //                if (fabsf(azoom[n]-1) > fabsf(zoommax)-1)
            //                    azoom[n] =     2 - fabsf(zoommax) ;


            // smooth adaptive zoom
            // zoom time factor
            float zf = 1 / (cutoff * tzoom);
            // predictor
            azoomsmoothed[n] = 2 * azoomsmoothed[n - 1] - azoomsmoothed[n - 2] -
                               zf * cdamp * freqnative * (azoomsmoothed[n - 1] - azoomsmoothed[n - 2] - azoom[n - 1] + azoom[n - 2])
                               //                    *( 1 + 0.5f*nonlinfactor.dxx/freqnative*fabsf(azoomsmoothed[n-1] - azoomsmoothed[n-2] - azoom[n-1] + azoom[n-2])) // disabled in v.1.4.0 for more smooth
                               - zf * zf * cquad * freqnative * freqnative * (azoomsmoothed[n - 1] - azoom[n - 1])
                //                    *( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
                ;
            // corrector, one iteration must be enough:
            azoomsmoothed[n] = 2 * azoomsmoothed[n - 1] - azoomsmoothed[n - 2] -
                               zf * cdamp * freqnative * 0.5f * (azoomsmoothed[n] - azoomsmoothed[n - 2] - azoom[n] + azoom[n - 2])
                               //                    *( 1 + 0.5f*nonlinfactor.dxx/freqnative*0.5f*fabsf(azoomsmoothed[n] - azoomsmoothed[n-2] - azoom[n] + azoom[n-2]) )
                               - zf * zf * cquad * freqnative * freqnative * (azoomsmoothed[n - 1] - azoom[n - 1])
                //                    *( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
                ;
            zf = zf * 0.7f;                              // slower zoom decreasing
            if (azoomsmoothed[n] > azoomsmoothed[n - 1]) // added in v.1.4.0 for slower zoom decreasing
            {
                // predictor
                azoomsmoothed[n] = 2 * azoomsmoothed[n - 1] - azoomsmoothed[n - 2] -
                                   zf * cdamp * freqnative * (azoomsmoothed[n - 1] - azoomsmoothed[n - 2] - azoom[n - 1] + azoom[n - 2])
                                   //                    *( 1 + 0.5f*nonlinfactor.dxx/freqnative*fabsf(azoomsmoothed[n-1] - azoomsmoothed[n-2] - azoom[n-1] + azoom[n-2]))
                                   - zf * zf * cquad * freqnative * freqnative * (azoomsmoothed[n - 1] - azoom[n - 1])
                    //                    *( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
                    ;
                // corrector, one iteration must be enough:
                azoomsmoothed[n] = 2 * azoomsmoothed[n - 1] - azoomsmoothed[n - 2] -
                                   zf * cdamp * freqnative * 0.5f * (azoomsmoothed[n] - azoomsmoothed[n - 2] - azoom[n] + azoom[n - 2])
                                   //                    *( 1 + 0.5f*nonlinfactor.dxx/freqnative*0.5f*fabsf(azoomsmoothed[n] - azoomsmoothed[n-2] - azoom[n] + azoom[n-2]) )
                                   - zf * zf * cquad * freqnative * freqnative * (azoomsmoothed[n - 1] - azoom[n - 1])
                    //                    *( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
                    ;
            }
            //            azoomsmoothed[n] = azoomcumul[n]; // debug - no azoom smoothing
            if (azoomsmoothed[n] > 1)
                azoomsmoothed[n] = 1; // not decrease image size
            // make zoom transform
            motion2transform(0, 0, 0, azoomsmoothed[n], pixaspect / nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.5.0
            // get non-adaptive image zoom from transform
            //                transform2motion (trsmoothed[n], 1, xcenter, ycenter, pixaspect/nfields, &dxdif, &dydif, &rotdif, &zoomdif); // disabled in v.1.5.0
            // modify transform with adaptive zoom added
            //                motion2transform (dxdif, dydif, rotdif, zoomdif*azoomsmoothed[n], pixaspect/nfields,  xcenter,  ycenter, 1, 1.0, &trsmoothed[n]); // replaced in v.1.5.0 by:
            sumtransform(&trsmoothed[n], &trtemp, &trsmoothed[n]); // added v.1.5.0
        }
    } else {
        motion2transform(0, 0, 0, initzoom, pixaspect / nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.7
        sumtransform(&trsmoothed[ndest], &trtemp, &trsmoothed[ndest]);                                 // added v.1.7
    }

    // calculate difference between smoothed and original non-smoothed cumulative tranform
    // it will be used as stabilization values

    inversetransform(&trcumul[ndest], &trinv);
    sumtransform(&trinv, &trsmoothed[ndest], ptrdif);
}


static void Average(DepanStabiliseData *d, int nbase, int ndest, int nmax, transform *ptrdif) {
    transform *trsmoothed = d->trsmoothed;
    transform *trcumul = d->trcumul;
    const float * const wint = d->wint;
    const float pixaspect = d->pixaspect;
    const int nfields = d->nfields;
    const int addzoom = d->addzoom;
    float *azoom = d->azoom;
    float *azoomsmoothed = d->azoomsmoothed;
    const float initzoom = d->initzoom;
    const float xcenter = d->xcenter;
    const float ycenter = d->ycenter;
    const int width = d->vi->width;
    const int height = d->vi->height;
    const int winfzsize = d->winfzsize;
    const int winrzsize = d->winrzsize;
    const float * const winfz = d->winfz;
    const float * const winrz = d->winrz;


    int n;
    transform trinv, trcur, trtemp;

    float norm = 0;
    trsmoothed[ndest].dxc = 0;
    trsmoothed[ndest].dyc = 0;
    trsmoothed[ndest].dxy = 0;
    for (n = nbase; n < ndest; n++) {
        trsmoothed[ndest].dxc += trcumul[n].dxc * wint[ndest - n];
        trsmoothed[ndest].dyc += trcumul[n].dyc * wint[ndest - n];
        trsmoothed[ndest].dxy += trcumul[n].dxy * wint[ndest - n];
        norm += wint[ndest - n];
    }
    for (n = ndest; n <= nmax; n++) {
        trsmoothed[ndest].dxc += trcumul[n].dxc * wint[n - ndest];
        trsmoothed[ndest].dyc += trcumul[n].dyc * wint[n - ndest];
        trsmoothed[ndest].dxy += trcumul[n].dxy * wint[n - ndest];
        norm += wint[n - ndest];
    }
    trsmoothed[ndest].dxc /= norm;
    trsmoothed[ndest].dyc /= norm;
    trsmoothed[ndest].dxy /= norm;
    trsmoothed[ndest].dyx = -trsmoothed[ndest].dxy * (pixaspect / nfields) * (pixaspect / nfields); // must be consistent
    norm = 0;
    trsmoothed[ndest].dxx = 0;
    for (n = VSMAX(nbase, ndest - 1); n < ndest; n++) { // very short interval
        trsmoothed[ndest].dxx += trcumul[n].dxx * wint[ndest - n];
        norm += wint[ndest - n];
    }
    for (n = ndest; n <= VSMIN(nmax, ndest + 1); n++) {
        trsmoothed[ndest].dxx += trcumul[n].dxx * wint[n - ndest];
        norm += wint[n - ndest];
    }
    trsmoothed[ndest].dxx /= norm;
    trsmoothed[ndest].dyy = trsmoothed[ndest].dxx;

    //            motion2transform (0, 0, 0, initzoom, pixaspect/nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.7
    //            sumtransform (trsmoothed[ndest],trtemp,  &trsmoothed[ndest]); // added v.1.7

    if (addzoom) { // calculate and add adaptive zoom factor to fill borders (for all frames from base to ndest)

        int nbasez = VSMAX(nbase, ndest - winfzsize);
        int nmaxz = VSMIN(nmax, ndest + winrzsize);
        // symmetrical
        //               nmaxz = ndest + min(nmaxz-ndest, ndest-nbasez);
        //               nbasez = ndest - min(nmaxz-ndest, ndest-nbasez);

        azoom[nbasez] = initzoom;
        for (n = nbasez + 1; n <= nmaxz; n++) {
            // get inverse transform
            inversetransform(&trcumul[n], &trinv);
            // calculate difference between smoothed and original non-smoothed cumulative transform
            //                    sumtransform(trinv, trsmoothed[n], &trcur);
            sumtransform(&trinv, &trcumul[n], &trcur);
            // find adaptive zoom factor
            azoom[n] = initzoom;
            float azoomtest = 1 + (trcur.dxc + trcur.dxy * ycenter) / xcenter; // xleft
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;
            azoomtest = 1 - (trcur.dxc + trcur.dxx * width + trcur.dxy * ycenter - width) / xcenter; //xright
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;
            azoomtest = 1 + (trcur.dyc + trcur.dyx * xcenter) / ycenter; // ytop
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;
            azoomtest = 1 - (trcur.dyc + trcur.dyx * xcenter + trcur.dyy * height - height) / ycenter; //ybottom
            if (azoomtest < azoom[n])
                azoom[n] = azoomtest;
            //                    azoom[n] = initzoom;
        }

        // smooth adaptive zoom
        // zoom time factor
        //                    zf = 1/(cutoff*tzoom);

        norm = 0;
        azoomsmoothed[ndest] = 0.0;
        for (n = nbasez; n < ndest; n++) {
            azoomsmoothed[ndest] += azoom[n] * winfz[ndest - n]; // fall
            norm += winfz[ndest - n];
        }
        for (n = ndest; n <= nmaxz; n++) {
            azoomsmoothed[ndest] += azoom[n] * winrz[n - ndest]; // rize
            norm += winrz[n - ndest];
        }
        azoomsmoothed[ndest] /= norm;
        //                    zf = zf*0.7f; // slower zoom decreasing
        //                    if (azoomsmoothed[n] > azoomsmoothed[n-1]) // added in v.1.4.0 for slower zoom decreasing
        //                    {
        //                    }

        //azoomsmoothed[ndest] = azoom[ndest]; // debug - no azoom smoothing

        if (azoomsmoothed[ndest] > 1)
            azoomsmoothed[ndest] = 1; // not decrease image size
        // make zoom transform
        motion2transform(0, 0, 0, azoomsmoothed[ndest], pixaspect / nfields, xcenter, ycenter, 1, 1.0, &trtemp);
        sumtransform(&trsmoothed[ndest], &trtemp, &trsmoothed[ndest]);

        //            }
    } else // no addzoom
    {
        motion2transform(0, 0, 0, initzoom, pixaspect / nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.7
        sumtransform(&trsmoothed[ndest], &trtemp, &trsmoothed[ndest]);                                 // added v.1.7
    }
    // calculate difference between smoothed and original non-smoothed cumulative tranform
    // it will be used as stabilization values

    inversetransform(&trcumul[ndest], &trinv);
    sumtransform(&trinv, &trsmoothed[ndest], ptrdif);
}


static void InertialLimit(DepanStabiliseData *d, float *dxdif, float *dydif, float *zoomdif, float *rotdif, int ndest, int *nbase) {
    const float initzoom = d->initzoom;
    const float dxmax = d->dxmax;
    const float dymax = d->dymax;
    const float zoommax = d->zoommax;
    const float rotmax = d->rotmax;

    // limit max motion corrections
    if (!(isfinite(*dxdif))) // check added in v.1.1.3
    {                       // infinite or NAN
        *dxdif = 0;
        *dydif = 0;
        *zoomdif = initzoom;
        *rotdif = 0;
        *nbase = ndest;
    } else if (fabsf(*dxdif) > fabsf(dxmax)) {
        if (dxmax >= 0) {
            *dxdif = *dxdif >= 0 ? sqrt(*dxdif * dxmax) : -sqrt(-*dxdif * dxmax); // soft limit v.1.8.2
        } else {
            *dxdif = 0;
            *dydif = 0;
            *zoomdif = initzoom;
            *rotdif = 0;
            *nbase = ndest;
        }
    }

    if (!(isfinite(*dydif))) { // infinite or NAN
        *dxdif = 0;
        *dydif = 0;
        *zoomdif = initzoom;
        *rotdif = 0;
        *nbase = ndest;
    } else if (fabsf(*dydif) > fabsf(dymax)) {
        if (dymax >= 0) {
            *dydif = *dydif >= 0 ? sqrt(*dydif * dymax) : -sqrt(-*dydif * dymax); // soft limit v.1.8.2
        } else {
            *dxdif = 0;
            *dydif = 0;
            *zoomdif = initzoom;
            *rotdif = 0;
            *nbase = ndest;
        }
    }

    if (!(isfinite(*zoomdif))) { // infinite or NAN
        *dxdif = 0;
        *dydif = 0;
        *zoomdif = initzoom;
        *rotdif = 0;
        *nbase = ndest;
    } else if (fabsf(*zoomdif - 1) > fabsf(zoommax) - 1) {
        if (zoommax >= 0) {
            *zoomdif = *zoomdif >= 1 ? 1 + sqrt(fabsf(*zoomdif - 1) * fabsf(zoommax - 1)) : 1 - sqrt(fabsf(*zoomdif - 1) * fabsf(zoommax - 1)); // soft limit v.1.8.2
        } else {
            *dxdif = 0;
            *dydif = 0;
            *zoomdif = initzoom;
            *rotdif = 0;
            *nbase = ndest;
        }
    }

    if (!(isfinite(*rotdif))) { // infinite or NAN
        *dxdif = 0;
        *dydif = 0;
        *zoomdif = initzoom;
        *rotdif = 0;
        *nbase = ndest;
    } else if (fabsf(*rotdif) > fabsf(rotmax)) {
        if (rotmax >= 0) {
            *rotdif = *rotdif >= 0 ? sqrt(*rotdif * rotmax) : -sqrt(-*rotdif * rotmax); // soft limit v.1.8.2
        } else {
            *dxdif = 0;
            *dydif = 0;
            *zoomdif = initzoom;
            *rotdif = 0;
            *nbase = ndest;
        }
    }
}


static int getDepanProps(float *motionx, float *motiony, float *motionrot, float *motionzoom, const VSFrameRef *frame, VSFrameContext *frameCtx, const VSAPI *vsapi) {
    const VSMap *frame_props = vsapi->getFramePropsRO(frame);

    int err[4];

    float x = (float)vsapi->propGetFloat(frame_props, prop_Depan_dx, 0, &err[0]);
    float y = (float)vsapi->propGetFloat(frame_props, prop_Depan_dy, 0, &err[1]);
    float rot = (float)vsapi->propGetFloat(frame_props, prop_Depan_rot, 0, &err[2]);
    float zoom = (float)vsapi->propGetFloat(frame_props, prop_Depan_zoom, 0, &err[3]);

    if (err[0] || err[1] || err[2] || err[3]) {
        vsapi->setFilterError("DepanStabilise: required frame properties not found in data clip.", frameCtx);
        return 0;
    }

    *motionx = x;
    *motiony = y;
    *motionrot = rot;
    *motionzoom = zoom;

    return 1;
}


static void compensateFrame(const VSFrameRef *src, VSFrameRef *dst, DepanStabiliseData *d, int notfilled, const transform *trdif, int *work2width4356, const VSAPI *vsapi) {
    int border[3] = { -1, -1, -1 };
    int blur[3] = { d->blur, d->blur / 2, d->blur / 2 };
    transform tr[3];

    if (notfilled) {
        border[0] = 0;
        border[1] = border[2] = 128;
    }

    tr[0] = tr[1] = *trdif;
    tr[1].dxc /= 2;
    tr[1].dyc /= 2;
    tr[2] = tr[1];

    for (int plane = 0; plane < d->vi->format->numPlanes; plane++) {
        const uint8_t *srcp = vsapi->getReadPtr(src, plane);
        int src_width = vsapi->getFrameWidth(src, plane);
        int src_height = vsapi->getFrameHeight(src, plane);
        int src_pitch = vsapi->getStride(src, plane);

        uint8_t *dstp = vsapi->getWritePtr(dst, plane);
        int dst_pitch = vsapi->getStride(dst, plane);

        // move src frame plane by vector to partially motion compensated position
        d->compensate_plane(dstp, dst_pitch, srcp, src_pitch, src_width, src_height, &tr[plane], d->mirror * notfilled, border[plane], work2width4356, blur[plane]);
    }
}


static void fillBorderPrev(VSFrameRef *dst, DepanStabiliseData *d, int nbase, int ndest, const transform *trdif, int *work2width4356, int *notfilled, VSFrameContext *frameCtx, const VSAPI *vsapi) {
    float dxt1, dyt1, rott1, zoomt1;

    int nprev = ndest - d->prev; // get prev frame number
    if (nprev < nbase)
        nprev = nbase; //  prev distance not exceed base

    int nprevbest = nprev;
    float dabsmin = 10000;

    transform tr[3];
    tr[0] = *trdif; // luma transform

    transform trcur;

    for (int n = ndest - 1; n >= nprev; n--) { // summary inverse transform
        motion2transform(d->motionx[n + 1], d->motiony[n + 1], d->motionrot[n + 1], d->motionzoom[n + 1], d->pixaspect / d->nfields, d->xcenter, d->ycenter, 1, 1.0f, &trcur);
        nprevbest = n;
        sumtransform(&tr[0], &trcur, &tr[0]);
        transform2motion(&tr[0], 1, d->xcenter, d->ycenter, d->pixaspect / d->nfields, &dxt1, &dyt1, &rott1, &zoomt1);
        if ((fabs(dxt1) + fabs(dyt1) + ndest - n) < dabsmin) { // most centered and nearest
            dabsmin = fabs(dxt1) + fabs(dyt1) + ndest - n;
            nprevbest = n;
        }
    }

    tr[1].dxc /= 2;
    tr[1].dyc /= 2;
    tr[2] = tr[1];

    // get original previous source frame
    const VSFrameRef *src = vsapi->getFrameFilter(nprevbest, d->clip, frameCtx);

    int border[3] = { 0, 128, 128 };
    int blur[3] = { d->blur, d->blur / 2, d->blur / 2 };

    for (int plane = 0; plane < d->vi->format->numPlanes; plane++) {
        const uint8_t *srcp = vsapi->getReadPtr(src, plane);
        int src_width = vsapi->getFrameWidth(src, plane);
        int src_height = vsapi->getFrameHeight(src, plane);
        int src_pitch = vsapi->getStride(src, plane);

        uint8_t *dstp = vsapi->getWritePtr(dst, plane);
        int dst_pitch = vsapi->getStride(dst, plane);

        compensate_plane_nearest(dstp, dst_pitch, srcp, src_pitch, src_width, src_height, &tr[plane], d->mirror, border[plane], work2width4356, blur[plane]);
    }

    *notfilled = 0; // mark as FILLED

    vsapi->freeFrame(src);
}


std::mutex g_depanstabilise_mutex;


static int fillBorderNext(VSFrameRef *dst, DepanStabiliseData *d, int ndest, const transform *trdif, int *work2width4356, int *notfilled, VSFrameContext *frameCtx, const VSAPI *vsapi) {
    float dxt1, dyt1, rott1, zoomt1;

    int nnext = ndest + d->next;
    if (nnext >= d->vi->numFrames)
        nnext = d->vi->numFrames - 1;
    int nnextbest = nnext;
    float dabsmin = 1000;

    transform tr[3];
    tr[0] = *trdif; // luma transform for current frame

    transform trcur, trinv;

    // get motion info about frames in interval from begin source to dest in reverse order
    {
        std::lock_guard<std::mutex> guard(g_depanstabilise_mutex);

        for (int n = ndest + 1; n <= nnext; n++) {
            if (d->motionx[n] == MOTIONUNKNOWN) {
                const VSFrameRef *dataframe = vsapi->getFrameFilter(n, d->data, frameCtx);
                if (!getDepanProps(&d->motionx[n], &d->motiony[n], &d->motionrot[n], &d->motionzoom[n], dataframe, frameCtx, vsapi)) {
                    vsapi->freeFrame(dataframe);
                    return 0;
                }
                vsapi->freeFrame(dataframe);
            }
        }
    }

    for (int n = ndest + 1; n <= nnext; n++) {
        if (d->motionx[n] != MOTIONBAD) { //if good
            motion2transform(d->motionx[n], d->motiony[n], d->motionrot[n], d->motionzoom[n], d->pixaspect / d->nfields, d->xcenter, d->ycenter, 1, 1.0f, &trcur);
            inversetransform(&trcur, &trinv);
            sumtransform(&trinv, &tr[0], &tr[0]);
            transform2motion(&tr[0], 1, d->xcenter, d->ycenter, d->pixaspect / d->nfields, &dxt1, &dyt1, &rott1, &zoomt1);
            if ((fabs(dxt1) + fabs(dyt1) + n - ndest) < dabsmin) { // most centered and nearest
                dabsmin = fabs(dxt1) + fabs(dyt1) + n - ndest;
                nnextbest = n;
            }
        } else { // bad
            nnextbest = n - 1; // limit fill frame to last good
            break;
        }
    }

    tr[1].dxc /= 2;
    tr[1].dyc /= 2;
    tr[2] = tr[1];

    // get original previous source frame
    const VSFrameRef *src = vsapi->getFrameFilter(nnextbest, d->clip, frameCtx);

    int border[3] = { -1, -1, -1 };
    int blur[3] = { d->blur, d->blur / 2, d->blur / 2 };

    if (*notfilled) {
        border[0] = 0;
        border[1] = border[2] = 128;
    }

    for (int plane = 0; plane < d->vi->format->numPlanes; plane++) {
        const uint8_t *srcp = vsapi->getReadPtr(src, plane);
        int src_width = vsapi->getFrameWidth(src, plane);
        int src_height = vsapi->getFrameHeight(src, plane);
        int src_pitch = vsapi->getStride(src, plane);

        uint8_t *dstp = vsapi->getWritePtr(dst, plane);
        int dst_pitch = vsapi->getStride(dst, plane);

        // move src frame plane by vector to partially motion compensated position
        compensate_plane_nearest(dstp, dst_pitch, srcp, src_pitch, src_width, src_height, &tr[plane], d->mirror * *notfilled, border[plane], work2width4356, blur[plane]);
    }

    *notfilled = 0; // mark as filled

    vsapi->freeFrame(src);

    return 1;
}


// show text info on frame image
static void attachInfo(VSFrameRef *dst, int nbase, int ndest, float dxdif, float dydif, float rotdif, float zoomdif, const VSAPI *vsapi) {
#define INFO_SIZE 128
    char info[INFO_SIZE + 1] = { 0 };

    snprintf(info, INFO_SIZE, "frame=%d %s=%d dx=%.2f dy=%.2f rot=%.3f zoom=%.5f", ndest, nbase == ndest ? "BASE!" : "base ", nbase, dxdif, dydif, rotdif, zoomdif);
#undef INFO_SIZE

    VSMap *dst_props = vsapi->getFramePropsRW(dst);
    vsapi->propSetData(dst_props, prop_DepanStabilise_info, info, -1, paReplace);
}


static const VSFrameRef *VS_CC depanStabiliseGetFrame0(int ndest, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    DepanStabiliseData *d = (DepanStabiliseData *)*instanceData;

    int nbase = (int)(ndest - 10 * d->fps / d->cutoff);
    if (nbase < 0)
        nbase = 0;

    if (activationReason == arInitial) {
        int nprev = VSMAX(nbase, ndest - d->prev);

        std::lock_guard<std::mutex> guard(g_depanstabilise_mutex);

        for (int i = nbase; i <= ndest; i++) {
            if (d->motionx[i] == MOTIONUNKNOWN)
                vsapi->requestFrameFilter(i, d->data, frameCtx);
            if (d->prev && i >= nprev)
                vsapi->requestFrameFilter(i, d->clip, frameCtx);
        }

        vsapi->requestFrameFilter(ndest, d->clip, frameCtx);

        if (d->next) {
            for (int i = ndest + 1; i <= VSMIN(ndest + d->next, d->vi->numFrames - 1); i++) {
                if (d->motionx[i] == MOTIONUNKNOWN)
                    vsapi->requestFrameFilter(i, d->data, frameCtx);
                vsapi->requestFrameFilter(i, d->clip, frameCtx);
            }
        }
    } else if (activationReason == arAllFramesReady) {
        // get motion info about frames in interval from begin source to dest in reverse order
        {
            std::lock_guard<std::mutex> guard(g_depanstabilise_mutex);

            for (int n = ndest; n >= nbase; n--) {
                if (d->motionx[n] == MOTIONUNKNOWN) {
                    const VSFrameRef *dataframe = vsapi->getFrameFilter(n, d->data, frameCtx);
                    if (!getDepanProps(&d->motionx[n], &d->motiony[n], &d->motionrot[n], &d->motionzoom[n], dataframe, frameCtx, vsapi)) {
                        vsapi->freeFrame(dataframe);
                        return NULL;
                    }
                    vsapi->freeFrame(dataframe);
                }

                if (d->motionx[n] == MOTIONBAD) {
                    if (n > nbase)
                        nbase = n;
                    break; // if strictly =0,  than no good
                }
            }
        }


        float dxdif, dydif, zoomdif, rotdif;
        transform trdif;

        if (nbase == ndest) { // we are at new scene start,
            motion2transform(0, 0, 0, d->initzoom, d->pixaspect / d->nfields, d->xcenter, d->ycenter, 1, 1.0f, &trdif);
        } else { // prepare stabilization data by estimation and smoothing of cumulative motion

            // cumulative transform (position) for all sequence from base

            /// maybe trcumul can be allocated per frame
            // base as null
            setNull(&d->trcumul[nbase]);

            // get cumulative transforms from base to ndest
            for (int n = nbase + 1; n <= ndest; n++) {
                transform trcur;

                motion2transform(d->motionx[n], d->motiony[n], d->motionrot[n], d->motionzoom[n], d->pixaspect / d->nfields, d->xcenter, d->ycenter, 1, 1.0f, &trcur);
                sumtransform(&d->trcumul[n - 1], &trcur, &d->trcumul[n]);
            }

            Inertial(d, nbase, ndest, &trdif);
            // summary motion from summary transform
            transform2motion(&trdif, 1, d->xcenter, d->ycenter, d->pixaspect / d->nfields, &dxdif, &dydif, &rotdif, &zoomdif);
            // fit last - decrease motion correction near end of clip - added in v.1.2.0

            if (d->vi->numFrames < d->fitlast + ndest + 1) {
                float endFactor = ((float)(d->vi->numFrames - ndest - 1)) / d->fitlast; // decrease factor
                dxdif *= endFactor;
                dydif *= endFactor;
                rotdif *= endFactor;
                zoomdif = d->initzoom + (zoomdif - d->initzoom) * endFactor;
            }

            InertialLimit(d, &dxdif, &dydif, &zoomdif, &rotdif, ndest, &nbase);

            // summary motion from summary transform after max correction
            motion2transform(dxdif, dydif, rotdif, zoomdif, d->pixaspect / d->nfields, d->xcenter, d->ycenter, 1, 1.0f, &trdif);
        }

        // ---------------------------------------------------------------------------
        const VSFrameRef *src = vsapi->getFrameFilter(ndest, d->clip, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

        //--------------------------------------------------------------------------
        // Ready to make motion stabilization,

        // --------------------------------------------------------------------
        // use some previous frame to fill borders
        int notfilled = 1; // init as not filled (borders by neighbor frames)

        if (d->prev > 0)
            fillBorderPrev(dst, d, nbase, ndest, &trdif, d->work2width4356, &notfilled, frameCtx, vsapi);

        // use next frame to fill borders
        if (d->next > 0) {
            if (!fillBorderNext(dst, d, ndest, &trdif, d->work2width4356, &notfilled, frameCtx, vsapi)) {
                vsapi->freeFrame(dst);
                vsapi->freeFrame(src);
                return NULL;
            }
        }

        compensateFrame(src, dst, d, notfilled, &trdif, d->work2width4356, vsapi);

        vsapi->freeFrame(src);

        if (d->info) {
            transform2motion(&trdif, 1, d->xcenter, d->ycenter, d->pixaspect / d->nfields, &dxdif, &dydif, &rotdif, &zoomdif);

            attachInfo(dst, nbase, ndest, dxdif, dydif, rotdif, zoomdif, vsapi);
        }

        return dst;
    }

    return NULL;
}


static const VSFrameRef *VS_CC depanStabiliseGetFrame1(int ndest, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    DepanStabiliseData *d = (DepanStabiliseData *)*instanceData;

    int nbase = VSMAX(0, ndest - d->radius);
    int nmax = VSMIN(ndest + d->radius, d->vi->numFrames - 1);

    if (activationReason == arInitial) {
        int nprev = VSMAX(nbase, ndest - d->prev);

        for (int i = nbase; i <= ndest; i++) {
            vsapi->requestFrameFilter(i, d->data, frameCtx);
            if (d->prev && i >= nprev)
                vsapi->requestFrameFilter(i, d->clip, frameCtx);
        }

        vsapi->requestFrameFilter(ndest, d->clip, frameCtx);

        int nnext = VSMIN(ndest + d->next, d->vi->numFrames - 1);

        if (nnext < nmax) {
            for (int i = ndest + 1; i <= nnext; i++) {
                vsapi->requestFrameFilter(i, d->data, frameCtx);
                if (d->next)
                    vsapi->requestFrameFilter(i, d->clip, frameCtx);
            }

            for (int i = nnext + 1; i <= nmax; i++)
                vsapi->requestFrameFilter(i, d->data, frameCtx);
        } else {
            for (int i = ndest + 1; i <= nmax; i++) {
                vsapi->requestFrameFilter(i, d->data, frameCtx);
                if (d->next)
                    vsapi->requestFrameFilter(i, d->clip, frameCtx);
            }

            if (d->next) {
                for (int i = nmax + 1; i <= ndest; i++) {
                    vsapi->requestFrameFilter(i, d->data, frameCtx);
                    vsapi->requestFrameFilter(i, d->clip, frameCtx);
                }
            }
        }
    } else if (activationReason == arAllFramesReady) {
        // get motion info about frames in interval from begin source to dest in reverse order
        for (int n = ndest; n >= nbase; n--) {
            const VSFrameRef *dataframe = vsapi->getFrameFilter(n, d->data, frameCtx);
            if (!getDepanProps(&d->motionx[n], &d->motiony[n], &d->motionrot[n], &d->motionzoom[n], dataframe, frameCtx, vsapi)) {
                vsapi->freeFrame(dataframe);
                return NULL;
            }
            vsapi->freeFrame(dataframe);

            if (d->motionx[n] == MOTIONBAD) {
                if (n > nbase)
                    nbase = n;
                break; // if strictly =0,  than no good
            }
        }


        for (int n = ndest + 1; n <= nmax; n++) {
            const VSFrameRef *dataframe = vsapi->getFrameFilter(n, d->data, frameCtx);
            if (!getDepanProps(&d->motionx[n], &d->motiony[n], &d->motionrot[n], &d->motionzoom[n], dataframe, frameCtx, vsapi)) {
                vsapi->freeFrame(dataframe);
                return NULL;
            }
            vsapi->freeFrame(dataframe);

            if (d->motionx[n] == MOTIONBAD) {
                if (n < nmax)
                    nmax = VSMAX(n - 1, ndest);
                break; // if strictly =0,  than no good
            }
        }


        int smaller_distance = VSMIN(nmax - ndest, ndest - nbase);
        nmax = ndest + smaller_distance;
        nbase = ndest - smaller_distance;


        // cumulative transform (position) for all sequence from base

        // base as null
        setNull(&d->trcumul[nbase]);

        float dxdif, dydif, zoomdif, rotdif;
        transform trdif;

        // get cumulative transforms from base to ndest
        for (int n = nbase + 1; n <= nmax; n++) {
            transform trcur;

            motion2transform(d->motionx[n], d->motiony[n], d->motionrot[n], d->motionzoom[n], d->pixaspect / d->nfields, d->xcenter, d->ycenter, 1, 1.0f, &trcur);
            sumtransform(&d->trcumul[n - 1], &trcur, &d->trcumul[n]);
        }

        Average(d, nbase, ndest, nmax, &trdif);
        // summary motion from summary transform
        transform2motion(&trdif, 1, d->xcenter, d->ycenter, d->pixaspect / d->nfields, &dxdif, &dydif, &rotdif, &zoomdif);

        // summary motion from summary transform after max correction
        motion2transform(dxdif, dydif, rotdif, zoomdif, d->pixaspect / d->nfields, d->xcenter, d->ycenter, 1, 1.0f, &trdif);


        // ---------------------------------------------------------------------------
        const VSFrameRef *src = vsapi->getFrameFilter(ndest, d->clip, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, src, core);

        //--------------------------------------------------------------------------
        // Ready to make motion stabilization,

        // --------------------------------------------------------------------
        // use some previous frame to fill borders
        int notfilled = 1; // init as not filled (borders by neighbor frames)

        if (d->prev > 0)
            fillBorderPrev(dst, d, nbase, ndest, &trdif, d->work2width4356, &notfilled, frameCtx, vsapi);

        // use next frame to fill borders
        if (d->next > 0) {
            if (!fillBorderNext(dst, d, ndest, &trdif, d->work2width4356, &notfilled, frameCtx, vsapi)) {
                vsapi->freeFrame(dst);
                vsapi->freeFrame(src);
                return NULL;
            }
        }

        compensateFrame(src, dst, d, notfilled, &trdif, d->work2width4356, vsapi);

        vsapi->freeFrame(src);

        if (d->info) {
            transform2motion(&trdif, 1, d->xcenter, d->ycenter, d->pixaspect / d->nfields, &dxdif, &dydif, &rotdif, &zoomdif);

            attachInfo(dst, nbase, ndest, dxdif, dydif, rotdif, zoomdif, vsapi);
        }

        return dst;
    }

    return NULL;
}


static void VS_CC depanStabiliseFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    DepanStabiliseData *d = (DepanStabiliseData *)instanceData;

    vsapi->freeNode(d->clip);
    vsapi->freeNode(d->data);

    free(d->motionx);
    free(d->motiony);
    free(d->motionzoom);
    free(d->motionrot);
    free(d->work2width4356);

    free(d->trcumul);
    free(d->trsmoothed);
    free(d->azoom);
    free(d->azoomsmoothed);
    free(d->wint);
    free(d->winrz);
    free(d->winfz);

    free(d);
}


static void VS_CC depanStabiliseCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    DepanStabiliseData d;
    memset(&d, 0, sizeof(d));

    int err;

    d.cutoff = (float)vsapi->propGetFloat(in, "cutoff", 0, &err);
    if (err)
        d.cutoff = 1.0f;

    d.damping = (float)vsapi->propGetFloat(in, "damping", 0, &err);
    if (err)
        d.damping = 0.9f;

    d.initzoom = (float)vsapi->propGetFloat(in, "initzoom", 0, &err);
    if (err)
        d.initzoom = 1.0f;

    d.addzoom = !!vsapi->propGetInt(in, "addzoom", 0, &err);

    d.prev = int64ToIntS(vsapi->propGetInt(in, "prev", 0, &err));

    d.next = int64ToIntS(vsapi->propGetInt(in, "next", 0, &err));

    d.mirror = int64ToIntS(vsapi->propGetInt(in, "mirror", 0, &err));

    d.blur = int64ToIntS(vsapi->propGetInt(in, "blur", 0, &err));

    d.dxmax = (float)vsapi->propGetFloat(in, "dxmax", 0, &err);
    if (err)
        d.dxmax = 60.0f;

    d.dymax = (float)vsapi->propGetFloat(in, "dymax", 0, &err);
    if (err)
        d.dymax = 30.0f;

    d.zoommax = (float)vsapi->propGetFloat(in, "zoommax", 0, &err);
    if (err)
        d.zoommax = 1.05f;

    d.rotmax = (float)vsapi->propGetFloat(in, "rotmax", 0, &err);
    if (err)
        d.rotmax = 1.0f;

    d.subpixel = int64ToIntS(vsapi->propGetInt(in, "subpixel", 0, &err));
    if (err)
        d.subpixel = 2;

    d.pixaspect = (float)vsapi->propGetFloat(in, "pixaspect", 0, &err);
    if (err)
        d.pixaspect = 1.0f;

    d.fitlast = int64ToIntS(vsapi->propGetInt(in, "fitlast", 0, &err));

    d.tzoom = (float)vsapi->propGetFloat(in, "tzoom", 0, &err);
    if (err)
        d.tzoom = 3.0f;

    d.info = !!vsapi->propGetInt(in, "info", 0, &err);

    d.method = int64ToIntS(vsapi->propGetInt(in, "method", 0, &err));

    d.fields = !!vsapi->propGetInt(in, "fields", 0, &err);


    // sanity checks
    if (d.cutoff <= 0.0f) {
        vsapi->setError(out, "DepanStabilise: cutoff must be greater than 0.");
        return;
    }

    if (d.prev < 0) {
        vsapi->setError(out, "DepanStabilise: prev must not be negative.");
        return;
    }

    if (d.next < 0) {
        vsapi->setError(out, "DepanStabilise: next must not be negative.");
        return;
    }

    if (d.subpixel < 0 || d.subpixel > 2) {
        vsapi->setError(out, "DepanStabilise: subpixel must be between 0 and 2 (inclusive).");
        return;
    }

    if (d.pixaspect <= 0.0f) {
        vsapi->setError(out, "DepanStabilise: pixaspect must be greater than 0.");
        return;
    }

    if (d.mirror < 0 || d.mirror > 15) {
        vsapi->setError(out, "DepanStabilise: mirror must be between 0 and 15 (inclusive).");
        return;
    }

    if (d.blur < 0) {
        vsapi->setError(out, "DepanStabilise: blur must not be negative.");
        return;
    }

    if (d.method < 0 || d.method > 1) {
        vsapi->setError(out, "DepanStabilise: method must be between 0 and 1 (inclusive).");
        return;
    }


    d.clip = vsapi->propGetNode(in, "clip", 0, NULL);
    d.vi = vsapi->getVideoInfo(d.clip);

    if (!isConstantFormat(d.vi) || d.vi->format->id != pfYUV420P8) { // etc
        vsapi->setError(out, "DepanStabilise: clip must have constant format and dimensions.");
        vsapi->freeNode(d.clip);
        return;
    }

    if (d.vi->fpsNum == 0 || d.vi->fpsDen == 0) {
        vsapi->setError(out, "DepanStabilise: clip must have known frame rate.");
        vsapi->freeNode(d.clip);
        return;
    }

    d.data = vsapi->propGetNode(in, "data", 0, NULL);

    if (d.vi->numFrames > vsapi->getVideoInfo(d.data)->numFrames) {
        vsapi->setError(out, "DepanStabilise: data must have at least as many frames as clip.");
        vsapi->freeNode(d.data);
        vsapi->freeNode(d.clip);
        return;
    }



    float lambda;

    d.zoommax = d.zoommax > 0 ? VSMAX(d.zoommax, d.initzoom) : -VSMAX(-d.zoommax, d.initzoom);

    // correction for fieldbased
    if (d.fields)
        d.nfields = 2;
    else
        d.nfields = 1;


    d.motionx = (float *)malloc(d.vi->numFrames * sizeof(float));
    d.motiony = (float *)malloc(d.vi->numFrames * sizeof(float));
    d.motionrot = (float *)malloc(d.vi->numFrames * sizeof(float));
    d.motionzoom = (float *)malloc(d.vi->numFrames * sizeof(float));

    d.motionx[0] = 0.0f;
    d.motiony[0] = 0.0f;
    d.motionrot[0] = 0.0f;
    d.motionzoom[0] = 1.0f;
    for (int i = 1; i < d.vi->numFrames; i++)
        d.motionx[i] = MOTIONUNKNOWN; // init as unknown for all frames


    d.work2width4356 = (int *)malloc((2 * d.vi->width + 4356) * sizeof(int)); // work


    d.trcumul = (transform *)malloc(d.vi->numFrames * sizeof(transform));
    d.trsmoothed = (transform *)malloc(d.vi->numFrames * sizeof(transform));

    d.azoom = (float *)malloc(d.vi->numFrames * sizeof(float));
    d.azoomsmoothed = (float *)malloc(d.vi->numFrames * sizeof(float));


    // prepare coefficients for inertial motion smoothing filter

    // elastic stiffness of spring
    d.kstiff = 1.0; // value is not important - (not included in result)
    //  relative frequency lambda at half height of response
    lambda = sqrtf(1 + 6 * d.damping * d.damping + sqrtf((1 + 6 * d.damping * d.damping) * (1 + 6 * d.damping * d.damping) + 3));
    // native oscillation frequency
    d.freqnative = d.cutoff / lambda;
    // mass of camera
    d.mass = d.kstiff / ((6.28f * d.freqnative) * (6.28f * d.freqnative));
    // damping parameter
    d.pdamp = 2 * d.damping * d.kstiff / (6.28f * d.freqnative);
    // frames per secomd
    d.fps = (float)d.vi->fpsNum / d.vi->fpsDen;

    // old smoothing filter coefficients from paper
    //        float a1 = (2*mass + pdamp*period)/(mass + pdamp*period + kstiff*period*period);
    //        float a2 = -mass/(mass + pdamp*period + kstiff*period*period);
    //        float b1 = (pdamp*period + kstiff*period*period)/(mass + pdamp*period + kstiff*period*period);
    //        float b2 = -pdamp*period/(mass + pdamp*period + kstiff*period*period);

    /*        s1 = (2*mass*fps*fps - kstiff)/(mass*fps*fps + pdamp*fps/2);
        s2 = (-mass*fps*fps + pdamp*fps/2)/(mass*fps*fps + pdamp*fps/2);
        c0 = pdamp*fps/2/(mass*fps*fps + pdamp*fps/2);
        c1 = kstiff/(mass*fps*fps + pdamp*fps/2);
        c2 = -pdamp*fps/2/(mass*fps*fps + pdamp*fps/2);
        cnl = -kstiff/(mass*fps*fps + pdamp*fps/2); // nonlinear
*/
    // approximate factor values for nonlinear members as half of max
    if (d.dxmax != 0.0f) {
        d.nonlinfactor.dxc = 5 / fabsf(d.dxmax);
    } else {
        d.nonlinfactor.dxc = 0;
    }
    if (fabsf(d.zoommax) != 1.0f) {
        d.nonlinfactor.dxx = 5 / (fabsf(d.zoommax) - 1);
        d.nonlinfactor.dyy = 5 / (fabsf(d.zoommax) - 1);
    } else {
        d.nonlinfactor.dxx = 0;
        d.nonlinfactor.dyy = 0;
    }
    if (d.dymax != 0.0f) {
        d.nonlinfactor.dyc = 5 / fabsf(d.dymax);
    } else {
        d.nonlinfactor.dyc = 0;
    }
    if (d.rotmax != 0.0f) {
        d.nonlinfactor.dxy = 5 / fabsf(d.rotmax);
        d.nonlinfactor.dyx = 5 / fabsf(d.rotmax);
    } else {
        d.nonlinfactor.dxy = 0;
        d.nonlinfactor.dyx = 0;
    }


    d.initzoom = 1 / d.initzoom; // make consistent with internal definition - v1.7

    d.wintsize = (int)(d.fps / (4 * d.cutoff));
    d.radius = d.wintsize;
    d.wint = (float *)malloc((d.wintsize + 1) * sizeof(float));

    float PI = 3.14159265258f;
    for (int i = 0; i < d.wintsize; i++)
        d.wint[i] = cosf(i * 0.5f * PI / d.wintsize);
    d.wint[d.wintsize] = 0;

    d.winrz = (float *)malloc((d.wintsize + 1) * sizeof(float));
    d.winfz = (float *)malloc((d.wintsize + 1) * sizeof(float));
    d.winrzsize = VSMIN(d.wintsize, (int)(d.fps * d.tzoom / 4));
    d.winfzsize = VSMIN(d.wintsize, (int)(d.fps * d.tzoom / 4));
    for (int i = 0; i < d.winrzsize; i++)
        d.winrz[i] = cosf(i * 0.5f * PI / d.winrzsize);
    for (int i = d.winrzsize; i <= d.wintsize; i++)
        d.winrz[i] = 0;
    for (int i = 0; i < d.winfzsize; i++)
        d.winfz[i] = cosf(i * 0.5f * PI / d.winfzsize);
    for (int i = d.winfzsize; i <= d.wintsize; i++)
        d.winfz[i] = 0;

    d.xcenter = d.vi->width / 2.0f; // center of frame
    d.ycenter = d.vi->height / 2.0f;

    CompensateFunction compensate_functions[3] = {
        compensate_plane_nearest,
        compensate_plane_bilinear,
        compensate_plane_bicubic
    };
    d.compensate_plane = compensate_functions[d.subpixel];


    DepanStabiliseData *data = (DepanStabiliseData *)malloc(sizeof(d));
    *data = d;

    VSFilterGetFrame getframe_functions[2] = {
        depanStabiliseGetFrame0,
        depanStabiliseGetFrame1,
    };


    vsapi->createFilter(in, out, "DepanStabilise", depanStabiliseInit, getframe_functions[d.method], depanStabiliseFree, fmParallelRequests, 0, data, core);

    if (vsapi->getError(out)) {
        depanStabiliseFree(data, core, vsapi);
        return;
    }

    if (d.info) {
        if (!invokeFrameProps(prop_DepanStabilise_info, out, core, vsapi)) {
            vsapi->setError(out, std::string("DepanStabilise: failed to invoke text.FrameProps: ").append(vsapi->getError(out)).c_str());
            depanStabiliseFree(data, core, vsapi);
            return;
        }
    }
}


extern "C" void mvdepanRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("DepanAnalyse",
                 "clip:clip;"
                 "vectors:clip;"
                 "mask:clip:opt;"
                 "zoom:int:opt;"
                 "rot:int:opt;"
                 "pixaspect:float:opt;"
                 "error:float:opt;"
                 "info:int:opt;"
                 "wrong:float:opt;"
                 "zerow:float:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 , depanAnalyseCreate, 0, plugin);

    registerFunc("DepanEstimate",
                 "clip:clip;"
                 "trust:float:opt;"
                 "winx:int:opt;"
                 "winy:int:opt;"
                 "wleft:int:opt;"
                 "wtop:int:opt;"
                 "dxmax:int:opt;"
                 "dymax:int:opt;"
                 "zoommax:float:opt;"
                 "stab:float:opt;"
                 "pixaspect:float:opt;"
                 "info:int:opt;"
                 "show:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 , depanEstimateCreate, 0, plugin);

    registerFunc("DepanCompensate",
                 "clip:clip;"
                 "data:clip;"
                 "offset:float:opt;"
                 "subpixel:int:opt;"
                 "pixaspect:float:opt;"
                 "matchfields:int:opt;"
                 "mirror:int:opt;"
                 "blur:int:opt;"
                 "info:int:opt;"
                 "fields:int:opt;"
                 "tff:int:opt;"
                 , depanCompensateCreate, 0, plugin);

    registerFunc("DepanStabilise",
                 "clip:clip;"
                 "data:clip;"
                 "cutoff:float:opt;"
                 "damping:float:opt;"
                 "initzoom:float:opt;"
                 "addzoom:int:opt;"
                 "prev:int:opt;"
                 "next:int:opt;"
                 "mirror:int:opt;"
                 "blur:int:opt;"
                 "dxmax:float:opt;"
                 "dymax:float:opt;"
                 "zoommax:float:opt;"
                 "rotmax:float:opt;"
                 "subpixel:int:opt;"
                 "pixaspect:float:opt;"
                 "fitlast:int:opt;"
                 "tzoom:float:opt;"
                 "info:int:opt;"
                 "method:int:opt;"
                 "fields:int:opt;"
                 , depanStabiliseCreate, 0, plugin);
}
