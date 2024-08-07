
#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "MaskFun.h"
#include "SimpleResize.h"

#include "MVFlowFPSHelper.h"


const VSFrame *VS_CC mvflowfpshelperGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVFlowFPSHelperData *d = (MVFlowFPSHelperData *)instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->vectors, frameCtx);

        FakeGroupOfPlanes fgop;

        fgopInit(&fgop, &d->vectors_data);

        const VSMap *mvprops = vsapi->getFramePropertiesRO(src);
        fgopUpdate(&fgop, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));

        int isUsable = fgopIsUsable(&fgop, d->thscd1, d->thscd2);

        if (isUsable) {
            VSFrame *dst = vsapi->copyFrame(src, core);
            vsapi->freeFrame(src);

            VSMap *props = vsapi->getFramePropertiesRW(dst);

            const int xRatioUV = d->vectors_data.xRatioUV;
            const int yRatioUV = d->vectors_data.yRatioUV;
            const int nBlkX = d->vectors_data.nBlkX;
            const int nBlkY = d->vectors_data.nBlkY;
            const int nHeightP = d->nHeightP;
            const int nHeightPUV = d->nHeightPUV;
            const int VPitchY = d->VPitchY;
            const int VPitchUV = d->VPitchUV;
            const int nBlkXP = d->nBlkXP;
            const int nBlkYP = d->nBlkYP;
            SimpleResize *upsizer = &d->upsizer;
            SimpleResize *upsizerUV = &d->upsizerUV;

            int full_size_y = nHeightP * VPitchY * sizeof(int16_t);
            int small_size = nBlkXP * nBlkYP * sizeof(int16_t);

            int16_t *VXFullY = (int16_t *)malloc(full_size_y);
            int16_t *VYFullY = (int16_t *)malloc(full_size_y);
            int16_t *VXSmallY = (int16_t *)malloc(small_size);
            int16_t *VYSmallY = (int16_t *)malloc(small_size);

            // make  vector vx and vy small masks
            MakeVectorSmallMasks(&fgop, nBlkX, nBlkY, VXSmallY, nBlkXP, VYSmallY, nBlkXP);

            CheckAndPadSmallY(VXSmallY, VYSmallY, nBlkXP, nBlkYP, nBlkX, nBlkY);

            upsizer->simpleResize_int16_t(upsizer, VXFullY, VPitchY, VXSmallY, nBlkXP, 1);
            upsizer->simpleResize_int16_t(upsizer, VYFullY, VPitchY, VYSmallY, nBlkXP, 0);

            vsapi->mapSetData(props, prop_VXFullY, (const char *)VXFullY, full_size_y, dtBinary, maReplace);
            vsapi->mapSetData(props, prop_VYFullY, (const char *)VYFullY, full_size_y, dtBinary, maReplace);

            free(VXFullY);
            free(VYFullY);

            if (d->supervi->format.colorFamily != cfGray) {
                int full_size_uv = nHeightPUV * VPitchUV * sizeof(int16_t);

                int16_t *VXFullUV = (int16_t *)malloc(full_size_uv);
                int16_t *VYFullUV = (int16_t *)malloc(full_size_uv);
                int16_t *VXSmallUV = (int16_t *)malloc(small_size);
                int16_t *VYSmallUV = (int16_t *)malloc(small_size);

                VectorSmallMaskYToHalfUV(VXSmallY, nBlkXP, nBlkYP, VXSmallUV, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallY, nBlkXP, nBlkYP, VYSmallUV, yRatioUV);

                upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUV, VPitchUV, VXSmallUV, nBlkXP, 1);
                upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUV, VPitchUV, VYSmallUV, nBlkXP, 0);

                free(VXSmallUV);
                free(VYSmallUV);

                vsapi->mapSetData(props, prop_VXFullUV, (const char *)VXFullUV, full_size_uv, dtBinary, maReplace);
                vsapi->mapSetData(props, prop_VYFullUV, (const char *)VYFullUV, full_size_uv, dtBinary, maReplace);

                free(VXFullUV);
                free(VYFullUV);
            }

            free(VXSmallY);
            free(VYSmallY);


            fgopDeinit(&fgop);

            return dst;
        } else { // poor estimation
            fgopDeinit(&fgop);

            return src;
        }
    }

    return NULL;
}


void VS_CC mvflowfpshelperFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVFlowFPSHelperData *d = (MVFlowFPSHelperData *)instanceData;

    vsapi->freeNode(d->vectors);

    free(d);
}
