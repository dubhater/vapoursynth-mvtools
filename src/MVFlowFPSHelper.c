
#include <VapourSynth.h>
#include <VSHelper.h>

#include "MaskFun.h"
#include "SimpleResize.h"

#include "MVFlowFPSHelper.h"


void VS_CC mvflowfpshelperInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVFlowFPSHelperData *d = (MVFlowFPSHelperData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


const VSFrameRef *VS_CC mvflowfpshelperGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVFlowFPSHelperData *d = (MVFlowFPSHelperData *)*instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->vectors, frameCtx);

        FakeGroupOfPlanes fgop;

        fgopInit(&fgop, &d->vectors_data);

        const int *mvs = (const int *)vsapi->getReadPtr(src, 0);
        fgopUpdate(&fgop, mvs + mvs[0] / sizeof(int));

        int isUsable = fgopIsUsable(&fgop, d->thscd1, d->thscd2);

        if (isUsable) {
            VSFrameRef *dst = vsapi->copyFrame(src, core);
            vsapi->freeFrame(src);

            VSMap *props = vsapi->getFramePropsRW(dst);

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

            int full_size_y = nHeightP * VPitchY;
            int small_size = nBlkXP * nBlkYP;

            uint8_t *VXFullY = (uint8_t *)malloc(full_size_y);
            uint8_t *VYFullY = (uint8_t *)malloc(full_size_y);
            uint8_t *VXSmallY = (uint8_t *)malloc(small_size);
            uint8_t *VYSmallY = (uint8_t *)malloc(small_size);

            // make  vector vx and vy small masks
            // 1. ATTENTION: vectors are assumed SHORT (|vx|, |vy| < 127) !
            // 2. they will be zeroed if not
            // 3. added 128 to all values
            MakeVectorSmallMasks(&fgop, nBlkX, nBlkY, VXSmallY, nBlkXP, VYSmallY, nBlkXP);
            if (nBlkXP > nBlkX) { // fill right
                for (int j = 0; j < nBlkY; j++) {
                    VXSmallY[j * nBlkXP + nBlkX] = VSMIN(VXSmallY[j * nBlkXP + nBlkX - 1], 128);
                    VYSmallY[j * nBlkXP + nBlkX] = VYSmallY[j * nBlkXP + nBlkX - 1];
                }
            }
            if (nBlkYP > nBlkY) { // fill bottom
                for (int i = 0; i < nBlkXP; i++) {
                    VXSmallY[nBlkXP * nBlkY + i] = VXSmallY[nBlkXP * (nBlkY - 1) + i];
                    VYSmallY[nBlkXP * nBlkY + i] = VSMIN(VYSmallY[nBlkXP * (nBlkY - 1) + i], 128);
                }
            }

            simpleResize(upsizer, VXFullY, VPitchY, VXSmallY, nBlkXP);
            simpleResize(upsizer, VYFullY, VPitchY, VYSmallY, nBlkXP);

            vsapi->propSetData(props, prop_VXFullY, (const char *)VXFullY, full_size_y, paReplace);
            vsapi->propSetData(props, prop_VYFullY, (const char *)VYFullY, full_size_y, paReplace);

            free(VXFullY);
            free(VYFullY);

            if (d->supervi->format->colorFamily != cmGray) {
                int full_size_uv = nHeightPUV * VPitchUV;

                uint8_t *VXFullUV = (uint8_t *)malloc(full_size_uv);
                uint8_t *VYFullUV = (uint8_t *)malloc(full_size_uv);
                uint8_t *VXSmallUV = (uint8_t *)malloc(small_size);
                uint8_t *VYSmallUV = (uint8_t *)malloc(small_size);

                VectorSmallMaskYToHalfUV(VXSmallY, nBlkXP, nBlkYP, VXSmallUV, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallY, nBlkXP, nBlkYP, VYSmallUV, yRatioUV);

                simpleResize(upsizerUV, VXFullUV, VPitchUV, VXSmallUV, nBlkXP);
                simpleResize(upsizerUV, VYFullUV, VPitchUV, VYSmallUV, nBlkXP);

                free(VXSmallUV);
                free(VYSmallUV);

                vsapi->propSetData(props, prop_VXFullUV, (const char *)VXFullUV, full_size_uv, paReplace);
                vsapi->propSetData(props, prop_VYFullUV, (const char *)VYFullUV, full_size_uv, paReplace);

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
    MVFlowFPSHelperData *d = (MVFlowFPSHelperData *)instanceData;

    vsapi->freeNode(d->vectors);

    free(d);
}
