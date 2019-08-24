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

#include <VapourSynth.h>
#include <VSHelper.h>

#include "MaskFun.h"


typedef struct MVFinestData {
    VSNodeRef *super;
    VSVideoInfo vi;

    int opt;

    int nWidth;
    int nHeight;
    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;
    int nPel;
    int xRatioUV;
    int yRatioUV;
} MVFinestData;


static void VS_CC mvfinestInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    (void)in;
    (void)out;
    (void)core;
    MVFinestData *d = (MVFinestData *)*instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvfinestGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    (void)frameData;

    MVFinestData *d = (MVFinestData *)*instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->super, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *ref = vsapi->getFrameFilter(n, d->super, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, ref, core);

        uint8_t *pDst[3];
        const uint8_t *pRef[3];
        int nDstPitches[3], nRefPitches[3];

        for (int i = 0; i < d->vi.format->numPlanes; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            pRef[i] = vsapi->getReadPtr(ref, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
            nRefPitches[i] = vsapi->getStride(ref, i);
        }

        int bitsPerSample = d->vi.format->bitsPerSample;
        int bytesPerSample = d->vi.format->bytesPerSample;

        if (d->nPel == 1) { // simply copy top lines
            for (int i = 0; i < d->vi.format->numPlanes; i++)
                vs_bitblt(pDst[i], nDstPitches[i], pRef[i], nRefPitches[i], d->vi.width * bytesPerSample, d->vi.height);
        } else {
            MVGroupOfFrames pRefGOF = { 0 };
            mvgofInit(&pRefGOF, d->nSuperLevels, d->nWidth, d->nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->opt, d->xRatioUV, d->yRatioUV, bitsPerSample);

            mvgofUpdate(&pRefGOF, (uint8_t **)pRef, nRefPitches);

            MVPlane **pPlanes = pRefGOF.frames[0]->planes;


            // merge refined planes to big single plane
            for (int i = 0; i < 3; i++) {
                if (pPlanes[i]) {
                    if (d->nPel == 2) {
                        Merge4PlanesToBig(pDst[i], nDstPitches[i],
                                          mvpGetAbsolutePointer(pPlanes[i], 0, 0),
                                          mvpGetAbsolutePointer(pPlanes[i], 1, 0),
                                          mvpGetAbsolutePointer(pPlanes[i], 0, 1),
                                          mvpGetAbsolutePointer(pPlanes[i], 1, 1),
                                          pPlanes[i]->nPaddedWidth, pPlanes[i]->nPaddedHeight,
                                          pPlanes[i]->nPitch, bitsPerSample);
                    } else if (d->nPel == 4) {
                        Merge16PlanesToBig(pDst[i], nDstPitches[i],
                                           mvpGetAbsolutePointer(pPlanes[i], 0, 0),
                                           mvpGetAbsolutePointer(pPlanes[i], 1, 0),
                                           mvpGetAbsolutePointer(pPlanes[i], 2, 0),
                                           mvpGetAbsolutePointer(pPlanes[i], 3, 0),
                                           mvpGetAbsolutePointer(pPlanes[i], 0, 1),
                                           mvpGetAbsolutePointer(pPlanes[i], 1, 1),
                                           mvpGetAbsolutePointer(pPlanes[i], 2, 1),
                                           mvpGetAbsolutePointer(pPlanes[i], 3, 1),
                                           mvpGetAbsolutePointer(pPlanes[i], 0, 2),
                                           mvpGetAbsolutePointer(pPlanes[i], 1, 2),
                                           mvpGetAbsolutePointer(pPlanes[i], 2, 2),
                                           mvpGetAbsolutePointer(pPlanes[i], 3, 2),
                                           mvpGetAbsolutePointer(pPlanes[i], 0, 3),
                                           mvpGetAbsolutePointer(pPlanes[i], 1, 3),
                                           mvpGetAbsolutePointer(pPlanes[i], 2, 3),
                                           mvpGetAbsolutePointer(pPlanes[i], 3, 3),
                                           pPlanes[i]->nPaddedWidth, pPlanes[i]->nPaddedHeight,
                                           pPlanes[i]->nPitch, bitsPerSample);
                    }
                }
            }

            mvgofDeinit(&pRefGOF);
        }

        vsapi->freeFrame(ref);

        return dst;
    }

    return 0;
}


static void VS_CC mvfinestFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    (void)core;

    MVFinestData *d = (MVFinestData *)instanceData;

    vsapi->freeNode(d->super);
    free(d);
}


static void VS_CC mvfinestCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    (void)userData;

    MVFinestData d;
    MVFinestData *data;

    int err;

    d.opt = !!vsapi->propGetInt(in, "opt", 0, &err);
    if (err)
        d.opt = 2;


    d.super = vsapi->propGetNode(in, "super", 0, 0);
    d.vi = *vsapi->getVideoInfo(d.super);

    if (!isConstantFormat(&d.vi) || d.vi.format->bitsPerSample > 16 || d.vi.format->sampleType != stInteger || d.vi.format->subSamplingW > 1 || d.vi.format->subSamplingH > 1 || (d.vi.format->colorFamily != cmYUV && d.vi.format->colorFamily != cmGray)) {
        vsapi->setError(out, "Finest: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        return;
    }

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Finest: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg + errorLen, ERROR_SIZE - errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->setError(out, errorMsg);
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[6];
    d.nHeight = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    d.nSuperVPad = int64ToIntS(vsapi->propGetInt(props, "Super_vpad", 0, &evil_err[2]));
    d.nSuperPel = int64ToIntS(vsapi->propGetInt(props, "Super_pel", 0, &evil_err[3]));
    d.nSuperModeYUV = int64ToIntS(vsapi->propGetInt(props, "Super_modeyuv", 0, &evil_err[4]));
    d.nSuperLevels = int64ToIntS(vsapi->propGetInt(props, "Super_levels", 0, &evil_err[5]));
    vsapi->freeFrame(evil);

    for (int i = 0; i < 6; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "Finest: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }

    d.nPel = d.nSuperPel;
    int nSuperWidth = d.vi.width;
    d.nWidth = nSuperWidth - 2 * d.nSuperHPad;

    d.xRatioUV = 1 << d.vi.format->subSamplingW;
    d.yRatioUV = 1 << d.vi.format->subSamplingH;

    d.vi.width = (d.nWidth + 2 * d.nSuperHPad) * d.nSuperPel;
    d.vi.height = (d.nHeight + 2 * d.nSuperVPad) * d.nSuperPel;


    data = (MVFinestData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Finest", mvfinestInit, mvfinestGetFrame, mvfinestFree, fmParallel, 0, data, core);
}


void mvfinestRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Finest",
                 "super:clip;"
                 "opt:int:opt;",
                 mvfinestCreate, 0, plugin);
}
