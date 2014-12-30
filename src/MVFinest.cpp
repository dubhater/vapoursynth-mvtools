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

//#include "CopyCode.h"
#include "MaskFun.h"


typedef struct {
    VSNodeRef *super;
    VSVideoInfo vi;

    bool isse;

    int nWidth;
    int nHeight;
    int nSuperHPad;
    int nSuperVPad;
    int nSuperPel;
    int nSuperModeYUV;
    int nSuperLevels;
    int nPel;
    int yRatioUV;
} MVFinestData;


static void VS_CC mvfinestInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVFinestData *d = (MVFinestData *) * instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvfinestGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVFinestData *d = (MVFinestData *) * instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->super, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *ref = vsapi->getFrameFilter(n, d->super, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, ref, core);

        uint8_t *pDst[3];
        const uint8_t *pRef[3];
        int nDstPitches[3], nRefPitches[3];

        for (int i = 0; i < 3; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            pRef[i] = vsapi->getReadPtr(ref, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
            nRefPitches[i] = vsapi->getStride(ref, i);
        }

        if (d->nPel == 1) // simply copy top lines
        {
            for (int i = 0; i < 3; i++)
                vs_bitblt(pDst[i], nDstPitches[i], pRef[i], nRefPitches[i], d->vi.width, d->vi.height);
        }
        else
        {
            MVGroupOfFrames *pRefGOF = new MVGroupOfFrames(d->nSuperLevels, d->nWidth, d->nHeight, d->nSuperPel, d->nSuperHPad, d->nSuperVPad, d->nSuperModeYUV, d->isse, d->yRatioUV);

            pRefGOF->Update(YUVPLANES, (uint8_t*)pRef[0], nRefPitches[0], (uint8_t*)pRef[1], nRefPitches[1], (uint8_t*)pRef[2], nRefPitches[2]);// v2.0

            MVPlane *pPlanes[3];

            pPlanes[0] = pRefGOF->GetFrame(0)->GetPlane(YPLANE);
            pPlanes[1] = pRefGOF->GetFrame(0)->GetPlane(UPLANE);
            pPlanes[2] = pRefGOF->GetFrame(0)->GetPlane(VPLANE);


            if (d->nPel == 2)
            {
                // merge refined planes to big single plane
                Merge4PlanesToBig(pDst[0], nDstPitches[0], pPlanes[0]->GetAbsolutePointer(0,0),
                        pPlanes[0]->GetAbsolutePointer(1,0), pPlanes[0]->GetAbsolutePointer(0,1),
                        pPlanes[0]->GetAbsolutePointer(1,1), pPlanes[0]->GetExtendedWidth(),
                        pPlanes[0]->GetExtendedHeight(), pPlanes[0]->GetPitch());
                if (pPlanes[1]) // 0 if plane not exist
                    Merge4PlanesToBig(pDst[1], nDstPitches[1], pPlanes[1]->GetAbsolutePointer(0,0),
                            pPlanes[1]->GetAbsolutePointer(1,0), pPlanes[1]->GetAbsolutePointer(0,1),
                            pPlanes[1]->GetAbsolutePointer(1,1), pPlanes[1]->GetExtendedWidth(),
                            pPlanes[1]->GetExtendedHeight(), pPlanes[1]->GetPitch());
                if (pPlanes[2])
                    Merge4PlanesToBig(pDst[2], nDstPitches[2], pPlanes[2]->GetAbsolutePointer(0,0),
                            pPlanes[2]->GetAbsolutePointer(1,0), pPlanes[2]->GetAbsolutePointer(0,1),
                            pPlanes[2]->GetAbsolutePointer(1,1), pPlanes[2]->GetExtendedWidth(),
                            pPlanes[2]->GetExtendedHeight(), pPlanes[2]->GetPitch());
            }
            else if (d->nPel == 4)
            {
                // merge refined planes to big single plane
                Merge16PlanesToBig(pDst[0], nDstPitches[0],
                        pPlanes[0]->GetAbsolutePointer(0,0), pPlanes[0]->GetAbsolutePointer(1,0),
                        pPlanes[0]->GetAbsolutePointer(2,0), pPlanes[0]->GetAbsolutePointer(3,0),
                        pPlanes[0]->GetAbsolutePointer(0,1), pPlanes[0]->GetAbsolutePointer(1,1),
                        pPlanes[0]->GetAbsolutePointer(2,1), pPlanes[0]->GetAbsolutePointer(3,1),
                        pPlanes[0]->GetAbsolutePointer(0,2), pPlanes[0]->GetAbsolutePointer(1,2),
                        pPlanes[0]->GetAbsolutePointer(2,2), pPlanes[0]->GetAbsolutePointer(3,2),
                        pPlanes[0]->GetAbsolutePointer(0,3), pPlanes[0]->GetAbsolutePointer(1,3),
                        pPlanes[0]->GetAbsolutePointer(2,3), pPlanes[0]->GetAbsolutePointer(3,3),
                        pPlanes[0]->GetExtendedWidth(), pPlanes[0]->GetExtendedHeight(), pPlanes[0]->GetPitch());
                if (pPlanes[1])
                    Merge16PlanesToBig(pDst[1], nDstPitches[1],
                            pPlanes[1]->GetAbsolutePointer(0,0), pPlanes[1]->GetAbsolutePointer(1,0),
                            pPlanes[1]->GetAbsolutePointer(2,0), pPlanes[1]->GetAbsolutePointer(3,0),
                            pPlanes[1]->GetAbsolutePointer(0,1), pPlanes[1]->GetAbsolutePointer(1,1),
                            pPlanes[1]->GetAbsolutePointer(2,1), pPlanes[1]->GetAbsolutePointer(3,1),
                            pPlanes[1]->GetAbsolutePointer(0,2), pPlanes[1]->GetAbsolutePointer(1,2),
                            pPlanes[1]->GetAbsolutePointer(2,2), pPlanes[1]->GetAbsolutePointer(3,2),
                            pPlanes[1]->GetAbsolutePointer(0,3), pPlanes[1]->GetAbsolutePointer(1,3),
                            pPlanes[1]->GetAbsolutePointer(2,3), pPlanes[1]->GetAbsolutePointer(3,3),
                            pPlanes[1]->GetExtendedWidth(), pPlanes[1]->GetExtendedHeight(), pPlanes[1]->GetPitch());
                if (pPlanes[2])
                    Merge16PlanesToBig(pDst[2], nDstPitches[2],
                            pPlanes[2]->GetAbsolutePointer(0,0), pPlanes[2]->GetAbsolutePointer(1,0),
                            pPlanes[2]->GetAbsolutePointer(2,0), pPlanes[2]->GetAbsolutePointer(3,0),
                            pPlanes[2]->GetAbsolutePointer(0,1), pPlanes[2]->GetAbsolutePointer(1,1),
                            pPlanes[2]->GetAbsolutePointer(2,1), pPlanes[2]->GetAbsolutePointer(3,1),
                            pPlanes[2]->GetAbsolutePointer(0,2), pPlanes[2]->GetAbsolutePointer(1,2),
                            pPlanes[2]->GetAbsolutePointer(2,2), pPlanes[2]->GetAbsolutePointer(3,2),
                            pPlanes[2]->GetAbsolutePointer(0,3), pPlanes[2]->GetAbsolutePointer(1,3),
                            pPlanes[2]->GetAbsolutePointer(2,3), pPlanes[2]->GetAbsolutePointer(3,3),
                            pPlanes[2]->GetExtendedWidth(), pPlanes[2]->GetExtendedHeight(), pPlanes[2]->GetPitch());
            }

            delete pRefGOF;
        }

        vsapi->freeFrame(ref);

        return dst;
    }

    return 0;
}


static void VS_CC mvfinestFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVFinestData *d = (MVFinestData *)instanceData;

    vsapi->freeNode(d->super);
    free(d);
}


static void VS_CC mvfinestCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVFinestData d;
    MVFinestData *data;

    int err;

    d.isse = vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;


    d.super = vsapi->propGetNode(in, "super", 0, 0);
    d.vi = *vsapi->getVideoInfo(d.super);

    int id = d.vi.format->id;
    if (!isConstantFormat(&d.vi) || (id != pfYUV420P8 && id != pfYUV422P8)) {
        vsapi->setError(out, "Finest: input clip must be YUV420P8 or YUV422P8, with constant dimensions.");
        vsapi->freeNode(d.super);
        return;
    }

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("Finest: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[6];
    d.nHeight = vsapi->propGetInt(props, "Super height", 0, &evil_err[0]);
    d.nSuperHPad = vsapi->propGetInt(props, "Super hpad", 0, &evil_err[1]);
    d.nSuperVPad = vsapi->propGetInt(props, "Super vpad", 0, &evil_err[2]);
    d.nSuperPel = vsapi->propGetInt(props, "Super pel", 0, &evil_err[3]);
    d.nSuperModeYUV = vsapi->propGetInt(props, "Super modeyuv", 0, &evil_err[4]);
    d.nSuperLevels = vsapi->propGetInt(props, "Super levels", 0, &evil_err[5]);
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
            "isse:int:opt;"
            , mvfinestCreate, 0, plugin);
}
