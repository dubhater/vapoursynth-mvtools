// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - YUY2
// See legal notice in Copying.txt for more information
//
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

#include <VapourSynth.h>
#include <VSHelper.h>

#include "MVClip.h"
#include "MVFilter.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *vectors;

    int thscd1;
    int thscd2;

    MVFilter *bleh;

    MVClipDicks *mvClip;
} MVSCDetectionData;


static void VS_CC mvscdetectionInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVSCDetectionData *d = (MVSCDetectionData *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


static const VSFrameRef *VS_CC mvscdetectionGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVSCDetectionData *d = (MVSCDetectionData *) * instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrameRef *dst = vsapi->copyFrame(src, core);
        vsapi->freeFrame(src);

        const VSFrameRef *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        MVClipBalls balls(d->mvClip, vsapi);
        balls.Update(mvn);
        vsapi->freeFrame(mvn);

        const char *propNames[2] = { "_SceneChangePrev", "_SceneChangeNext" };
        VSMap *props = vsapi->getFramePropsRW(dst);
        vsapi->propSetInt(props, propNames[(int)d->mvClip->IsBackward()], !balls.IsUsable(), paReplace);

        return dst;
    }

    return 0;
}


static void VS_CC mvscdetectionFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVSCDetectionData *d = (MVSCDetectionData *)instanceData;

    vsapi->freeNode(d->node);
    vsapi->freeNode(d->vectors);
    delete d->mvClip;
    delete d->bleh;
    free(d);
}


static void VS_CC mvscdetectionCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVSCDetectionData d;
    MVSCDetectionData *data;

    int err;

    d.thscd1 = int64ToIntS(vsapi->propGetInt(in, "thscd1", 0, &err));
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;


    d.vectors = vsapi->propGetNode(in, "vectors", 0, NULL);

    try {
        d.mvClip = new MVClipDicks(d.vectors, d.thscd1, d.thscd2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("SCDetection: ").append(e.what()).c_str());
        vsapi->freeNode(d.vectors);
        return;
    }

    try {
        d.bleh = new MVFilter(d.vectors, "SCDetection", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("SCDetection: ").append(e.what()).c_str());
        vsapi->freeNode(d.vectors);
        delete d.mvClip;
        return;
    }


    d.node = vsapi->propGetNode(in, "clip", 0, NULL);
    d.vi = vsapi->getVideoInfo(d.node);


    data = (MVSCDetectionData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "SCDetection", mvscdetectionInit, mvscdetectionGetFrame, mvscdetectionFree, fmParallel, 0, data, core);
}


void mvscdetectionRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("SCDetection",
            "clip:clip;"
            "vectors:clip;"
            "thscd1:int:opt;"
            "thscd2:int:opt;"
            , mvscdetectionCreate, 0, plugin);
}
