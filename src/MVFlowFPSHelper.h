#ifndef MVFLOWFPSHELPER_H
#define MVFLOWFPSHELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <VapourSynth.h>

#include "MVAnalysisData.h"
#include "SimpleResize.h"

typedef struct MVFlowFPSHelperData {
    VSNodeRef *vectors;
    const VSVideoInfo *vi;

    const VSVideoInfo *supervi;

    int thscd1, thscd2;

    MVAnalysisData vectors_data;

    int nHeightP;
    int nHeightPUV;
    int VPitchY;
    int VPitchUV;
    int nBlkXP;
    int nBlkYP;

    SimpleResize upsizer;
    SimpleResize upsizerUV;
} MVFlowFPSHelperData;


static const char prop_VXFullY[] = "VXFullY";
static const char prop_VYFullY[] = "VYFullY";
static const char prop_VXFullUV[] = "VXFullUV";
static const char prop_VYFullUV[] = "VYFullUV";


void VS_CC mvflowfpshelperInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi);
const VSFrameRef *VS_CC mvflowfpshelperGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi);
void VS_CC mvflowfpshelperFree(void *instanceData, VSCore *core, const VSAPI *vsapi);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
