#include <VapourSynth.h>


// Extra indirection to keep the parameter lists with the respective filters.


void mvsuperRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);


VS_EXTERNAL_API(void) VapourSynthPluginInit(VSConfigPlugin configFunc, VSRegisterFunction registerFunc, VSPlugin *plugin) {
    configFunc("com.nodame.mvtools", "mv", "MVTools", VAPOURSYNTH_API_VERSION, 1, plugin);

    mvsuperRegister(registerFunc, plugin);
}
