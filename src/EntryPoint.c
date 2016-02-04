#include <VapourSynth.h>


// Extra indirection to keep the parameter lists with the respective filters.


void mvsuperRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvanalyseRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvdegrainsRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvcompensateRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvrecalculateRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvmaskRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvfinestRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvflowblurRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvflowinterRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvflowfpsRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvblockfpsRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);
void mvscdetectionRegister(VSRegisterFunction registerFunc, VSPlugin *plugin);


VS_EXTERNAL_API(void) VapourSynthPluginInit(VSConfigPlugin configFunc, VSRegisterFunction registerFunc, VSPlugin *plugin) {
    configFunc("com.nodame.mvtools", "mv", "MVTools", VAPOURSYNTH_API_VERSION, 1, plugin);

    mvsuperRegister(registerFunc, plugin);
    mvanalyseRegister(registerFunc, plugin);
    mvdegrainsRegister(registerFunc, plugin);
    mvcompensateRegister(registerFunc, plugin);
    mvrecalculateRegister(registerFunc, plugin);
    mvmaskRegister(registerFunc, plugin);
    mvfinestRegister(registerFunc, plugin);
    mvflowblurRegister(registerFunc, plugin);
    mvflowinterRegister(registerFunc, plugin);
    mvflowfpsRegister(registerFunc, plugin);
    mvblockfpsRegister(registerFunc, plugin);
    mvscdetectionRegister(registerFunc, plugin);
}
