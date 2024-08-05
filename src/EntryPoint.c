#include <VapourSynth4.h>
#include <stdlib.h>

#include "CPU.h"


// Extra indirection to keep the parameter lists with the respective filters.


void mvsuperRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvanalyseRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvdegrainsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvcompensateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvrecalculateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvmaskRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvfinestRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowblurRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowinterRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowfpsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvblockfpsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvscdetectionRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvdepanRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);


uint32_t g_cpuinfo = 0;

VS_EXTERNAL_API(void)
VapourSynthPluginInit2(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    const int packageVersion = atoi(PACKAGE_VERSION);

    vspapi->configPlugin("com.nodame.mvtools", "mv", "MVTools v" PACKAGE_VERSION, VS_MAKE_VERSION(packageVersion, 0), VS_MAKE_VERSION(VAPOURSYNTH_API_MAJOR, VAPOURSYNTH_API_MINOR), 0, plugin);

    mvsuperRegister(plugin, vspapi);
    mvanalyseRegister(plugin, vspapi);
    mvdegrainsRegister(plugin, vspapi);
    mvcompensateRegister(plugin, vspapi);
    mvrecalculateRegister(plugin, vspapi);
    mvmaskRegister(plugin, vspapi);
    mvfinestRegister(plugin, vspapi);
    mvflowRegister(plugin, vspapi);
    mvflowblurRegister(plugin, vspapi);
    mvflowinterRegister(plugin, vspapi);
    mvflowfpsRegister(plugin, vspapi);
    mvblockfpsRegister(plugin, vspapi);
    mvscdetectionRegister(plugin, vspapi);
    mvdepanRegister(plugin, vspapi);

    g_cpuinfo = cpu_detect();
}
