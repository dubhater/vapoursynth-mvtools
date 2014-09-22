#ifndef MVT_CPU_H
#define MVT_CPU_H

#include <stdint.h>

uint32_t cpu_detect( void );

extern "C" void mvtools_cpu_emms();

#endif
