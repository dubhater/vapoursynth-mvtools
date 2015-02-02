#ifndef MVT_CPU_H
#define MVT_CPU_H

#include <stdint.h>

uint32_t cpu_detect( void );

extern "C" void mvtools_cpu_emms();
extern "C" uint32_t mvtools_cpu_cpuid( uint32_t op, uint32_t *eax, uint32_t *ebx, uint32_t *ecx, uint32_t *edx );
extern "C" void mvtools_cpu_xgetbv( uint32_t op, uint32_t *eax, uint32_t *edx );

#endif
