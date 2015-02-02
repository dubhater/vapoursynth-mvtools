#ifndef MVT_CPU_H
#define MVT_CPU_H

#include <stdint.h>


#define X264_CPU_CMOV            0x0000001
#define X264_CPU_MMX             0x0000002
#define X264_CPU_MMX2            0x0000004  /* MMX2 aka MMXEXT aka ISSE */
#define X264_CPU_MMXEXT          X264_CPU_MMX2
#define X264_CPU_SSE             0x0000008
#define X264_CPU_SSE2            0x0000010
#define X264_CPU_SSE3            0x0000020
#define X264_CPU_SSSE3           0x0000040
#define X264_CPU_SSE4            0x0000080  /* SSE4.1 */
#define X264_CPU_SSE42           0x0000100  /* SSE4.2 */
#define X264_CPU_LZCNT           0x0000200  /* Phenom support for "leading zero count" instruction. */
#define X264_CPU_AVX             0x0000400  /* AVX support: requires OS support even if YMM registers aren't used. */
#define X264_CPU_XOP             0x0000800  /* AMD XOP */
#define X264_CPU_FMA4            0x0001000  /* AMD FMA4 */
#define X264_CPU_FMA3            0x0002000  /* FMA3 */
#define X264_CPU_AVX2            0x0004000  /* AVX2 */
#define X264_CPU_BMI1            0x0008000  /* BMI1 */
#define X264_CPU_BMI2            0x0010000  /* BMI2 */
/* x86 modifiers */
#define X264_CPU_CACHELINE_32    0x0020000  /* avoid memory loads that span the border between two cachelines */
#define X264_CPU_CACHELINE_64    0x0040000  /* 32/64 is the size of a cacheline in bytes */
#define X264_CPU_SSE2_IS_SLOW    0x0080000  /* avoid most SSE2 functions on Athlon64 */
#define X264_CPU_SSE2_IS_FAST    0x0100000  /* a few functions are only faster on Core2 and Phenom */
#define X264_CPU_SLOW_SHUFFLE    0x0200000  /* The Conroe has a slow shuffle unit (relative to overall SSE performance) */
#define X264_CPU_STACK_MOD4      0x0400000  /* if stack is only mod4 and not mod16 */
#define X264_CPU_SLOW_CTZ        0x0800000  /* BSR/BSF x86 instructions are really slow on some CPUs */
#define X264_CPU_SLOW_ATOM       0x1000000  /* The Atom is terrible: slow SSE unaligned loads, slow
                                             * SIMD multiplies, slow SIMD variable shifts, slow pshufb,
                                             * cacheline split penalties -- gather everything here that
                                             * isn't shared by other CPUs to avoid making half a dozen
                                             * new SLOW flags. */
#define X264_CPU_SLOW_PSHUFB     0x2000000  /* such as on the Intel Atom */
#define X264_CPU_SLOW_PALIGNR    0x4000000  /* such as on the AMD Bobcat */


uint32_t cpu_detect( void );

extern "C" void mvtools_cpu_emms();
extern "C" uint32_t mvtools_cpu_cpuid( uint32_t op, uint32_t *eax, uint32_t *ebx, uint32_t *ecx, uint32_t *edx );
extern "C" void mvtools_cpu_xgetbv( uint32_t op, uint32_t *eax, uint32_t *edx );

#endif
