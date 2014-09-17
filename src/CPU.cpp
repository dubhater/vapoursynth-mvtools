/*****************************************************************************
 * cpu.c: h264 encoder library
 *****************************************************************************
 * Copyright (C) 2003 Laurent Aimar
 * $Id: cpu.c,v 1.1 2004/06/03 19:27:06 fenrir Exp $
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111, USA.
 *****************************************************************************/
// adapted to be used by MVTools by TSchniede

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "MVInterface.h"



extern "C" uint32_t mvtools_cpu_cpuid( uint32_t op, uint32_t *eax, uint32_t *ebx, uint32_t *ecx, uint32_t *edx );


uint32_t cpu_detect( void )
{
	uint32_t cpu = 0;
    uint32_t eax, ebx, ecx, edx;
    uint32_t vendor[4] = {0};
    uint32_t max_extended_cap;
    int cache;


    mvtools_cpu_cpuid( 0, &eax, vendor+0, vendor+2, vendor+1 );
    if( eax == 0 )
        return 0;

    mvtools_cpu_cpuid( 1, &eax, &ebx, &ecx, &edx );
    if( edx&0x00800000 )
        cpu |= CPU_MMX;
    else
        return 0;
    if( edx&0x02000000 )
        cpu |= CPU_MMXEXT|CPU_SSE;
    if( edx&0x04000000 )
        cpu |= CPU_SSE2;
    if( ecx&0x00000001 )
        cpu |= CPU_SSE3;
    if( ecx&0x00000200 )
        cpu |= CPU_SSSE3;
    if( ecx&0x00080000 )
        cpu |= CPU_SSE4;

    if( cpu & CPU_SSSE3 )
        cpu |= CPU_SSE2_IS_FAST;
    if( cpu & CPU_SSE4 )
        cpu |= CPU_PHADD_IS_FAST;

    mvtools_cpu_cpuid( 0x80000000, &eax, &ebx, &ecx, &edx );
    max_extended_cap = eax;

    if( !strcmp((char*)vendor, "AuthenticAMD") && max_extended_cap >= 0x80000001 )
    {
        mvtools_cpu_cpuid( 0x80000001, &eax, &ebx, &ecx, &edx );
        if( edx&0x00400000 )
            cpu |= CPU_MMXEXT;
        if( cpu & CPU_SSE2 )
        {
            if( ecx&0x00000040 ) /* SSE4a */
                cpu |= CPU_SSE2_IS_FAST;
            else
                cpu |= CPU_SSE2_IS_SLOW;
        }
    }

    if( !strcmp((char*)vendor, "GenuineIntel") )
    {
        int family, model;//, stepping;
        mvtools_cpu_cpuid( 1, &eax, &ebx, &ecx, &edx );
        family = ((eax>>8)&0xf) + ((eax>>20)&0xff);
        model  = ((eax>>4)&0xf) + ((eax>>12)&0xf0);
        //stepping = eax&0xf;
        /* 6/9 (pentium-m "banias"), 6/13 (pentium-m "dothan"), and 6/14 (core1 "yonah")
         * theoretically support sse2, but it's significantly slower than mmx for
         * almost all of x264's functions, so let's just pretend they don't. */
        if( family==6 && (model==9 || model==13 || model==14) )
        {
            cpu &= ~(CPU_SSE2|CPU_SSE3);
			assert(!(cpu&(CPU_SSSE3|CPU_SSE4)));
        }
    }

    if( !strcmp((char*)vendor, "GenuineIntel") || !strcmp((char*)vendor, "CyrixInstead") )
    {
        /* cacheline size is specified in 3 places, any of which may be missing */
        mvtools_cpu_cpuid( 1, &eax, &ebx, &ecx, &edx );
        cache = (ebx&0xff00)>>5; // cflush size
        if( !cache && max_extended_cap >= 0x80000006 )
        {
            mvtools_cpu_cpuid( 0x80000006, &eax, &ebx, &ecx, &edx );
            cache = ecx&0xff; // cacheline size
        }
        if( !cache )
        {
            // Cache and TLB Information
            static const unsigned char cache32_ids[] = { 0x0a, 0x0c, 0x41, 0x42, 0x43, 0x44, 0x45, 0x82, 0x83, 0x84, 0x85, 0 };
            static const unsigned char cache64_ids[] = { 0x22, 0x23, 0x25, 0x29, 0x2c, 0x46, 0x47, 0x49, 0x60, 0x66, 0x67, 0x68, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7c, 0x7f, 0x86, 0x87, 0 };
            uint32_t buf[4];
            int max, i=0, j;
            do {
                mvtools_cpu_cpuid( 2, buf+0, buf+1, buf+2, buf+3 );
                max = buf[0]&0xff;
                buf[0] &= ~0xff;
                for(j=0; j<4; j++)
                    if( !(buf[j]>>31) )
                        while( buf[j] )
                        {
							if( strchr( (const char*)cache32_ids, buf[j]&0xff ) )
                                cache = 32;
                            if( strchr( (const char*)cache64_ids, buf[j]&0xff ) )
                                cache = 64;
	                            buf[j] >>= 8;
                        }
            } while( ++i < max );
        }

        if( cache == 32 )
            cpu |= CPU_CACHELINE_32;
        else if( cache == 64 )
            cpu |= CPU_CACHELINE_64;
		}

    return cpu;
}
