// Author: Manao
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

#include "CopyCode.h"

// I use here the copy code from avisynth. I duplicated it, because I have to use it
// in a class which doesn't have to know what "env" is. Anyway, such static functions
// should not have been put into that class in the first place ( imho )

#if 0
void BitBlt(unsigned char* dstp, int dst_pitch, const unsigned char* srcp, int src_pitch, int row_size, int height, bool isse) {
  if ( (!height)|| (!row_size)) return;
  if (isse) {
    if (height == 1 || (src_pitch == dst_pitch && dst_pitch == row_size)) {
      memcpy_amd(dstp, srcp, row_size*height);
    } else {
      asm_BitBlt_ISSE(dstp,dst_pitch,srcp,src_pitch,row_size,height);
    }
    return;
  }
  if (height == 1 || (dst_pitch == src_pitch && src_pitch == row_size)) {
    memcpy(dstp, srcp, row_size*height); // Fizick: fixed bug
  } else {
    for (int y=height; y>0; --y) {
      memcpy(dstp, srcp, row_size);
      dstp += dst_pitch;
      srcp += src_pitch;
    }
  }
}
#endif

void MemZoneSet(unsigned char *ptr, unsigned char value, int width,
				int height, int offsetX, int offsetY, int pitch)
{
	ptr += offsetX + offsetY*pitch;
	for (int y=offsetY; y<height+offsetY; y++)
	{
		memset(ptr, value, width);
		ptr += pitch;
	}
}

#if 0
// Coded by Steady

void asm_BitBlt_ISSE(unsigned char* dstp, int dst_pitch, const unsigned char* srcp, int src_pitch, int row_size, int height) {

  if(row_size==0 || height==0) return; //abort on goofs
  //move backwards for easier looping and to disable hardware prefetch
  const unsigned char* srcStart=srcp+src_pitch*(height-1);
  unsigned char* dstStart=dstp+dst_pitch*(height-1);

  if(row_size < 64) {
    _asm {
      mov   esi,srcStart  //move rows from bottom up
      mov   edi,dstStart
      mov   edx,row_size
      dec   edx
      mov   ebx,height
      align 16
memoptS_rowloop:
      mov   ecx,edx
//      rep movsb
memoptS_byteloop:
      mov   AL,[esi+ecx]
      mov   [edi+ecx],AL
      sub   ecx,1
      jnc   memoptS_byteloop
      sub   esi,src_pitch
      sub   edi,dst_pitch
      dec   ebx
      jne   memoptS_rowloop
    };
    return;
  }//end small version

  else if( (int(dstp) | row_size | src_pitch | dst_pitch) & 7) {//not QW aligned
    //unaligned version makes no assumptions on alignment

    _asm {
//****** initialize
      mov   esi,srcStart  //bottom row
      mov   AL,[esi]
      mov   edi,dstStart
      mov   edx,row_size
      mov   ebx,height

//********** loop starts here ***********

      align 16
memoptU_rowloop:
      mov   ecx,edx     //row_size
      dec   ecx         //offset to last byte in row
      add   ecx,esi     //ecx= ptr last byte in row
      and   ecx,~63     //align to first byte in cache line
memoptU_prefetchloop:
      mov   AX,[ecx]    //tried AL,AX,EAX, AX a tiny bit faster
      sub   ecx,64
      cmp   ecx,esi
      jae   memoptU_prefetchloop

//************ write *************

      movq    mm6,[esi]     //move the first unaligned bytes
      movntq  [edi],mm6
//************************
      mov   eax,edi
      neg   eax
      mov   ecx,eax
      and   eax,63      //eax=bytes from [edi] to start of next 64 byte cache line
      and   ecx,7       //ecx=bytes from [edi] to next QW
      align 16
memoptU_prewrite8loop:        //write out odd QW's so 64 bit write is cache line aligned
      cmp   ecx,eax           //start of cache line ?
      jz    memoptU_pre8done  //if not, write single QW
      movq    mm7,[esi+ecx]
      movntq  [edi+ecx],mm7
      add   ecx,8
      jmp   memoptU_prewrite8loop

      align 16
memoptU_write64loop:
      movntq  [edi+ecx-64],mm0
      movntq  [edi+ecx-56],mm1
      movntq  [edi+ecx-48],mm2
      movntq  [edi+ecx-40],mm3
      movntq  [edi+ecx-32],mm4
      movntq  [edi+ecx-24],mm5
      movntq  [edi+ecx-16],mm6
      movntq  [edi+ecx- 8],mm7
memoptU_pre8done:
      add   ecx,64
      cmp   ecx,edx         //while(offset <= row_size) do {...
      ja    memoptU_done64
      movq    mm0,[esi+ecx-64]
      movq    mm1,[esi+ecx-56]
      movq    mm2,[esi+ecx-48]
      movq    mm3,[esi+ecx-40]
      movq    mm4,[esi+ecx-32]
      movq    mm5,[esi+ecx-24]
      movq    mm6,[esi+ecx-16]
      movq    mm7,[esi+ecx- 8]
      jmp   memoptU_write64loop
memoptU_done64:

      sub     ecx,64    //went to far
      align 16
memoptU_write8loop:
      add     ecx,8           //next QW
      cmp     ecx,edx         //any QW's left in row ?
      ja      memoptU_done8
      movq    mm0,[esi+ecx-8]
      movntq  [edi+ecx-8],mm0
      jmp   memoptU_write8loop
memoptU_done8:

      movq    mm1,[esi+edx-8] //write the last unaligned bytes
      movntq  [edi+edx-8],mm1
      sub   esi,src_pitch
      sub   edi,dst_pitch
      dec   ebx               //row counter (=height at start)
      jne   memoptU_rowloop

      sfence
      emms
    };
    return;
  }//end unaligned version

  else {//QW aligned version (fastest)
  //else dstp and row_size QW aligned - hope for the best from srcp
  //QW aligned version should generally be true when copying full rows
    _asm {
      mov   esi,srcStart  //start of bottom row
      mov   edi,dstStart
      mov   ebx,height
      mov   edx,row_size
      align 16
memoptA_rowloop:
      mov   ecx,edx //row_size
      dec   ecx     //offset to last byte in row

//********forward routine
      add   ecx,esi
      and   ecx,~63   //align prefetch to first byte in cache line(~3-4% faster)
      align 16
memoptA_prefetchloop:
      mov   AX,[ecx]
      sub   ecx,64
      cmp   ecx,esi
      jae   memoptA_prefetchloop

      mov   eax,edi
      xor   ecx,ecx
      neg   eax
      and   eax,63            //eax=bytes from edi to start of cache line
      align 16
memoptA_prewrite8loop:        //write out odd QW's so 64bit write is cache line aligned
      cmp   ecx,eax           //start of cache line ?
      jz    memoptA_pre8done  //if not, write single QW
      movq    mm7,[esi+ecx]
      movntq  [edi+ecx],mm7
      add   ecx,8
      jmp   memoptA_prewrite8loop

      align 16
memoptA_write64loop:
      movntq  [edi+ecx-64],mm0
      movntq  [edi+ecx-56],mm1
      movntq  [edi+ecx-48],mm2
      movntq  [edi+ecx-40],mm3
      movntq  [edi+ecx-32],mm4
      movntq  [edi+ecx-24],mm5
      movntq  [edi+ecx-16],mm6
      movntq  [edi+ecx- 8],mm7
memoptA_pre8done:
      add   ecx,64
      cmp   ecx,edx
      ja    memoptA_done64    //less than 64 bytes left
      movq    mm0,[esi+ecx-64]
      movq    mm1,[esi+ecx-56]
      movq    mm2,[esi+ecx-48]
      movq    mm3,[esi+ecx-40]
      movq    mm4,[esi+ecx-32]
      movq    mm5,[esi+ecx-24]
      movq    mm6,[esi+ecx-16]
      movq    mm7,[esi+ecx- 8]
      jmp   memoptA_write64loop

memoptA_done64:
      sub   ecx,64

      align 16
memoptA_write8loop:           //less than 8 QW's left
      add   ecx,8
      cmp   ecx,edx
      ja    memoptA_done8     //no QW's left
      movq    mm7,[esi+ecx-8]
      movntq  [edi+ecx-8],mm7
      jmp   memoptA_write8loop

memoptA_done8:
      sub   esi,src_pitch
      sub   edi,dst_pitch
      dec   ebx               //row counter (height)
      jne   memoptA_rowloop

      sfence
      emms
    };
    return;
  }//end aligned version
}//end BitBlt_memopt()

// Very optimized memcpy() routine for all AMD Athlon and Duron family.
// This code uses any of FOUR different basic copy methods, depending
// on the transfer size.
// NOTE:  Since this code uses MOVNTQ (also known as "Non-Temporal MOV" or
// "Streaming Store"), and also uses the software prefetchnta instructions,
// be sure you're running on Athlon/Duron or other recent CPU before calling!

#define TINY_BLOCK_COPY 64       // upper limit for movsd type copy
// The smallest copy uses the X86 "movsd" instruction, in an optimized
// form which is an "unrolled loop".

#define IN_CACHE_COPY 64 * 1024  // upper limit for movq/movq copy w/SW prefetch
// Next is a copy that uses the MMX registers to copy 8 bytes at a time,
// also using the "unrolled loop" optimization.   This code uses
// the software prefetch instruction to get the data into the cache.

#define UNCACHED_COPY 197 * 1024 // upper limit for movq/movntq w/SW prefetch
// For larger blocks, which will spill beyond the cache, it's faster to
// use the Streaming Store instruction MOVNTQ.   This write instruction
// bypasses the cache and writes straight to main memory.  This code also
// uses the software prefetch instruction to pre-read the data.
// USE 64 * 1024 FOR THIS VALUE IF YOU'RE ALWAYS FILLING A "CLEAN CACHE"

#define BLOCK_PREFETCH_COPY  infinity // no limit for movq/movntq w/block prefetch
#define CACHEBLOCK 80h // number of 64-byte blocks (cache lines) for block prefetch
// For the largest size blocks, a special technique called Block Prefetch
// can be used to accelerate the read operations.   Block Prefetch reads
// one address per cache line, for a series of cache lines, in a short loop.
// This is faster than using software prefetch.  The technique is great for
// getting maximum read bandwidth, especially in DDR memory systems.

// Inline assembly syntax for use with Visual C++

void memcpy_amd(void *dest, const void *src, size_t n)
{
  __asm {

	mov		ecx, [n]		; number of bytes to copy
	mov		edi, [dest]		; destination
	mov		esi, [src]		; source
	mov		ebx, ecx		; keep a copy of count

	cld
	cmp		ecx, TINY_BLOCK_COPY
	jb		$memcpy_ic_3	; tiny? skip mmx copy

	cmp		ecx, 32*1024		; don't align between 32k-64k because
	jbe		$memcpy_do_align	;  it appears to be slower
	cmp		ecx, 64*1024
	jbe		$memcpy_align_done
$memcpy_do_align:
	mov		ecx, 8			; a trick that's faster than rep movsb...
	sub		ecx, edi		; align destination to qword
	and		ecx, 111b		; get the low bits
	sub		ebx, ecx		; update copy count
	neg		ecx				; set up to jump into the array
	add		ecx, offset $memcpy_align_done
	jmp		ecx				; jump to array of movsb's

align 4
	movsb
	movsb
	movsb
	movsb
	movsb
	movsb
	movsb
	movsb

$memcpy_align_done:			; destination is dword aligned
	mov		ecx, ebx		; number of bytes left to copy
	shr		ecx, 6			; get 64-byte block count
	jz		$memcpy_ic_2	; finish the last few bytes

	cmp		ecx, IN_CACHE_COPY/64	; too big 4 cache? use uncached copy
	jae		$memcpy_uc_test

// This is small block copy that uses the MMX registers to copy 8 bytes
// at a time.  It uses the "unrolled loop" optimization, and also uses
// the software prefetch instruction to get the data into the cache.
align 16
$memcpy_ic_1:			; 64-byte block copies, in-cache copy

	prefetchnta [esi + (200*64/34+192)]		; start reading ahead

	movq	mm0, [esi+0]	; read 64 bits
	movq	mm1, [esi+8]
	movq	[edi+0], mm0	; write 64 bits
	movq	[edi+8], mm1	;    note:  the normal movq writes the
	movq	mm2, [esi+16]	;    data to cache; a cache line will be
	movq	mm3, [esi+24]	;    allocated as needed, to store the data
	movq	[edi+16], mm2
	movq	[edi+24], mm3
	movq	mm0, [esi+32]
	movq	mm1, [esi+40]
	movq	[edi+32], mm0
	movq	[edi+40], mm1
	movq	mm2, [esi+48]
	movq	mm3, [esi+56]
	movq	[edi+48], mm2
	movq	[edi+56], mm3

	add		esi, 64			; update source pointer
	add		edi, 64			; update destination pointer
	dec		ecx				; count down
	jnz		$memcpy_ic_1	; last 64-byte block?

$memcpy_ic_2:
	mov		ecx, ebx		; has valid low 6 bits of the byte count
$memcpy_ic_3:
	shr		ecx, 2			; dword count
	and		ecx, 1111b		; only look at the "remainder" bits
	neg		ecx				; set up to jump into the array
	add		ecx, offset $memcpy_last_few
	jmp		ecx				; jump to array of movsd's

$memcpy_uc_test:
	cmp		ecx, UNCACHED_COPY/64	; big enough? use block prefetch copy
	jae		$memcpy_bp_1

$memcpy_64_test:
	or		ecx, ecx		; tail end of block prefetch will jump here
	jz		$memcpy_ic_2	; no more 64-byte blocks left

// For larger blocks, which will spill beyond the cache, it's faster to
// use the Streaming Store instruction MOVNTQ.   This write instruction
// bypasses the cache and writes straight to main memory.  This code also
// uses the software prefetch instruction to pre-read the data.
align 16
$memcpy_uc_1:				; 64-byte blocks, uncached copy

	prefetchnta [esi + (200*64/34+192)]		; start reading ahead

	movq	mm0,[esi+0]		; read 64 bits
	add		edi,64			; update destination pointer
	movq	mm1,[esi+8]
	add		esi,64			; update source pointer
	movq	mm2,[esi-48]
	movntq	[edi-64], mm0	; write 64 bits, bypassing the cache
	movq	mm0,[esi-40]	;    note: movntq also prevents the CPU
	movntq	[edi-56], mm1	;    from READING the destination address
	movq	mm1,[esi-32]	;    into the cache, only to be over-written
	movntq	[edi-48], mm2	;    so that also helps performance
	movq	mm2,[esi-24]
	movntq	[edi-40], mm0
	movq	mm0,[esi-16]
	movntq	[edi-32], mm1
	movq	mm1,[esi-8]
	movntq	[edi-24], mm2
	movntq	[edi-16], mm0
	dec		ecx
	movntq	[edi-8], mm1
	jnz		$memcpy_uc_1	; last 64-byte block?

	jmp		$memcpy_ic_2		; almost done

// For the largest size blocks, a special technique called Block Prefetch
// can be used to accelerate the read operations.   Block Prefetch reads
// one address per cache line, for a series of cache lines, in a short loop.
// This is faster than using software prefetch, in this case.
// The technique is great for getting maximum read bandwidth,
// especially in DDR memory systems.
$memcpy_bp_1:			; large blocks, block prefetch copy

	cmp		ecx, CACHEBLOCK			; big enough to run another prefetch loop?
	jl		$memcpy_64_test			; no, back to regular uncached copy

	mov		eax, CACHEBLOCK / 2		; block prefetch loop, unrolled 2X
	add		esi, CACHEBLOCK * 64	; move to the top of the block
align 16
$memcpy_bp_2:
	mov		edx, [esi-64]		; grab one address per cache line
	mov		edx, [esi-128]		; grab one address per cache line
	sub		esi, 128			; go reverse order
	dec		eax					; count down the cache lines
	jnz		$memcpy_bp_2		; keep grabbing more lines into cache

	mov		eax, CACHEBLOCK		; now that it's in cache, do the copy
align 16
$memcpy_bp_3:
	movq	mm0, [esi   ]		; read 64 bits
	movq	mm1, [esi+ 8]
	movq	mm2, [esi+16]
	movq	mm3, [esi+24]
	movq	mm4, [esi+32]
	movq	mm5, [esi+40]
	movq	mm6, [esi+48]
	movq	mm7, [esi+56]
	add		esi, 64				; update source pointer
	movntq	[edi   ], mm0		; write 64 bits, bypassing cache
	movntq	[edi+ 8], mm1		;    note: movntq also prevents the CPU
	movntq	[edi+16], mm2		;    from READING the destination address
	movntq	[edi+24], mm3		;    into the cache, only to be over-written,
	movntq	[edi+32], mm4		;    so that also helps performance
	movntq	[edi+40], mm5
	movntq	[edi+48], mm6
	movntq	[edi+56], mm7
	add		edi, 64				; update dest pointer

	dec		eax					; count down

	jnz		$memcpy_bp_3		; keep copying
	sub		ecx, CACHEBLOCK		; update the 64-byte block count
	jmp		$memcpy_bp_1		; keep processing chunks

// The smallest copy uses the X86 "movsd" instruction, in an optimized
// form which is an "unrolled loop".   Then it handles the last few bytes.
align 4
	movsd
	movsd			; perform last 1-15 dword copies
	movsd
	movsd
	movsd
	movsd
	movsd
	movsd
	movsd
	movsd			; perform last 1-7 dword copies
	movsd
	movsd
	movsd
	movsd
	movsd
	movsd

$memcpy_last_few:		; dword aligned from before movsd's
	mov		ecx, ebx	; has valid low 2 bits of the byte count
	and		ecx, 11b	; the last few cows must come home
	jz		$memcpy_final	; no more, let's leave
	rep		movsb		; the last 1, 2, or 3 bytes

$memcpy_final:
	emms				; clean up the MMX state
	sfence				; flush the write buffer
	mov		eax, [dest]	; ret value = destination pointer

    }
}

#endif // #if 0
