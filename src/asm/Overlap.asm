%include "include/x86inc.asm"


SECTION_TEXT


%macro OVERS2 0
    movd m0, [srcpq] ; two more than needed
    punpcklbw m0, m7
    movd m1, [winpq]
    movdqa m2, m0
    pmullw m0, m1
    pmulhw m2, m1
    punpcklwd m0, m2
    psrld m0, 6
    packssdw m0, m7
    movd m1, [dstpq]
    paddusw m0, m1
    movd [dstpq], m0

    add dstpq, dst_strideq
    add srcpq, src_strideq
    add winpq, win_strideq
%endmacro


INIT_XMM
cglobal Overlaps2x2_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7    ; =0

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS2
    OVERS2

    RET


INIT_XMM
cglobal Overlaps2x4_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7    ; =0

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS2
    OVERS2
    OVERS2
    OVERS2

    RET


%macro OVERS4 0
    movd m0, [srcpq]
    punpcklbw m0, m7
    movq m1, [winpq]
    movdqa m2, m0
    pmullw m0, m1
    pmulhw m2, m1
    punpcklwd m0, m2
    psrld m0, 6
    packssdw m0, m7
    movq m1, [dstpq]
    paddusw m0, m1
    movq [dstpq], m0

    add dstpq, dst_strideq
    add srcpq, src_strideq
    add winpq, win_strideq
%endmacro


INIT_XMM
cglobal Overlaps4x2_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7    ; =0

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS4
    OVERS4

    RET


INIT_XMM
cglobal Overlaps4x4_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7    ; =0

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS4
    OVERS4
    OVERS4
    OVERS4

    RET


INIT_XMM
cglobal Overlaps4x8_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7    ; =0

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS4
    OVERS4
    OVERS4
    OVERS4

    OVERS4
    OVERS4
    OVERS4
    OVERS4

    RET


; 0 or 1 parameters.
%macro OVERS8 0-1
; Done like this because if I supply a default value (of 0) for %1,
; %0 is always 1, whether a parameter was actually passed or not.
%if %0 == 0
    %assign offset 0
%else
    %assign offset %1
%endif

    movq m0, [srcpq + offset/2]
    punpcklbw m0, m7
    movdqu m1, [winpq + offset] ; TODO: check if winpq is aligned
    movdqa m2, m0
    pmullw m0, m1
    pmulhw m2, m1
    movdqa m1, m0
    punpcklwd m0, m2
    punpckhwd m1, m2
    psrld m0, 6
    psrld m1, 6
    packssdw m0, m1
    movdqu m1, [dstpq + offset] ; TODO: check if dstpq is aligned
    paddusw m0, m1
    movdqu [dstpq + offset], m0

; if no parameters were passed
%if %0 == 0
    add dstpq, dst_strideq
    add srcpq, src_strideq
    add winpq, win_strideq
%endif
%endmacro


INIT_XMM
cglobal Overlaps8x1_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    OVERS8

    RET


INIT_XMM
cglobal Overlaps8x2_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS8
    OVERS8

    RET


INIT_XMM
cglobal Overlaps8x4_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS8
    OVERS8
    OVERS8
    OVERS8

    RET


INIT_XMM
cglobal Overlaps8x8_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS8
    OVERS8
    OVERS8
    OVERS8

    OVERS8
    OVERS8
    OVERS8
    OVERS8

    RET


INIT_XMM
cglobal Overlaps8x16_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS8
    OVERS8
    OVERS8
    OVERS8

    OVERS8
    OVERS8
    OVERS8
    OVERS8

    OVERS8
    OVERS8
    OVERS8
    OVERS8

    OVERS8
    OVERS8
    OVERS8
    OVERS8

    RET


; OVERS16 is two OVERS8 per line.
%macro OVERS16 0
    OVERS8 0
    OVERS8 16

    add dstpq, dst_strideq
    add srcpq, src_strideq
    add winpq, win_strideq
%endmacro


INIT_XMM
cglobal Overlaps16x1_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS16

    RET


INIT_XMM
cglobal Overlaps16x2_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS16
    OVERS16

    RET


INIT_XMM
cglobal Overlaps16x4_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    RET


INIT_XMM
cglobal Overlaps16x8_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    RET


INIT_XMM
cglobal Overlaps16x16_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    RET


INIT_XMM
cglobal Overlaps16x32_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    OVERS16
    OVERS16
    OVERS16
    OVERS16

    RET


; OVERS32 is four OVERS8 per line.
%macro OVERS32 0
    OVERS8 0
    OVERS8 16
    OVERS8 32
    OVERS8 48

    add dstpq, dst_strideq
    add srcpq, src_strideq
    add winpq, win_strideq
%endmacro


INIT_XMM
cglobal Overlaps32x8_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    RET


INIT_XMM
cglobal Overlaps32x16_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    RET


INIT_XMM
cglobal Overlaps32x32_sse2, 6, 6, 8, dstp, dst_stride, srcp, src_stride, winp, win_stride

    ; prepare constants
    pxor m7, m7

    ; It's in pixels, apparently.
    add win_strideq, win_strideq

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    OVERS32
    OVERS32
    OVERS32
    OVERS32

    RET
