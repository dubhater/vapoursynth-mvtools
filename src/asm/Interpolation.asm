%include "include/x86inc.asm"


SECTION_RODATA


word20 times 8 dw 20


SECTION_TEXT


%macro RB2CubicHorizontalInplaceLine_sse2_macro 3 ; weight1, weight2, shift
    ; prepare constants
    pcmpeqw m6, m6 ; = FFFFFFFFFFFFFFFF
    psrlw m6, 15
    psllw m6, %3 - 1; 4 ; *16 = 16

    pcmpeqw m5, m5 ; = FFFFFFFFFFFFFFFF
    psrlw m5, 8 ; /256 = 00FF00FF00FF00FF

    mov r2d, %1; 05
    movd m4, r2d
    pshuflw m4, m4, 0
    punpcklqdq m4, m4

    mov r2d, %2; 10
    movd m3, r2d
    pshuflw m3, m3, 0
    punpcklqdq m3, m3


    mov xq, 1 ; very first (left) is skipped

.loopw:
    cmp xq, widthq
    jge .finish

    movdqu m0, [srcpq + xq * 2 - 2]
    pand m0, m5; clear odd bytes

    movdqu m1, [srcpq + xq * 2 - 1]
    pand m1, m5; clear odd bytes

    movdqu m2, [srcpq + xq * 2]
    pand m2, m5
    movdqu m7, [srcpq + xq * 2 + 1]
    pand m7, m5
    paddw m2, m7
    pmullw m2, m3
    paddw m0, m2

    movdqu m2, [srcpq + xq * 2 + 2]
    pand m2, m5; clear odd bytes
    paddw m1, m2
    pmullw m1, m4
    paddw m0, m1

    movdqu m1, [srcpq + xq * 2 + 3]
    pand m1, m5; clear odd bytes
    paddw m0, m1

    paddw m0, m6; +16
    psrlw m0, %3; /32
    packuswb m0, m0 ; words to bytes
    movq [srcpq + xq], m0; write 4 bytes
    add xq, 8
    jmp .loopw
.finish:
    RET
%endmacro


INIT_XMM
cglobal RB2CubicHorizontalInplaceLine_sse2, 2, 3, 7, srcp, width, x
    RB2CubicHorizontalInplaceLine_sse2_macro 5, 10, 5


INIT_XMM
cglobal RB2QuadraticHorizontalInplaceLine_sse2, 2, 3, 7, srcp, width, x
    RB2CubicHorizontalInplaceLine_sse2_macro 9, 22, 6


%macro RB2CubicVerticalLine_sse2_macro 3 ; RB2CubicVerticalLine_SSE(pDst, pSrc, nSrcPitch, nWidthMMX);
    ; prepare constants
    pcmpeqw m6, m6 ; = FFFFFFFFFFFFFFFF
    psrlw m6, 15
    psllw m6, %3-1; 4 ; *16 = 16

    pxor m7, m7 ; =0

    mov r4d, %1; 05
    movd m4, r4d
    pshuflw m4, m4, 0
    punpcklqdq m4, m4

    mov r4d, %2; 10
    movd m3, r4d
    pshuflw m3, m3, 0
    punpcklqdq m3, m3

    sub srcpq, src_strideq ; prepare for valid addresation
    sub srcpq, src_strideq

    lea src_stride3q, [src_strideq * 3]
    lea src_stride5q, [src_strideq * 5]

    xor xq, xq
.loopw:
    add xq, 8
    cmp xq, widthq
    jg .finish

    movq m0, [srcpq] ; 4 bytes
    punpcklbw m0, m7 ; to 4 words

    movq m1, [srcpq+src_strideq]
    punpcklbw m1, m7
    pmullw m1, m4
    paddw m0, m1

    movq m1, [srcpq+src_strideq*2]
    punpcklbw m1, m7
    pmullw m1, m3
    paddw m0, m1

    movq m1, [srcpq+src_stride3q]
    punpcklbw m1, m7
    pmullw m1, m3
    paddw m0, m1

    movq m1, [srcpq+src_strideq*4]
    punpcklbw m1, m7
    pmullw m1, m4
    paddw m0, m1

    movq m1, [srcpq+src_stride5q]
    punpcklbw m1, m7
    paddw m0, m1

    paddw m0, m6; +16
    psrlw m0, %3; /32
    packuswb m0, m0 ; words to bytes

    movq [dstpq], m0; write 4 bytes

    add srcpq, 8
    add dstpq, 8

    jmp .loopw
.finish:
    RET
%endmacro


INIT_XMM
cglobal RB2CubicVerticalLine_sse2, 4, 7, 8, dstp, srcp, src_stride, width, x, src_stride3, src_stride5
    RB2CubicVerticalLine_sse2_macro 5, 10, 5


INIT_XMM
cglobal RB2QuadraticVerticalLine_sse2, 4, 7, 8, dstp, srcp, src_stride, width, x, src_stride3, src_stride5
    RB2CubicVerticalLine_sse2_macro 9, 22, 6


INIT_XMM
cglobal VerticalBilinear_sse2, 6, 7, 2, dstp, srcp, stride, width, height, x, srcpn
    mov srcpnq, srcpq
    add srcpnq, strideq

    dec heightq

.v_loopy:
    xor xq, xq
.v_loopx:
    movdqu m0, [srcpq + xq]
    movdqu m1, [srcpnq + xq]
    pavgb m0, m1
    movdqu [dstpq + xq], m0
    add xq, 16
    cmp xq, widthq
    jl .v_loopx

    add srcpnq, strideq
    add srcpq, strideq
    add dstpq, strideq

    dec heightq
    jnz .v_loopy

    xor xq, xq
.v_loopfinal:
    movdqu m0, [srcpq + xq]
    movdqu [dstpq + xq], m0
    add xq, 16
    cmp xq, widthq
    jl .v_loopfinal

    RET


INIT_XMM
cglobal HorizontalBilinear_sse2, 6, 6, 2, x, srcp, stride, width, height, dstp
    ; extra mov because I need x to contain a byte-sized register and only
    ; r0 and r1 have it everywhere (Windows, Linux, OS X, 32 bit and 64 bit).
    ; See DECLARE_REG in x86inc.asm.
    mov dstpq, xq
.h_loopy:
    xor xq, xq
.h_loopx:
    movdqu m0, [srcpq + xq]
    movdqu m1, [srcpq + xq + 1]
    pavgb m0, m1
    movdqu [dstpq + xq], m0
    add xq, 16
    cmp xq, widthq
    jl .h_loopx

    mov xb, [srcpq + widthq - 1]
    mov [dstpq + widthq - 1], xb

    add srcpq, strideq
    add dstpq, strideq

    dec heightq
    jnz .h_loopy

    RET


INIT_XMM
cglobal DiagonalBilinear_sse2, 6, 7, 6, x, srcp, stride, width, height, dstp, srcpn
    ; extra mov because I need x to contain a byte-sized register and only
    ; r0 and r1 have it everywhere (Windows, Linux, OS X, 32 bit and 64 bit).
    ; See DECLARE_REG in x86inc.asm.
    mov dstpq, xq

    pcmpeqw m4, m4
    psrlw m4, 15
    psllw m4, 1

    pxor m5, m5

    lea srcpnq, [srcpq + strideq]

.d_loopy:
    xor xq, xq
.d_loopx:
    movq m0, [srcpq+xq]
    movq m1, [srcpq+xq+1]

    movq m2, [srcpnq+xq]
    movq m3, [srcpnq+xq+1]

    punpcklbw m0, m5
    punpcklbw m1, m5
    punpcklbw m2, m5
    punpcklbw m3, m5

    paddw m0, m1
    paddw m0, m2
    paddw m0, m3
    paddw m0, m4

    psrlw m0, 2

    packuswb m0, m5

    movq [dstpq+xq], m0

    add xq, 8
    cmp xq, widthq
    jl .d_loopx

    ; using xmm registers because nothing else is available
    movzx xd, byte [srcpq + widthq - 1]
    movd m0, xd
    movzx xd, byte [srcpnq + widthq - 1]
    movd m1, xd
    pavgb m0, m1
    movd xd, m0
    mov [dstpq + widthq - 1], xb

    add srcpq, strideq
    add dstpq, strideq
    add srcpnq, strideq

    dec heightq
    jnz .d_loopy

    xor xq, xq
.d_loop_final :
    movq m0, [srcpq+xq]
    movq m1, [srcpq+xq+1]
    pavgb m0, m1
    movq [dstpq+xq], m0
    add xq, 8
    cmp xq, widthq
    jl .d_loop_final

    mov xb, [srcpq + widthq - 1]
    mov [dstpq + widthq - 1], xb

    RET


INIT_XMM
cglobal VerticalWiener_sse2, 6, 7, 8, dstp, srcp, stride, width, height, x, srcpn
    mov srcpnq, srcpq
    add srcpnq, strideq

    sub heightq, 4

    xor xq, xq
.vw_loopx0:
    movdqu m0, [srcpq + xq]
    movdqu m1, [srcpnq + xq]
    pavgb m0, m1
    movdqu [dstpq + xq], m0

    add xq, 16
    cmp xq, widthq
    jl .vw_loopx0

    add srcpq, strideq
    add srcpnq, strideq
    add dstpq, strideq
    dec heightq

    xor xq, xq
.vw_loopx1:
    movdqu m0, [srcpq + xq]
    movdqu m1, [srcpnq + xq]
    pavgb m0, m1
    movdqu [dstpq + xq], m0

    add xq, 16
    cmp xq, widthq
    jl .vw_loopx1

    sub srcpq, strideq ; return it to the original value
    add dstpq, strideq
    dec heightq


    pcmpeqw m6, m6
    psrlw m6, 15
    psllw m6, 4

    pxor m7, m7

.vw_loopy:
    xor xq, xq
.vw_loopx:
    mov r6, srcpq
    movq m0, [srcpq + xq]
    add r6, strideq
    movq m1, [r6 + xq]
    add r6, strideq
    movq m2, [r6 + xq]
    add r6, strideq
    movq m3, [r6 + xq]
    add r6, strideq
    movq m4, [r6 + xq]
    add r6, strideq
    movq m5, [r6 + xq]

    punpcklbw m0, m7
    punpcklbw m1, m7
    punpcklbw m2, m7
    punpcklbw m3, m7
    punpcklbw m4, m7
    punpcklbw m5, m7

    paddw m2, m3
    psllw m2, 2
    movdqa m3, m2
    psllw m3, 2
    paddw m2, m3

    paddw m0, m5
    paddw m0, m2

    paddw m0, m6

    paddw m1, m4
    movdqa m4, m1
    psllw m4, 2
    paddw m1, m4

    psubusw m0, m1
    psrlw m0, 5

    packuswb m0, m0

    movq [dstpq + xq], m0

    add xq, 8
    cmp xq, widthq
    jl .vw_loopx

    add srcpq, strideq
    add dstpq, strideq
    dec heightq
    jnz .vw_loopy


    add srcpq, strideq
    add srcpq, strideq
    mov srcpnq, srcpq
    add srcpnq, strideq

    xor xq, xq
.vw_loopxh_4:
    movdqu m0, [srcpq + xq]
    movdqu m1, [srcpnq + xq]
    pavgb m0, m1
    movdqu [dstpq + xq], m0
    add xq, 16
    cmp xq, widthq
    jl .vw_loopxh_4

    add srcpq, strideq
    add srcpnq, strideq
    add dstpq, strideq
    dec heightq

    xor xq, xq
.vw_loopxh_3:
    movdqu m0, [srcpq + xq]
    movdqu m1, [srcpnq + xq]
    pavgb m0, m1
    movdqu [dstpq + xq], m0
    add xq, 16
    cmp xq, widthq
    jl .vw_loopxh_3

    add srcpq, strideq
    add srcpnq, strideq
    add dstpq, strideq
    dec heightq

    xor xq, xq
.vw_loopxh_2:
    movdqu m0, [srcpq + xq]
    movdqu m1, [srcpnq + xq]
    pavgb m0, m1
    movdqu [dstpq + xq], m0
    add xq, 16
    cmp xq, widthq
    jl .vw_loopxh_2

    add srcpq, strideq
    add srcpnq, strideq
    add dstpq, strideq
    dec heightq

    xor xq, xq
.vw_loopfinal:
    movdqu m0, [srcpq + xq]
    movdqu [dstpq + xq], m0
    add xq, 16
    cmp xq, widthq
    jl .vw_loopfinal

    RET


INIT_XMM
cglobal HorizontalWiener_sse2, 6, 6, 9, x, srcp, stride, width, height, dstp
    ; extra mov because I need x to contain a byte-sized register and only
    ; r0 and r1 have it everywhere (Windows, Linux, OS X, 32 bit and 64 bit).
    ; See DECLARE_REG in x86inc.asm.
    mov dstpq, xq

    pcmpeqw m6, m6
    psrlw m6, 15
    psllw m6, 4

    pxor m7, m7

    sub widthq, 4

.hw_loopy:
    movd m0, [srcpq]
    movd m1, [srcpq + 1]
    pavgb m0, m1
    movd [dstpq], m0

    mov xq, 4
.hw_loopx:
    movq m0, [srcpq + xq - 2]
    movq m1, [srcpq + xq - 1]
    punpcklbw m0, m7
    punpcklbw m1, m7
    movq m2, [srcpq + xq]
    movq m3, [srcpq + xq + 1]
    punpcklbw m2, m7
    punpcklbw m3, m7
    movq m4, [srcpq + xq + 2]
    movq m5, [srcpq + xq + 3]
    punpcklbw m4, m7
    punpcklbw m5, m7

    paddw m2, m3

    pmullw m2, [word20]

    paddw m0, m5
    paddw m0, m2

    paddw m0, m6

    paddw m1, m4

    movdqa m4, m1
    psllw m4, 2
    paddw m1, m4

    psubusw m0, m1
    psrlw m0, 5

    packuswb m0, m0

    movq [dstpq + xq], m0
    add xq, 8
    cmp xq, widthq
    jl .hw_loopx

    movd m0, [srcpq + xq]
    movd m1, [srcpq + xq + 1]
    pavgb m0, m1
    movd [dstpq + xq], m0

    mov xb, [srcpq + widthq + 3]
    mov [dstpq + widthq + 3], xb

    add srcpq, strideq
    add dstpq, strideq

    dec heightq
    jnz .hw_loopy

    RET


INIT_XMM
cglobal RB2BilinearFilteredVerticalLine_sse2, 4, 6, 8, dstp, srcp, src_stride, width, x, src_stride3
    pcmpeqw m6, m6
    psrlw m6, 15
    psllw m6, 2

    pxor m7, m7

    lea src_stride3q, [src_strideq * 3]

    sub srcpq, src_strideq

    xor xq, xq
.loopw:
    add xq, 8
    cmp xq, widthq
    jg .finish

    movq m0, [srcpq]
    punpcklbw m0, m7

    movq m1, [srcpq + src_strideq]
    punpcklbw m1, m7

    movq m2, [srcpq + src_strideq * 2]
    punpcklbw m2, m7

    paddw m1, m2
    movdqa m2, m1
    psllw m1, 1
    paddw m1, m2

    paddw m0, m1

    movq m1, [srcpq + src_stride3q]
    punpcklbw m1, m7

    paddw m0, m1

    paddw m0, m6
    psrlw m0, 3

    packuswb m0, m0
    movq [dstpq], m0

    add srcpq, 8
    add dstpq, 8

    jmp .loopw

.finish:
    RET


INIT_XMM
cglobal RB2BilinearFilteredHorizontalInplaceLine_sse2, 2, 3, 8, srcp, width, x
    pcmpeqw m6, m6
    psrlw m6, 15
    psllw m6, 2

    pxor m7, m7

    pcmpeqw m5, m5
    psrlw m5, 8


    mov xq, 1
.loopw:
    cmp xq, widthq
    jge .finish

    movdqu m0, [srcpq + xq * 2 - 1]
    pand m0, m5

    movdqu m1, [srcpq + xq * 2]
    pand m1, m5

    movdqu m2, [srcpq + xq * 2 + 1]
    pand m2, m5

    paddw m1, m2
    movdqa m2, m1
    psllw m2, 1
    paddw m1, m2

    paddw m0, m1

    movdqu m1, [srcpq + xq * 2 + 2]
    pand m1, m5

    paddw m0, m1

    paddw m0, m6
    psrlw m0, 3

    packuswb m0, m0
    movq [srcpq + xq], m0

    add xq, 8
    jmp .loopw
.finish:
    RET


INIT_XMM
cglobal Average2_sse2, 6, 7, 2, dstp, srcp1, srcp2, stride, width, height, x
.yloop:
    xor xq, xq
.xloop:
    movdqu m0, [srcp1q + xq]
    movdqu m1, [srcp2q + xq]
    pavgb m0, m1
    movdqu [dstpq + xq], m0

    add xq, 16
    cmp xq, widthq
    jl .xloop

    add srcp1q, strideq
    add srcp2q, strideq
    add dstpq, strideq
    dec heightq
    jnz .yloop

    RET
