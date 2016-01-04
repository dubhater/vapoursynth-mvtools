%include "include/x86inc.asm"


;SECTION_RODATA





SECTION_TEXT


INIT_XMM
cglobal sad_4x2_sse2, 4, 4, 4, srcp1, stride1, srcp2, stride2
    movd m0, [srcp1q]
    movd m1, [srcp2q]
    movd m2, [srcp1q + stride1q]
    movd m3, [srcp2q + stride2q]
    punpckldq m0, m2
    punpckldq m1, m3

    psadbw m0, m1

    movd eax, m0

    RET


INIT_XMM
cglobal sad_8x1_sse2, 4, 4, 2, srcp1, stride1, srcp2, stride2
    movq m0, [srcp1q]
    movq m1, [srcp2q]

    psadbw m0, m1

    movd eax, m0

    RET


INIT_XMM
cglobal sad_8x2_sse2, 4, 4, 2, srcp1, stride1, srcp2, stride2
    movq m0, [srcp1q]
    movhps m0, [srcp1q + stride1q]
    movq m1, [srcp2q]
    movhps m1, [srcp2q + stride2q]

    psadbw m0, m1

    movhlps m1, m0
    paddw m0, m1
    movd eax, m0

    RET


INIT_XMM
cglobal sad_16x1_sse2, 4, 4, 2, srcp1, stride1, srcp2, stride2
    movdqu m0, [srcp1q]
    movdqu m1, [srcp2q]

    psadbw m0, m1

    movhlps m1, m0
    paddw m0, m1
    movd eax, m0

    RET


INIT_XMM
cglobal sad_16x2_sse2, 4, 4, 4, srcp1, stride1, srcp2, stride2
    movdqu m0, [srcp1q]
    movdqu m1, [srcp1q + stride1q]
    movdqu m2, [srcp2q]
    movdqu m3, [srcp2q + stride2q]

    psadbw m0, m2
    psadbw m1, m3

    paddw m0, m1

    movhlps m1, m0
    paddw m0, m1
    movd eax, m0

    RET


%macro SAD16x2 0
    movdqu m1, [srcp1q]
    movdqu m2, [srcp1q + stride1q]
    movdqu m3, [srcp2q]
    movdqu m4, [srcp2q + stride2q]
    psadbw m1, m3
    psadbw m2, m4
    lea srcp1q, [srcp1q + stride1q * 2]
    paddd m0, m1
    paddd m0, m2
    lea srcp2q, [srcp2q + stride2q * 2]
%endmacro


INIT_XMM
cglobal sad_16x4_sse2, 4, 4, 5, srcp1, stride1, srcp2, stride2
    pxor m0, m0

    SAD16x2
    SAD16x2

    movhlps m1, m0
    paddd m0, m1
    movd eax, m0

    RET


INIT_XMM
cglobal sad_16x32_sse2, 4, 4, 5, srcp1, stride1, srcp2, stride2
    pxor m0, m0

    SAD16x2
    SAD16x2
    SAD16x2
    SAD16x2

    SAD16x2
    SAD16x2
    SAD16x2
    SAD16x2

    SAD16x2
    SAD16x2
    SAD16x2
    SAD16x2

    SAD16x2
    SAD16x2
    SAD16x2
    SAD16x2

    movhlps m1, m0
    paddd m0, m1
    movd eax, m0

    RET


%macro SAD32x2 0
    movdqu m1, [srcp1q]
    movdqu m2, [srcp1q + 16]
    movdqu m3, [srcp2q]
    movdqu m4, [srcp2q + 16]

    psadbw m1, m3
    psadbw m2, m4

    movdqu m3, [srcp1q + stride1q]
    movdqu m4, [srcp1q + stride1q + 16]
    movdqu m5, [srcp2q + stride2q]
    movdqu m6, [srcp2q + stride2q + 16]

    psadbw m3, m5
    psadbw m4, m6

    lea srcp1q, [srcp1q + stride1q * 2]
    paddw m1, m2
    paddw m3, m4
    lea srcp2q, [srcp2q + stride2q * 2]
    paddd m0, m1
    paddd m0, m3
%endmacro


INIT_XMM
cglobal sad_32x8_sse2, 4, 4, 7, srcp1, stride1, srcp2, stride2
    pxor m0, m0

    SAD32x2
    SAD32x2
    SAD32x2
    SAD32x2

    movhlps m1, m0
    paddd m0, m1
    movd eax, m0

    RET


INIT_XMM
cglobal sad_32x16_sse2, 4, 4, 7, srcp1, stride1, srcp2, stride2
    pxor m0, m0

    SAD32x2
    SAD32x2
    SAD32x2
    SAD32x2

    SAD32x2
    SAD32x2
    SAD32x2
    SAD32x2

    movhlps m1, m0
    paddd m0, m1
    movd eax, m0

    RET


INIT_XMM
cglobal sad_32x32_sse2, 4, 4, 7, srcp1, stride1, srcp2, stride2
    pxor m0, m0

    SAD32x2
    SAD32x2
    SAD32x2
    SAD32x2

    SAD32x2
    SAD32x2
    SAD32x2
    SAD32x2

    SAD32x2
    SAD32x2
    SAD32x2
    SAD32x2

    SAD32x2
    SAD32x2
    SAD32x2
    SAD32x2

    movhlps m1, m0
    paddd m0, m1
    movd eax, m0

    RET


INIT_XMM
cglobal sad_2x2_u16_sse2, 4, 4, 6, srcp1, stride1, srcp2, stride2
    pxor m0, m0

    movd m2, [srcp1q]
    movd m4, [srcp1q + stride1q]
    punpcklwd m2, m4
    movd m3, [srcp2q]
    movd m5, [srcp2q + stride2q]
    punpcklwd m3, m5

    movdqa m4, m2
    psubusw m2, m3
    psubusw m3, m4
    por m2, m3

    punpcklwd m2, m0

    movhlps m1, m2
    paddd m2, m1

    pshufd m1, m2, 11100101b
    paddd m1, m2
    movd eax, m1

    RET


INIT_XMM
cglobal sad_2x4_u16_sse2, 4, 4, 8, srcp1, stride1, srcp2, stride2
    pxor m0, m0

    movd m2, [srcp1q]
    movd m4, [srcp1q + stride1q]
    punpcklwd m2, m4
    movd m3, [srcp2q]
    movd m5, [srcp2q + stride2q]
    punpcklwd m3, m5

    lea srcp1q, [srcp1q + stride1q * 2]
    lea srcp2q, [srcp2q + stride2q * 2]

    movd m4, [srcp1q]
    movd m6, [srcp1q + stride1q]
    punpcklwd m4, m6
    movd m5, [srcp2q]
    movd m7, [srcp2q + stride2q]
    punpcklwd m5, m7

    punpcklwd m2, m4
    punpcklwd m3, m5

    movdqa m4, m2
    psubusw m2, m3
    psubusw m3, m4
    por m2, m3

    movdqa m3, m2
    punpcklwd m2, m0
    punpckhwd m3, m0
    paddd m2, m3

    movhlps m0, m2
    paddd m0, m2
    pshufd m1, m0, 11100101b
    paddd m0, m1
    movd eax, m0

    RET


%macro SAD_U16_4x2_4x8 1
INIT_XMM
cglobal sad_4x%1_u16_sse2, 4, 5, 6, srcp1, stride1, srcp2, stride2
    pxor m0, m0
    pxor m1, m1

    mov r4d, %1
.loop:
    movq m2, [srcp1q]
    movq m3, [srcp2q]

    add srcp1q, stride1q
    add srcp2q, stride2q

    movdqa m4, m2
    psubusw m2, m3
    psubusw m3, m4
    por m2, m3

    punpcklwd m2, m1
    paddd m0, m2

    sub r4d, 1
    jnz .loop

    movhlps m1, m0
    paddd m0, m1
    pshufd m1, m0, 11100101b
    paddd m0, m1
    movd eax, m0

    RET
%endmacro


%macro SAD_U16_8x1_32x32 2
INIT_XMM
cglobal sad_%1x%2_u16_sse2, 4, 6, 6, srcp1, stride1, srcp2, stride2
    pxor m0, m0
    pxor m1, m1

    mov r4d, %2
.yloop:
    xor r5q, r5q
.xloop:
    movdqu m2, [srcp1q + r5q]
    movdqu m3, [srcp2q + r5q]
    movdqa m4, m2

    psubusw m2, m3
    psubusw m3, m4
    por m2, m3

    movdqa m3, m2
    punpcklwd m2, m1
    paddd m0, m2
    punpckhwd m3, m1
    paddd m0, m3

    add r5q, 16
    cmp r5q, %1 * 2
    jne .xloop

    add srcp1q, stride1q
    add srcp2q, stride2q
    sub r4d, 1
    jnz .yloop

    movhlps m1, m0
    paddd m0, m1
    pshufd m1, m0, 11100101b
    paddd m0, m1
    movd eax, m0

    RET
%endmacro


SAD_U16_4x2_4x8 2
SAD_U16_4x2_4x8 4
SAD_U16_4x2_4x8 8

SAD_U16_8x1_32x32 8, 1
SAD_U16_8x1_32x32 8, 2
SAD_U16_8x1_32x32 8, 4
SAD_U16_8x1_32x32 8, 8
SAD_U16_8x1_32x32 8, 16
SAD_U16_8x1_32x32 16, 1
SAD_U16_8x1_32x32 16, 2
SAD_U16_8x1_32x32 16, 4
SAD_U16_8x1_32x32 16, 8
SAD_U16_8x1_32x32 16, 16
SAD_U16_8x1_32x32 16, 32
SAD_U16_8x1_32x32 32, 8
SAD_U16_8x1_32x32 32, 16
SAD_U16_8x1_32x32 32, 32
