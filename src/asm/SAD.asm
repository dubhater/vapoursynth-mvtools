%include "include/x86inc.asm"


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
