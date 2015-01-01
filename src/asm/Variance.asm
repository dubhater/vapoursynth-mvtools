%include "include/x86inc.asm"


SECTION_TEXT

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro VAR4x4 0
    movd m0, [srcpq]                ; first line
    movd m1, [srcpq + strideq]        ; second line
    movd m4, [srcpq + strideq * 2]   ; third line
    movd m5, [srcpq + stride3q]        ; fourth line

    psadbw m0, m7
    psadbw m1, m6

    psadbw m4, m7
    psadbw m5, m6

    paddw m0, m4
    paddw m1, m5

    paddw m0, m1
%endmacro


INIT_XMM
cglobal Luma4x4_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR4x4

    movd eax, m0

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro VAR8x4 0
    movq m0, [srcpq]                ; first line
    movq m1, [srcpq + strideq]        ; second line

    movq m4, [srcpq + strideq * 2]   ; third line
    movq m5, [srcpq + stride3q]        ; fourth line

    psadbw m0, m7
    psadbw m1, m6

    psadbw m4, m7
    psadbw m5, m6

    paddw m0, m4
    paddw m1, m5

    paddw m0, m1
%endmacro


INIT_XMM
cglobal Luma8x4_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR8x4

    movd eax, m0

    RET


; mm7 and mm6 must be times 8 db mean
; mm0 will be the var
; mm0..mm7 are used
%macro VAR8x8 0
    movq m0, [srcpq]                ; first line
    movq m1, [srcpq + strideq]        ; second line
    movq m4, [srcpq + strideq * 2]   ; third line
    movq m5, [srcpq + stride3q]        ; fourth line

    psadbw m0, m7
    psadbw m1, m6

    psadbw m4, m7
    psadbw m5, m6

    paddw m0, m4
    paddw m1, m5

    lea srcpq, [srcpq + strideq * 4]    ;

    movq m2, [srcpq]                ; fifth line
    movq m3, [srcpq + strideq]        ; sixth line
    movq m4, [srcpq + strideq * 2]    ; seventh line
    movq m5, [srcpq + stride3q]        ; eighth line

    psadbw m2, m7
    psadbw m3, m6

    psadbw m4, m7
    psadbw m5, m6

    paddw m0, m2
    paddw m1, m3

    paddw m0, m4
    paddw m1, m5

    paddw m0, m1
%endmacro


INIT_XMM
cglobal Luma8x8_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR8x8

    movd eax, m0

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro VAR16x2 0
    movdqu m0, [srcpq]
    movdqu m1, [srcpq + strideq]

    psadbw m0, m6
    psadbw m1, m7

    paddd m0, m1
    movdqa m1, m0
    psrldq m1, 8
    paddd m0, m1
%endmacro


INIT_XMM
cglobal Luma16x2_sse2, 2, 2, 8, srcp, stride
    pxor m6, m6
    pxor m7, m7

    VAR16x2

    movd eax, m0

    RET


%macro VAR16x4 0
    movdqu m2, [srcpq]
    movdqu m3, [srcpq + strideq]
    movdqu m4, [srcpq + strideq * 2]
    movdqu m5, [srcpq + stride3q]

    psadbw m2, m6
    psadbw m3, m7
    psadbw m4, m6
    psadbw m5, m7

    lea srcpq, [srcpq + strideq * 4]

    paddd m0, m2
    paddd m1, m3
    paddd m0, m4
    paddd m1, m5
%endmacro


%macro VAR16x8 0
    pxor m0, m0
    pxor m1, m1

    VAR16x4
    VAR16x4

    paddd m0, m1
    movdqa m1, m0
    psrldq m1, 8
    paddd m0, m1
%endmacro


INIT_XMM
cglobal Luma16x8_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR16x8

    movd eax, m0

    RET


%macro VAR16x16 0
    pxor m0, m0
    pxor m1, m1

    VAR16x4
    VAR16x4
    VAR16x4
    VAR16x4

    paddd m0, m1
    movdqa m1, m0
    psrldq m1, 8
    paddd m0, m1
%endmacro


INIT_XMM
cglobal Luma16x16_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR16x16

    movd eax, m0

    RET


%macro VAR16x32 0
    pxor m0, m0
    pxor m1, m1

    VAR16x4
    VAR16x4
    VAR16x4
    VAR16x4

    VAR16x4
    VAR16x4
    VAR16x4
    VAR16x4

    paddd m0, m1
    movdqa m1, m0
    psrldq m1, 8
    paddd m0, m1
%endmacro


INIT_XMM
cglobal Luma16x32_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR16x32

    movd eax, m0

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro VAR32x2 0
    movdqu m2, [srcpq]
    movdqu m3, [srcpq + 16]
    movdqu m4, [srcpq + strideq]
    movdqu m5, [srcpq + strideq + 16]

    psadbw m2, m6
    psadbw m3, m7
    psadbw m4, m6
    psadbw m5, m7

    lea srcpq, [srcpq + strideq * 2]

    paddd m0, m2
    paddd m1, m3
    paddd m0, m4
    paddd m1, m5
%endmacro


%macro VAR32x16 0
    pxor m0, m0
    pxor m1, m1

    VAR32x2
    VAR32x2
    VAR32x2
    VAR32x2

    VAR32x2
    VAR32x2
    VAR32x2
    VAR32x2

    paddd m0, m1
    movdqa m1, m0
    psrldq m1, 8
    paddd m0, m1
%endmacro


INIT_XMM
cglobal Luma32x16_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR32x16

    movd eax, m0

    RET


%macro VAR32x32 0
    pxor m0, m0
    pxor m1, m1

    VAR32x2
    VAR32x2
    VAR32x2
    VAR32x2

    VAR32x2
    VAR32x2
    VAR32x2
    VAR32x2

    VAR32x2
    VAR32x2
    VAR32x2
    VAR32x2

    VAR32x2
    VAR32x2
    VAR32x2
    VAR32x2

    paddd m0, m1
    movdqa m1, m0
    psrldq m1, 8
    paddd m0, m1
%endmacro


INIT_XMM
cglobal Luma32x32_sse2, 2, 3, 8, srcp, stride, stride3
    lea stride3q, [strideq * 3]

    pxor m6, m6
    pxor m7, m7

    VAR32x32

    movd eax, m0

    RET
