%include "include/x86inc.asm"


SECTION_TEXT


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro MOV2x2 0
    mov r4w, [srcpq]
    mov r5w, [srcpq + src_strideq]
    mov [dstpq], r4w
    mov [dstpq + dst_strideq], r5w
%endmacro


INIT_XMM
cglobal Copy2x2_sse2, 4, 6, 0, dstp, dst_stride, srcp, src_stride
    ; lol sse2

    ; Original uses 
    ;   movsw
    ;   lea esi, [esi + ecx - 2]
    ;   lea edi, [edi + edx - 2]
    ;   movsw
    ; but whatever.
    MOV2x2

    RET


INIT_XMM
cglobal Copy2x4_sse2, 4, 6, 0, dstp, dst_stride, srcp, src_stride
    ; lol sse2

    MOV2x2

    lea srcpq, [srcpq + 2 * src_strideq]
    lea dstpq, [dstpq + 2 * dst_strideq]

    MOV2x2

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro MOV4x2 0
    mov r4d, [srcpq]
    mov r5d, [srcpq + src_strideq]
    mov [dstpq], r4d
    mov [dstpq + dst_strideq], r5d
%endmacro


INIT_XMM
cglobal Copy4x2_sse2, 4, 6, 0, dstp, dst_stride, srcp, src_stride
    ; lol sse2

    MOV4x2

    RET


INIT_XMM
cglobal Copy4x4_sse2, 4, 6, 0, dstp, dst_stride, srcp, src_stride
    ; lol sse2

    MOV4x2

    lea srcpq, [srcpq + 2 * src_strideq]
    lea dstpq, [dstpq + 2 * dst_strideq]

    MOV4x2

    RET


%macro MOV4x4 0
    movd m0, [srcpq]
    movd m1, [srcpq + src_strideq]
    movd m2, [srcpq + src_strideq * 2]
    movd m3, [srcpq + src_stride3q]
    movd [dstpq], m0
    movd [dstpq + dst_strideq], m1
    movd [dstpq + dst_strideq * 2], m2
    movd [dstpq + dst_stride3q], m3
%endmacro


INIT_XMM
cglobal Copy4x8_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV4x4

    lea srcpq, [srcpq + 4 * src_strideq]
    lea dstpq, [dstpq + 4 * dst_strideq]

    MOV4x4

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro MOV8x4 0
    movq m0, [srcpq]                     ; 0 >
    movq m1, [srcpq + src_strideq]        ; 1 >
    movq m2, [srcpq + src_strideq * 2]    ; 2 >
    movq m3, [srcpq + src_stride3q]                ; 3 >
    movq [dstpq], m0                     ; < 0
    movq [dstpq + dst_strideq], m1        ; < 1
    movq [dstpq + dst_strideq * 2], m2    ; < 2
    movq [dstpq + dst_stride3q], m3                ; < 3
%endmacro


INIT_XMM
cglobal Copy8x1_sse2, 4, 4, 1, dstp, dst_stride, srcp, src_stride
    movq m0, [srcpq]
    movq [dstpq], m0

    RET


INIT_XMM
cglobal Copy8x2_sse2, 4, 4, 2, dstp, dst_stride, srcp, src_stride
    movq m0, [srcpq]
    movq m1, [srcpq + src_strideq]
    movq [dstpq], m0
    movq [dstpq + dst_strideq], m1

    RET


INIT_XMM
cglobal Copy8x4_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV8x4

    RET


INIT_XMM
cglobal Copy8x8_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV8x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV8x4

    RET


INIT_XMM
cglobal Copy8x16_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV8x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV8x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV8x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV8x4

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro MOV16x4 0
    movdqu m0, [srcpq]
    movdqu m1, [srcpq + src_strideq]
    movdqu m2, [srcpq + src_strideq * 2]
    movdqu m3, [srcpq + src_stride3q]

    movdqu [dstpq], m0
    movdqu [dstpq + dst_strideq], m1
    movdqu [dstpq + dst_strideq * 2], m2
    movdqu [dstpq + dst_stride3q], m3
%endmacro


INIT_XMM
cglobal Copy16x1_sse2, 4, 4, 1, dstp, dst_stride, srcp, src_stride
    movdqu m0, [srcpq]

    movdqu [dstpq], m0

    RET


INIT_XMM
cglobal Copy16x2_sse2, 4, 4, 2, dstp, dst_stride, srcp, src_stride
    movdqu m0, [srcpq]
    movdqu m1, [srcpq + src_strideq]

    movdqu [dstpq], m0
    movdqu [dstpq + dst_strideq], m1

    RET


INIT_XMM
cglobal Copy16x4_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV16x4

    RET


INIT_XMM
cglobal Copy16x8_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    RET


INIT_XMM
cglobal Copy16x16_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    RET


INIT_XMM
cglobal Copy16x32_sse2, 4, 6, 4, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV16x4

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro MOV32x4 0
    movdqu m0, [srcpq]
    movdqu m1, [srcpq + 16]
    movdqu m2, [srcpq + src_strideq]
    movdqu m3, [srcpq + src_strideq + 16]
    movdqu m4, [srcpq + src_strideq * 2]
    movdqu m5, [srcpq + src_strideq * 2 + 16]
    movdqu m6, [srcpq + src_stride3q]
    movdqu m7, [srcpq + src_stride3q + 16]

    movdqu [dstpq], m0
    movdqu [dstpq + 16], m1
    movdqu [dstpq + dst_strideq], m2
    movdqu [dstpq + dst_strideq + 16], m3
    movdqu [dstpq + dst_strideq * 2], m4
    movdqu [dstpq + dst_strideq * 2 + 16], m5
    movdqu [dstpq + dst_stride3q], m6
    movdqu [dstpq + dst_stride3q + 16], m7
%endmacro


INIT_XMM
cglobal Copy32x8_sse2, 4, 6, 8, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    RET


INIT_XMM
cglobal Copy32x16_sse2, 4, 6, 8, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    RET


INIT_XMM
cglobal Copy32x32_sse2, 4, 6, 8, dstp, dst_stride, srcp, src_stride, dst_stride3, src_stride3
    lea dst_stride3q, [dst_strideq * 3] ; yasm is smart and turns it into [dst_strideq + dst_strideq * 2]
    lea src_stride3q, [src_strideq * 3]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    lea srcpq, [srcpq + src_strideq * 4]
    lea dstpq, [dstpq + dst_strideq * 4]

    MOV32x4

    RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
