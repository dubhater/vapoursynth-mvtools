%include "include/x86inc.asm"


;SECTION_RODATA


SECTION_TEXT


INIT_XMM
cglobal LimitChanges_sse2, 7, 7, 6, dstp, dst_stride, srcp, src_stride, width, height, limit
    movd m5, limitq ; r6
    punpcklbw m5, m5
    punpcklwd m5, m5
    punpckldq m5, m5
    punpcklqdq m5, m5

    pxor m4, m4 ; 0

.h_loopy:
    ; limit is no longer needed
    xor r6, r6

.h_loopx:
    ; srcp and dstp should be aligned
    movdqa m0, [srcpq + r6] ; src bytes
    movdqa m1, [dstpq + r6] ; dest bytes
    movdqa m2, m5        ;/* copy limit */
    paddusb m2, m0    ;/* max possible (m0 is original) */
    movdqa m3, m1    ;/* (m1 is changed) */
    psubusb m3, m0    ;/* changed - orig,   saturated to 0 */
    psubusb m3, m5    ;/* did it change too much, Y where nonzero */
    pcmpeqb m3, m4    ;/* now ff where new value should be used, else 00 (m4=0)*/
    pand m1, m3    ;    /* use new value for these pixels */
    pandn m3, m2    ; /* use max value for these pixels */
    por m1, m3        ;/* combine them, get result with limited  positive correction */
    movdqa m3, m5        ;/* copy limit */
    movdqa m2, m0        ;/* copy orig */
    psubusb m2, m5    ;/* min possible */
    movdqa m3, m0    ;/* copy orig */
    psubusb m3, m1    ;/* orig - changed, saturated to 0 */
    psubusb m3, m5    ;/* did it change too much, Y where nonzero */
    pcmpeqb m3, m4    ;/* now ff where new value should be used, else 00 */
    pand m1, m3        ;/* use new value for these pixels */
    pandn m3, m2        ;/* use min value for these pixels */
    por m1, m3        ;/* combine them, get result with limited  negative correction */

    movdqa [dstpq + r6], m1
    add r6, 16
    cmp r6, widthq
    jl .h_loopx

; do not process rightmost rest bytes

    add srcpq, src_strideq
    add dstpq, dst_strideq
    dec heightq
    jnz .h_loopy

    RET
