// This used to contain code from the SimpleResize Avisynth plugin, written
// by Tom Barry and modified by Fizick. All of that was rewritten by dubhater,
// using code by anon32 for, ahem, inspiration.
// Only the name and the basic algorithm remain.

#include <stdlib.h>

#include <VSHelper.h>

#include "SimpleResize.h"


static void InitTables(int *offsets, int *weights, int out, int in) {
    // We don't do shifts.
    float leftmost = 0.5f;       // + shift
    float rightmost = in - 0.5f; // + shift

    int leftmost_idx = VSMAX((int)leftmost, 0);
    int rightmost_idx = VSMIN((int)rightmost, in - 1);

    for (int i = 0; i < out; i++) {
        float position = (i + 0.5f) * (float)in / (float)out;

        float weight;
        int offset;

        if (position <= leftmost) {
            offset = leftmost_idx;
            weight = 0.0f;
        } else if (position >= rightmost) {
            offset = rightmost_idx - 1;
            weight = 1.0f;
        } else {
            offset = (int)(position - leftmost);
            weight = position - leftmost - offset;
        }

        offsets[i] = offset;

        weights[i] = (int)(weight * 32768);
    }
}


void simpleInit(SimpleResize *simple, int dst_width, int dst_height, int src_width, int src_height) {
    simple->src_width = src_width;
    simple->src_height = src_height;
    simple->dst_width = dst_width;
    simple->dst_height = dst_height;

    // Offset to first line of the pair.
    simple->vertical_offsets = (int *)malloc(dst_height * sizeof(int));
    // Weight of the second line of the pair.
    simple->vertical_weights = (int *)malloc(dst_height * sizeof(int));

    simple->horizontal_offsets = (int *)malloc(dst_width * sizeof(int));
    simple->horizontal_weights = (int *)malloc(dst_width * sizeof(int));

    InitTables(simple->horizontal_offsets, simple->horizontal_weights, dst_width, src_width);
    InitTables(simple->vertical_offsets, simple->vertical_weights, dst_height, src_height);
}


void simpleDeinit(SimpleResize *simple) {
    free(simple->vertical_offsets);
    free(simple->vertical_weights);
    free(simple->horizontal_offsets);
    free(simple->horizontal_weights);
    memset(simple, 0, sizeof(SimpleResize));
}


// Thread-safe.
#define simpleResize(PixelType) \
void simpleResize_##PixelType(const SimpleResize *simple, PixelType *dstp, int dst_stride, const PixelType *srcp, int src_stride) { \
    const PixelType *srcp1; \
    const PixelType *srcp2; \
 \
    PixelType *workp = (PixelType *)malloc(simple->src_width * sizeof(PixelType)); \
 \
    for (int y = 0; y < simple->dst_height; y++) { \
        int weight_bottom = simple->vertical_weights[y]; \
        int weight_top = 32768 - weight_bottom; \
 \
        srcp1 = srcp + simple->vertical_offsets[y] * src_stride; \
        srcp2 = srcp1 + src_stride; \
 \
        /* vertical */ \
        for (int x = 0; x < simple->src_width; x++) { \
            workp[x] = (srcp1[x] * weight_top + srcp2[x] * weight_bottom + 16384) / 32768; \
        } \
 \
        /* horizontal */ \
        for (int x = 0; x < simple->dst_width; x++) { \
            int weight_right = simple->horizontal_weights[x]; \
            int weight_left = 32768 - weight_right; \
            int offset = simple->horizontal_offsets[x]; \
 \
            dstp[x] = (workp[offset] * weight_left + workp[offset + 1] * weight_right + 16384) / 32768; \
        } \
 \
        dstp += dst_stride; \
    } \
 \
    free(workp); \
}


simpleResize(uint8_t)
simpleResize(int16_t)
