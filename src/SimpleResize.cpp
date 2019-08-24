// This used to contain code from the SimpleResize Avisynth plugin, written
// by Tom Barry and modified by Fizick. All of that was rewritten by dubhater,
// using code by anon32 for, ahem, inspiration.
// Only the name and the basic algorithm remain.

#include <algorithm>
#include <stdlib.h>

#include <VSHelper.h>

#include "CPU.h"
#include "SimpleResize.h"


#if defined(MVTOOLS_X86)
void simpleResize_uint8_t_avx2(const SimpleResize *simple,
                               uint8_t *dstp, int dst_stride,
                               const uint8_t *srcp, int src_stride,
                               int horizontal_vectors);
void simpleResize_int16_t_avx2(const SimpleResize *simple,
                               int16_t *dstp, int dst_stride,
                               const int16_t *srcp, int src_stride,
                               int horizontal_vectors);
#endif


extern "C" uint32_t g_cpuinfo;


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

        weights[i] = (int)(weight * simple_resize_weight_max);
    }
}


// Thread-safe.
template <typename PixelType>
static void simpleResize(const SimpleResize *simple,
                         PixelType *dstp, int dst_stride,
                         const PixelType *srcp, int src_stride,
                         int horizontal_vectors) {

    // Apparently only 16 bit vectors need limiting.
    bool limit_vectors = sizeof(PixelType) == 2;

    int pel = simple->pel;
    int minimum = 0;
    int maximum = simple->limit_height * pel - 1;
    int horizontal_step = horizontal_vectors ? pel : 0;
    int vertical_step = horizontal_vectors ? 0 : pel;

    PixelType *workp = (PixelType *)malloc(simple->src_width * sizeof(PixelType));

    for (int y = 0; y < simple->dst_height; y++) {
        int weight_bottom = simple->vertical_weights[y];
        int weight_top = simple_resize_weight_max - weight_bottom;

        const PixelType *srcp1 = srcp + simple->vertical_offsets[y] * src_stride;
        const PixelType *srcp2 = srcp1 + src_stride;

        /* vertical */
        for (int x = 0; x < simple->src_width; x++) {
            workp[x] = (srcp1[x] * weight_top + srcp2[x] * weight_bottom + simple_resize_weight_half) >> simple_resize_weight_shift;
        }

        if (horizontal_vectors) {
            minimum = 0;
            maximum = simple->limit_width * pel - 1;
        }

        /* horizontal */
        for (int x = 0; x < simple->dst_width; x++) {
            int weight_right = simple->horizontal_weights[x];
            int weight_left = simple_resize_weight_max - weight_right;
            int offset = simple->horizontal_offsets[x];

            int result = (workp[offset] * weight_left + workp[offset + 1] * weight_right + simple_resize_weight_half) >> simple_resize_weight_shift;

            if (limit_vectors) {
                result = std::max(minimum, std::min(result, maximum));

                minimum -= horizontal_step;
                maximum -= horizontal_step;
            }

            dstp[x] = result;
        }

        dstp += dst_stride;

        if (limit_vectors) {
            minimum -= vertical_step;
            maximum -= vertical_step;
        }
    }

    free(workp);
}


void simpleInit(SimpleResize *simple, int dst_width, int dst_height, int src_width, int src_height, int limit_width, int limit_height, int pel, int opt) {
    simple->src_width = src_width;
    simple->src_height = src_height;
    simple->dst_width = dst_width;
    simple->dst_height = dst_height;

    simple->limit_width = limit_width;
    simple->limit_height = limit_height;
    simple->pel = pel;

    // Offset to first line of the pair.
    simple->vertical_offsets = (int *)malloc(dst_height * sizeof(int));
    // Weight of the second line of the pair.
    simple->vertical_weights = (int *)malloc(dst_height * sizeof(int));

    simple->horizontal_offsets = (int *)malloc(dst_width * sizeof(int));
    simple->horizontal_weights = (int *)malloc(dst_width * sizeof(int));

    InitTables(simple->horizontal_offsets, simple->horizontal_weights, dst_width, src_width);
    InitTables(simple->vertical_offsets, simple->vertical_weights, dst_height, src_height);

    simple->simpleResize_uint8_t = simpleResize<uint8_t>;
    simple->simpleResize_int16_t = simpleResize<int16_t>;

    if (opt) {
#if defined(MVTOOLS_X86)
        if (g_cpuinfo & X264_CPU_AVX2) {
            simple->simpleResize_uint8_t = simpleResize_uint8_t_avx2;
            simple->simpleResize_int16_t = simpleResize_int16_t_avx2;

            for (int i = 0; i < dst_width; i++) {
                int w = simple->horizontal_weights[i];
                simple->horizontal_weights[i] = (w << 16) | (simple_resize_weight_max - w);
            }
        }
#endif
    }
}


void simpleDeinit(SimpleResize *simple) {
    free(simple->vertical_offsets);
    free(simple->vertical_weights);
    free(simple->horizontal_offsets);
    free(simple->horizontal_weights);
    memset(simple, 0, sizeof(SimpleResize));
}

