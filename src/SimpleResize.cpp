// This used to contain code from the SimpleResize Avisynth plugin, written
// by Tom Barry and modified by Fizick. All of that was rewritten by dubhater,
// using code by anon32 for, ahem, inspiration.
// Only the name and the basic algorithm remain.

#include <algorithm>

#include <cstdlib>

#include "SimpleResize.h"


SimpleResize::SimpleResize(int _dst_width, int _dst_height, int _src_width, int _src_height) {
    src_width = _src_width;
    src_height = _src_height;
    dst_width = _dst_width;
    dst_height = _dst_height;

    // Offset to first line of the pair.
    vertical_offsets = (int *)malloc(dst_height * sizeof(int));
    // Weight of the second line of the pair.
    vertical_weights = (int *)malloc(dst_height * sizeof(int));

    horizontal_offsets = (int *)malloc(dst_width * sizeof(int));
    horizontal_weights = (int *)malloc(dst_width * sizeof(int));

    InitTables(horizontal_offsets, horizontal_weights, dst_width, src_width);
    InitTables(vertical_offsets, vertical_weights, dst_height, src_height);
}


SimpleResize::~SimpleResize() {
    free(vertical_offsets);
    free(vertical_weights);
    free(horizontal_offsets);
    free(horizontal_weights);
}


// Thread-safe.
void SimpleResize::Resize(uint8_t *dstp, int dst_stride, const uint8_t* srcp, int src_stride) {
    const uint8_t *srcp1;
    const uint8_t *srcp2;

    uint8_t *workp = (uint8_t *)malloc(src_width);

    for (int y = 0; y < dst_height; y++) {
        int weight_bottom = vertical_weights[y];
        int weight_top = 32768 - weight_bottom;

        srcp1 = srcp + vertical_offsets[y] * src_stride;
        srcp2 = srcp1 + src_stride;

        // vertical
        for (int x = 0; x < src_width; x++) {
            workp[x] = (srcp1[x] * weight_top + srcp2[x] * weight_bottom + 16384) / 32768;
        }

        // horizontal
        for (int x = 0; x < dst_width; x++) {
            int weight_right = horizontal_weights[x];
            int weight_left = 32768 - weight_right;
            int offset = horizontal_offsets[x];

            dstp[x] = (workp[offset] * weight_left + workp[offset + 1] * weight_right + 16384) / 32768;
        }

        dstp += dst_stride;
    }

    free(workp);
}


void SimpleResize::InitTables(int *offsets, int *weights, int out, int in) {
    // We don't do shifts.
    float leftmost = 0.5f; // + shift
    float rightmost = in - 0.5f; // + shift

    int leftmost_idx = std::max((int)leftmost, 0);
    int rightmost_idx = std::min((int)rightmost, in - 1);

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
