// This used to contain code from the SimpleResize Avisynth plugin, written
// by Tom Barry and modified by Fizick. All of that was rewritten by dubhater.
// Only the name and the basic algorithm remain.


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

    InitTables();
};


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
        int weight_top = 256 - weight_bottom;

        srcp1 = srcp + vertical_offsets[y] * src_stride;
        srcp2 = srcp1 + src_stride;

        // vertical
        for (int x = 0; x < src_width; x++) {
            workp[x] = (srcp1[x] * weight_top + srcp2[x] * weight_bottom + 128) / 256;
        }

        // horizontal
        for (int x = 0; x < dst_width; x++) {
            int weight_right = horizontal_weights[x];
            int weight_left = 256 - weight_right;
            int offset = horizontal_offsets[x];

            dstp[x] = (workp[offset] * weight_left + workp[offset + 1] * weight_right + 128) / 256;
        }

        dstp += dst_stride;
    }

    free(workp);
}


void SimpleResize::InitTables() {
    // Written with integer, even ratios in mind, but the clamping to
    // src_width-2/src_height-2 makes it work (badly?) with fractional
    // ratios too.

    int ratio = dst_height / src_height;

    // Distance between adjacent dst lines
    float dst_distance = 1.0f / ratio;
    // Distance from any src line to the nearest dst line - for even ratios
    // only. Would be zero for odd ratios.
    float initial_distance = dst_distance / 2.0f;

    // Distance between adjacent src lines.
    //float src_distance = 1; // unused

    // Edge pixels that are left with only one src line, thus can't be
    // interpolated like the rest.
    int edge = ratio / 2;

    for (int y = 0; y < dst_height - ratio; y++) {
        int offset = y / ratio;
        if (offset > src_height - 2)
            offset = src_height - 2;

        vertical_offsets[y + edge] = offset;

        float weight = initial_distance + (y % ratio) * dst_distance;

        // For a maximum ratio of 32 (the maximum block size), 256 is sufficient.
        // Nothing is lost in the conversion to int.
        vertical_weights[y + edge] = (int)(weight * 256);
    }

    // The edges.
    for (int y = 0; y < edge; y++) {
        vertical_offsets[y] = vertical_offsets[edge];
        vertical_weights[y] = vertical_weights[edge];
    }

    for (int y = dst_height - edge; y < dst_height; y++) {
        vertical_offsets[y] = vertical_offsets[dst_height - edge - 1];
        vertical_weights[y] = vertical_weights[dst_height - edge - 1];
    }


    ratio = dst_width / src_width;

    // Distance between adjacent dst columns
    dst_distance = 1.0f / ratio;
    // Distance from any src column to the nearest dst column - for even ratios
    // only. Would be zero for odd ratios.
    initial_distance = dst_distance / 2.0f;

    // Distance between adjacent src columns.
    //float src_distance = 1; // unused

    // Edge pixels that are left with only one src column, thus can't be
    // interpolated like the rest.
    edge = ratio / 2;

    for (int x = 0; x < dst_width - ratio; x++) {
        int offset = x / ratio;
        if (offset > src_width - 2)
            offset = src_width - 2;

        horizontal_offsets[x + edge] = offset;

        float weight = initial_distance + (x % ratio) * dst_distance;

        // For a maximum ratio of 32 (the maximum block size), 256 is sufficient.
        // Nothing is lost in the conversion to int.
        horizontal_weights[x + edge] = (int)(weight * 256);
    }

    // The edges.
    for (int x = 0; x < edge; x++) {
        horizontal_offsets[x] = horizontal_offsets[edge];
        horizontal_weights[x] = horizontal_weights[edge];
    }

    for (int x = dst_width - edge; x < dst_width; x++) {
        horizontal_offsets[x] = horizontal_offsets[dst_width - edge - 1];
        horizontal_weights[x] = horizontal_weights[dst_width - edge - 1];
    }
}
