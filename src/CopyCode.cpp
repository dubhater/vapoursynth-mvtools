#include <cstdint>
#include <cstring>
#include <unordered_map>

#include "CopyCode.h"
#include "CPU.h"

template <unsigned width, unsigned height>
void copyBlock(uint8_t * __restrict pDst, intptr_t nDstPitch, const uint8_t * __restrict pSrc, intptr_t nSrcPitch) {
    int unroll = (height >= 8 ? 8 : (height >= 4 ? 4 : (height >= 2 ? 2 : 1))) / ((width + 15) / 16);
    unroll = unroll < 1 ? 1 : unroll;

    nDstPitch = width;

    for (unsigned j = 0; j < height; j += unroll) {
        memcpy(pDst + 0 * nDstPitch, pSrc + 0 * nSrcPitch, width);
        if (unroll > 1) {
            memcpy(pDst + 1 * nDstPitch, pSrc + 1 * nSrcPitch, width);
        }
        if (unroll > 2) {
            memcpy(pDst + 2 * nDstPitch, pSrc + 2 * nSrcPitch, width);
            memcpy(pDst + 3 * nDstPitch, pSrc + 3 * nSrcPitch, width);
        }
        if (unroll > 4) {
            memcpy(pDst + 4 * nDstPitch, pSrc + 4 * nSrcPitch, width);
            memcpy(pDst + 5 * nDstPitch, pSrc + 5 * nSrcPitch, width);
            memcpy(pDst + 6 * nDstPitch, pSrc + 6 * nSrcPitch, width);
            memcpy(pDst + 7 * nDstPitch, pSrc + 7 * nSrcPitch, width);
        }
        pDst += nDstPitch * unroll;
        pSrc += nSrcPitch * unroll;
    }
}


#define KEY(width, height, bits) (width) << 16 | (height) << 8 | (bits)
#define COPY(width, height) \
    { KEY(width, height, 8), copyBlock<width * sizeof(uint8_t), height> }, \
    { KEY(width, height, 16), copyBlock<width * sizeof(uint16_t), height> },

static const std::unordered_map<uint32_t, COPYFunction> copy_functions = {
    COPY(2, 2)
    COPY(2, 4)
    COPY(4, 2)
    COPY(4, 4)
    COPY(4, 8)
    COPY(8, 1)
    COPY(8, 2)
    COPY(8, 4)
    COPY(8, 8)
    COPY(8, 16)
    COPY(16, 1)
    COPY(16, 2)
    COPY(16, 4)
    COPY(16, 8)
    COPY(16, 16)
    COPY(16, 32)
    COPY(32, 8)
    COPY(32, 16)
    COPY(32, 32)
    COPY(32, 64)
    COPY(64, 16)
    COPY(64, 32)
    COPY(64, 64)
    COPY(64, 128)
    COPY(128, 32)
    COPY(128, 64)
    COPY(128, 128)
};

COPYFunction selectCopyFunction(unsigned width, unsigned height, unsigned bits) {
    return copy_functions.at(KEY(width, height, bits));
}

#undef COPY
#undef KEY

