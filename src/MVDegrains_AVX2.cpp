#include <stdexcept>
#include <unordered_map>

#include "MVDegrains.h"

enum InstructionSets {
    Scalar,
    SSE2,
    AVX2,
};

// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86)
#define DEGRAIN_AVX2(radius, width, height) \
    { KEY(width, height, 8, AVX2), Degrain_avx2<radius, width, height> },

#define DEGRAIN_LEVEL_AVX2(radius) \
    {\
        DEGRAIN_AVX2(radius, 8, 2)\
        DEGRAIN_AVX2(radius, 8, 4)\
        DEGRAIN_AVX2(radius, 8, 8)\
        DEGRAIN_AVX2(radius, 8, 16)\
        DEGRAIN_AVX2(radius, 16, 1)\
        DEGRAIN_AVX2(radius, 16, 2)\
        DEGRAIN_AVX2(radius, 16, 4)\
        DEGRAIN_AVX2(radius, 16, 8)\
        DEGRAIN_AVX2(radius, 16, 16)\
        DEGRAIN_AVX2(radius, 16, 32)\
        DEGRAIN_AVX2(radius, 32, 8)\
        DEGRAIN_AVX2(radius, 32, 16)\
        DEGRAIN_AVX2(radius, 32, 32)\
        DEGRAIN_AVX2(radius, 32, 64)\
        DEGRAIN_AVX2(radius, 64, 16)\
        DEGRAIN_AVX2(radius, 64, 32)\
        DEGRAIN_AVX2(radius, 64, 64)\
        DEGRAIN_AVX2(radius, 64, 128)\
        DEGRAIN_AVX2(radius, 128, 32)\
        DEGRAIN_AVX2(radius, 128, 64)\
        DEGRAIN_AVX2(radius, 128, 128)\
    }
#else
#define DEGRAIN_AVX2(radius, width, height)
#define DEGRAIN_LEVEL_AVX2(radius)
#endif


#if defined(MVTOOLS_X86)

#include <immintrin.h>

// XXX Moves the pointers passed in pRefs. This is okay because they are not
// used after this function is done with them.
template <int radius, int blockWidth, int blockHeight>
static void Degrain_avx2(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch, const uint8_t **pRefs, const int *nRefPitches, int WSrc, const int *WRefs) {
    static_assert(blockWidth >= 16 || (blockWidth == 8 && blockHeight >= 2), "");

    __m256i zero = _mm256_setzero_si256();
    __m256i wsrc = _mm256_set1_epi16(WSrc);

    __m256i wrefs[12];
    for(int i = 0; i < radius * 2; i += 2) {
        wrefs[i] = _mm256_set1_epi16(WRefs[i]);
        wrefs[i + 1] = _mm256_set1_epi16(WRefs[i + 1]);
    }
    __m256i src, accum, refs[12];

    int pitchMul = blockWidth == 8 ? 2 : 1;

    for (int y = 0; y < blockHeight; y += pitchMul) {
        for (int x = 0; x < blockWidth; x += 16 / pitchMul) {
            if (blockWidth == 8) {
                src = _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pSrc + x)), _mm_loadl_epi64((const __m128i *)(pSrc + nSrcPitch + x))));
                for(int i = 0; i < radius * 2; i += 2) {
                    refs[i] = _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs[i] + x)), _mm_loadl_epi64((const __m128i *)(pRefs[i] + nRefPitches[i] + x))));
                    refs[i + 1] = _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs[i + 1] + x)), _mm_loadl_epi64((const __m128i *)(pRefs[i + 1] + nRefPitches[i + 1] + x))));
                }
            } else {
                src = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pSrc + x)));
                for(int i = 0; i < radius * 2; i += 2) {
                    refs[i] = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs[i] + x)));
                    refs[i + 1] = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs[i + 1] + x)));
                }
            }

            src = _mm256_mullo_epi16(src, wsrc);
            for(int i = 0; i < radius * 2; i += 2) {
                refs[i] = _mm256_mullo_epi16(refs[i], wrefs[i]);
                refs[i + 1] = _mm256_mullo_epi16(refs[i + 1], wrefs[i + 1]);
            }

            accum = _mm256_set1_epi16(128);
            accum = _mm256_add_epi16(accum, src);

            for(int i = 0; i < radius * 2; i += 2) {
                accum = _mm256_add_epi16(accum, refs[i]);
                accum = _mm256_add_epi16(accum, refs[i + 1]);
            }
            accum = _mm256_srli_epi16(accum, 8);
            accum = _mm256_packus_epi16(accum, zero);

            if (blockWidth == 8) {
                _mm_storel_epi64((__m128i *)(pDst + x), _mm256_castsi256_si128(accum));
                _mm_storel_epi64((__m128i *)(pDst + nDstPitch + x), _mm256_extractf128_si256(accum, 1));
            } else {
                accum = _mm256_permute4x64_epi64(accum, _MM_SHUFFLE(0, 0, 2, 0));
                _mm_storeu_si128((__m128i *)(pDst + x), _mm256_castsi256_si128(accum));
            }
        }

        pDst += nDstPitch * pitchMul;
        pSrc += nSrcPitch * pitchMul;

        for(int i = 0; i < radius * 2; i += 2) {
            pRefs[i] += nRefPitches[i] * pitchMul;
            pRefs[i + 1] += nRefPitches[i + 1] * pitchMul;
        }
    }
}
#endif

static const std::unordered_map<uint32_t, DenoiseFunction> degrain_functions[6] = {
    DEGRAIN_LEVEL_AVX2(1),
    DEGRAIN_LEVEL_AVX2(2),
    DEGRAIN_LEVEL_AVX2(3),
    DEGRAIN_LEVEL_AVX2(4),
    DEGRAIN_LEVEL_AVX2(5),
    DEGRAIN_LEVEL_AVX2(6),
};

DenoiseFunction selectDegrainFunctionAVX2(unsigned radius, unsigned width, unsigned height, unsigned bits) {
    try {
        return degrain_functions[radius - 1].at(KEY(width, height, bits, AVX2));
    } catch (std::out_of_range &) {
        return nullptr;
    }
}
