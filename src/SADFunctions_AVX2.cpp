#if defined(MVTOOLS_X86)

#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <immintrin.h>
#include "SADFunctions.h"

#define zeroes _mm256_setzero_si256()


// This version used for width >= 32.
template <unsigned width, unsigned height>
struct SADWrapperU8_AVX2 {
    static_assert(width >= 32, "");

    static unsigned int sad_u8_avx2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m256i sum = zeroes;

        for (unsigned y = 0; y < height; y++) {
            for (unsigned x = 0; x < width; x += 32) {
                __m256i m2 = _mm256_loadu_si256((const __m256i *)&pSrc[x]);
                __m256i m3 = _mm256_loadu_si256((const __m256i *)&pRef[x]);

                __m256i diff = _mm256_sad_epu8(m2, m3);

                sum = _mm256_add_epi64(sum, diff);
            }

            pSrc += /*nSrcPitch*/ width;
            pRef += nRefPitch;
        }

        sum = _mm256_add_epi64(sum, _mm256_permute4x64_epi64(sum, _MM_SHUFFLE(0, 0, 3, 2)));
        sum = _mm256_add_epi64(sum, _mm256_shuffle_epi32(sum, _MM_SHUFFLE(0, 0, 3, 2)));
        return (unsigned)_mm_cvtsi128_si32(_mm256_castsi256_si128(sum));
    }

};

template <unsigned height>
struct SADWrapperU8_AVX2<16, height> {
    static_assert(height >= 2, "");

    static unsigned int sad_u8_avx2(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch) {
        __m256i sum = zeroes;

        for (int y = 0; y < height; y += 2) {
            __m256i m2 = _mm256_loadu_si256((const __m256i *)(pSrc + y * 16));
            __m256i m3 = _mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)(pRef + y * nRefPitch)));
            m3 = _mm256_insertf128_si256(m3, _mm_loadu_si128((const __m128i *)(pRef + (y + 1) * nRefPitch)), 1);

            __m256i diff = _mm256_sad_epu8(m2, m3);
            sum = _mm256_add_epi64(sum, diff);
        }

        sum = _mm256_add_epi64(sum, _mm256_permute4x64_epi64(sum, _MM_SHUFFLE(0, 0, 3, 2)));
        sum = _mm256_add_epi64(sum, _mm256_shuffle_epi32(sum, _MM_SHUFFLE(0, 0, 3, 2)));
        return (unsigned)_mm_cvtsi128_si32(_mm256_castsi256_si128(sum));
    }
};


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)


#define SAD_U8_AVX2(width, height) \
    { KEY(width, height, 8, 0), SADWrapperU8_AVX2<width, height>::sad_u8_avx2 },

static const std::unordered_map<uint32_t, SADFunction> sad_functions = {
    SAD_U8_AVX2(16, 2)
    SAD_U8_AVX2(16, 4)
    SAD_U8_AVX2(16, 4)
    SAD_U8_AVX2(16, 8)
    SAD_U8_AVX2(16, 16)
    SAD_U8_AVX2(16, 32)
    SAD_U8_AVX2(32, 8)
    SAD_U8_AVX2(32, 16)
    SAD_U8_AVX2(32, 32)
    SAD_U8_AVX2(32, 64)
    SAD_U8_AVX2(64, 16)
    SAD_U8_AVX2(64, 32)
    SAD_U8_AVX2(64, 64)
    SAD_U8_AVX2(64, 128)
    SAD_U8_AVX2(128, 32)
    SAD_U8_AVX2(128, 64)
    SAD_U8_AVX2(128, 128)
};

SADFunction selectSADFunctionAVX2(unsigned width, unsigned height, unsigned bits) {
    try {
        return sad_functions.at(KEY(width, height, bits, 0));
    } catch (const std::out_of_range &) {
        return nullptr;
    }
}

#endif