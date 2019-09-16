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
#else
#define DEGRAIN_AVX2(radius, width, height)
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
    __m256i wrefs0 = _mm256_set1_epi16(WRefs[0]);
    __m256i wrefs1 = _mm256_set1_epi16(WRefs[1]);
    __m256i wrefs2 = _mm256_set1_epi16(WRefs[2]);
    __m256i wrefs3 = _mm256_set1_epi16(WRefs[3]);
    __m256i wrefs4 = _mm256_set1_epi16(WRefs[4]);
    __m256i wrefs5 = _mm256_set1_epi16(WRefs[5]);
    __m256i src, refs0, refs1, refs2, refs3, refs4, refs5;

    const uint8_t *pRefs0 = pRefs[0];
    const uint8_t *pRefs1 = pRefs[1];
    const uint8_t *pRefs2 = pRefs[2];
    const uint8_t *pRefs3 = pRefs[3];
    const uint8_t *pRefs4 = pRefs[4];
    const uint8_t *pRefs5 = pRefs[5];

    int pitchMul = blockWidth == 8 ? 2 : 1;
    int nRefPitches0 = nRefPitches[0];
    int nRefPitches1 = nRefPitches[1];
    int nRefPitches2 = nRefPitches[2];
    int nRefPitches3 = nRefPitches[3];
    int nRefPitches4 = nRefPitches[4];
    int nRefPitches5 = nRefPitches[5];

    for (int y = 0; y < blockHeight; y += pitchMul) {
        for (int x = 0; x < blockWidth; x += 16 / pitchMul) {
            if (blockWidth == 8) {
                src = _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pSrc + x)), _mm_loadl_epi64((const __m128i *)(pSrc + nSrcPitch + x))));
                refs0 = _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs0 + x)), _mm_loadl_epi64((const __m128i *)(pRefs0 + nRefPitches0 + x))));
                refs1 = _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs1 + x)), _mm_loadl_epi64((const __m128i *)(pRefs1 + nRefPitches1 + x))));
                refs2 = radius > 1 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs2 + x)), _mm_loadl_epi64((const __m128i *)(pRefs2 + nRefPitches2 + x)))) : zero;
                refs3 = radius > 1 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs3 + x)), _mm_loadl_epi64((const __m128i *)(pRefs3 + nRefPitches3 + x)))) : zero;
                refs4 = radius > 2 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs4 + x)), _mm_loadl_epi64((const __m128i *)(pRefs4 + nRefPitches4 + x)))) : zero;
                refs5 = radius > 2 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs5 + x)), _mm_loadl_epi64((const __m128i *)(pRefs5 + nRefPitches5 + x)))) : zero;
            } else {
                src = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pSrc + x)));
                refs0 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs0 + x)));
                refs1 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs1 + x)));
                refs2 = radius > 1 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs2 + x))) : zero;
                refs3 = radius > 1 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs3 + x))) : zero;
                refs4 = radius > 2 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs4 + x))) : zero;
                refs5 = radius > 2 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs5 + x))) : zero;
            }

            src = _mm256_mullo_epi16(src, wsrc);
            refs0 = _mm256_mullo_epi16(refs0, wrefs0);
            refs1 = _mm256_mullo_epi16(refs1, wrefs1);
            refs2 = radius > 1 ? _mm256_mullo_epi16(refs2, wrefs2) : refs2;
            refs3 = radius > 1 ? _mm256_mullo_epi16(refs3, wrefs3) : refs3;
            refs4 = radius > 2 ? _mm256_mullo_epi16(refs4, wrefs4) : refs4;
            refs5 = radius > 2 ? _mm256_mullo_epi16(refs5, wrefs5) : refs5;

            __m256i accum = _mm256_set1_epi16(128);

            accum = _mm256_add_epi16(accum, src);
            accum = _mm256_add_epi16(accum, refs0);
            accum = _mm256_add_epi16(accum, refs1);
            accum = radius > 1 ? _mm256_add_epi16(accum, refs2) : accum;
            accum = radius > 1 ? _mm256_add_epi16(accum, refs3) : accum;
            accum = radius > 2 ? _mm256_add_epi16(accum, refs4) : accum;
            accum = radius > 2 ? _mm256_add_epi16(accum, refs5) : accum;

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
        pRefs0 += nRefPitches0 * pitchMul;
        pRefs1 += nRefPitches1 * pitchMul;
        pRefs2 += radius > 1 ? nRefPitches2 * pitchMul : 0;
        pRefs3 += radius > 1 ? nRefPitches3 * pitchMul : 0;
        pRefs4 += radius > 2 ? nRefPitches4 * pitchMul : 0;
        pRefs5 += radius > 2 ? nRefPitches5 * pitchMul : 0;
    }
}
#endif

static const std::unordered_map<uint32_t, DenoiseFunction> degrain_functions[3] = {
    {
        DEGRAIN_AVX2(1, 8, 2)
        DEGRAIN_AVX2(1, 8, 4)
        DEGRAIN_AVX2(1, 8, 8)
        DEGRAIN_AVX2(1, 8, 16)
        DEGRAIN_AVX2(1, 16, 1)
        DEGRAIN_AVX2(1, 16, 2)
        DEGRAIN_AVX2(1, 16, 4)
        DEGRAIN_AVX2(1, 16, 8)
        DEGRAIN_AVX2(1, 16, 16)
        DEGRAIN_AVX2(1, 16, 32)
        DEGRAIN_AVX2(1, 32, 8)
        DEGRAIN_AVX2(1, 32, 16)
        DEGRAIN_AVX2(1, 32, 32)
        DEGRAIN_AVX2(1, 32, 64)
        DEGRAIN_AVX2(1, 64, 16)
        DEGRAIN_AVX2(1, 64, 32)
        DEGRAIN_AVX2(1, 64, 64)
        DEGRAIN_AVX2(1, 64, 128)
        DEGRAIN_AVX2(1, 128, 32)
        DEGRAIN_AVX2(1, 128, 64)
        DEGRAIN_AVX2(1, 128, 128)
    },
    {
        DEGRAIN_AVX2(2, 8, 2)
        DEGRAIN_AVX2(2, 8, 4)
        DEGRAIN_AVX2(2, 8, 8)
        DEGRAIN_AVX2(2, 8, 16)
        DEGRAIN_AVX2(2, 16, 1)
        DEGRAIN_AVX2(2, 16, 2)
        DEGRAIN_AVX2(2, 16, 4)
        DEGRAIN_AVX2(2, 16, 8)
        DEGRAIN_AVX2(2, 16, 16)
        DEGRAIN_AVX2(2, 16, 32)
        DEGRAIN_AVX2(2, 32, 8)
        DEGRAIN_AVX2(2, 32, 16)
        DEGRAIN_AVX2(2, 32, 32)
        DEGRAIN_AVX2(2, 32, 64)
        DEGRAIN_AVX2(2, 64, 16)
        DEGRAIN_AVX2(2, 64, 32)
        DEGRAIN_AVX2(2, 64, 64)
        DEGRAIN_AVX2(2, 64, 128)
        DEGRAIN_AVX2(2, 128, 32)
        DEGRAIN_AVX2(2, 128, 64)
        DEGRAIN_AVX2(2, 128, 128)
    },
    {
        DEGRAIN_AVX2(3, 8, 2)
        DEGRAIN_AVX2(3, 8, 4)
        DEGRAIN_AVX2(3, 8, 8)
        DEGRAIN_AVX2(3, 8, 16)
        DEGRAIN_AVX2(3, 16, 1)
        DEGRAIN_AVX2(3, 16, 2)
        DEGRAIN_AVX2(3, 16, 4)
        DEGRAIN_AVX2(3, 16, 8)
        DEGRAIN_AVX2(3, 16, 16)
        DEGRAIN_AVX2(3, 16, 32)
        DEGRAIN_AVX2(3, 32, 8)
        DEGRAIN_AVX2(3, 32, 16)
        DEGRAIN_AVX2(3, 32, 32)
        DEGRAIN_AVX2(3, 32, 64)
        DEGRAIN_AVX2(3, 64, 16)
        DEGRAIN_AVX2(3, 64, 32)
        DEGRAIN_AVX2(3, 64, 64)
        DEGRAIN_AVX2(3, 64, 128)
        DEGRAIN_AVX2(3, 128, 32)
        DEGRAIN_AVX2(3, 128, 64)
        DEGRAIN_AVX2(3, 128, 128)
    }
};

DenoiseFunction selectDegrainFunctionAVX2(unsigned radius, unsigned width, unsigned height, unsigned bits) {
    try {
        return degrain_functions[radius - 1].at(KEY(width, height, bits, AVX2));
    } catch (std::out_of_range &) {
        return nullptr;
    }
}
