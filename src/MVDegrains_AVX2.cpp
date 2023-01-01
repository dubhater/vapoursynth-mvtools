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
    __m256i wrefs0 = _mm256_set1_epi16(WRefs[0]);
    __m256i wrefs1 = _mm256_set1_epi16(WRefs[1]);
    __m256i wrefs2 = _mm256_set1_epi16(WRefs[2]);
    __m256i wrefs3 = _mm256_set1_epi16(WRefs[3]);
    __m256i wrefs4 = _mm256_set1_epi16(WRefs[4]);
    __m256i wrefs5 = _mm256_set1_epi16(WRefs[5]);
    __m256i wrefs6 = _mm256_set1_epi16(WRefs[6]);
    __m256i wrefs7 = _mm256_set1_epi16(WRefs[7]);
    __m256i wrefs8 = _mm256_set1_epi16(WRefs[8]);
    __m256i wrefs9 = _mm256_set1_epi16(WRefs[9]);
    __m256i wrefs10 = _mm256_set1_epi16(WRefs[10]);
    __m256i wrefs11 = _mm256_set1_epi16(WRefs[11]);
    __m256i src, refs0, refs1, refs2, refs3, refs4, refs5, refs6, refs7, refs8, refs9, refs10, refs11;

    const uint8_t *pRefs0 = pRefs[0];
    const uint8_t *pRefs1 = pRefs[1];
    const uint8_t *pRefs2 = pRefs[2];
    const uint8_t *pRefs3 = pRefs[3];
    const uint8_t *pRefs4 = pRefs[4];
    const uint8_t *pRefs5 = pRefs[5];
    const uint8_t *pRefs6 = pRefs[6];
    const uint8_t *pRefs7 = pRefs[7];
    const uint8_t *pRefs8 = pRefs[8];
    const uint8_t *pRefs9 = pRefs[9];
    const uint8_t *pRefs10 = pRefs[10];
    const uint8_t *pRefs11 = pRefs[11];

    int pitchMul = blockWidth == 8 ? 2 : 1;
    int nRefPitches0 = nRefPitches[0];
    int nRefPitches1 = nRefPitches[1];
    int nRefPitches2 = nRefPitches[2];
    int nRefPitches3 = nRefPitches[3];
    int nRefPitches4 = nRefPitches[4];
    int nRefPitches5 = nRefPitches[5];
    int nRefPitches6 = nRefPitches[6];
    int nRefPitches7 = nRefPitches[7];
    int nRefPitches8 = nRefPitches[8];
    int nRefPitches9 = nRefPitches[9];
    int nRefPitches10 = nRefPitches[10];
    int nRefPitches11 = nRefPitches[11];

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
                refs6 = radius > 3 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs6 + x)), _mm_loadl_epi64((const __m128i *)(pRefs6 + nRefPitches6 + x)))) : zero;
                refs7 = radius > 3 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs7 + x)), _mm_loadl_epi64((const __m128i *)(pRefs7 + nRefPitches7 + x)))) : zero;
                refs8 = radius > 4 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs8 + x)), _mm_loadl_epi64((const __m128i *)(pRefs8 + nRefPitches8 + x)))) : zero;
                refs9 = radius > 4 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs9 + x)), _mm_loadl_epi64((const __m128i *)(pRefs9 + nRefPitches9 + x)))) : zero;
                refs10 = radius > 5 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs10 + x)), _mm_loadl_epi64((const __m128i *)(pRefs10 + nRefPitches10 + x)))) : zero;
                refs11 = radius > 5 ? _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pRefs11 + x)), _mm_loadl_epi64((const __m128i *)(pRefs11 + nRefPitches11 + x)))) : zero;
            } else {
                src = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pSrc + x)));
                refs0 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs0 + x)));
                refs1 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs1 + x)));
                refs2 = radius > 1 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs2 + x))) : zero;
                refs3 = radius > 1 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs3 + x))) : zero;
                refs4 = radius > 2 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs4 + x))) : zero;
                refs5 = radius > 2 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs5 + x))) : zero;
                refs6 = radius > 3 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs6 + x))) : zero;
                refs7 = radius > 3 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs7 + x))) : zero;
                refs8 = radius > 4 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs8 + x))) : zero;
                refs9 = radius > 4 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs9 + x))) : zero;
                refs10 = radius > 5 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs10 + x))) : zero;
                refs11 = radius > 5 ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pRefs11 + x))) : zero;
            }

            src = _mm256_mullo_epi16(src, wsrc);
            refs0 = _mm256_mullo_epi16(refs0, wrefs0);
            refs1 = _mm256_mullo_epi16(refs1, wrefs1);
            refs2 = radius > 1 ? _mm256_mullo_epi16(refs2, wrefs2) : refs2;
            refs3 = radius > 1 ? _mm256_mullo_epi16(refs3, wrefs3) : refs3;
            refs4 = radius > 2 ? _mm256_mullo_epi16(refs4, wrefs4) : refs4;
            refs5 = radius > 2 ? _mm256_mullo_epi16(refs5, wrefs5) : refs5;
            refs6 = radius > 3 ? _mm256_mullo_epi16(refs6, wrefs6) : refs6;
            refs7 = radius > 3 ? _mm256_mullo_epi16(refs7, wrefs7) : refs7;
            refs8 = radius > 4 ? _mm256_mullo_epi16(refs8, wrefs8) : refs8;
            refs9 = radius > 4 ? _mm256_mullo_epi16(refs9, wrefs9) : refs9;
            refs10 = radius > 5 ? _mm256_mullo_epi16(refs10, wrefs10) : refs10;
            refs11 = radius > 5 ? _mm256_mullo_epi16(refs11, wrefs11) : refs11;

            __m256i accum = _mm256_set1_epi16(128);

            accum = _mm256_add_epi16(accum, src);
            accum = _mm256_add_epi16(accum, refs0);
            accum = _mm256_add_epi16(accum, refs1);
            accum = radius > 1 ? _mm256_add_epi16(accum, refs2) : accum;
            accum = radius > 1 ? _mm256_add_epi16(accum, refs3) : accum;
            accum = radius > 2 ? _mm256_add_epi16(accum, refs4) : accum;
            accum = radius > 2 ? _mm256_add_epi16(accum, refs5) : accum;
            accum = radius > 3 ? _mm256_add_epi16(accum, refs6) : accum;
            accum = radius > 3 ? _mm256_add_epi16(accum, refs7) : accum;
            accum = radius > 4 ? _mm256_add_epi16(accum, refs8) : accum;
            accum = radius > 4 ? _mm256_add_epi16(accum, refs9) : accum;
            accum = radius > 5 ? _mm256_add_epi16(accum, refs10) : accum;
            accum = radius > 5 ? _mm256_add_epi16(accum, refs11) : accum;

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
        pRefs6 += radius > 3 ? nRefPitches6 * pitchMul : 0;
        pRefs7 += radius > 3 ? nRefPitches7 * pitchMul : 0;
        pRefs8 += radius > 4 ? nRefPitches8 * pitchMul : 0;
        pRefs9 += radius > 4 ? nRefPitches9 * pitchMul : 0;
        pRefs10 += radius > 5 ? nRefPitches10 * pitchMul : 0;
        pRefs11 += radius > 5 ? nRefPitches11 * pitchMul : 0;
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
