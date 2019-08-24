#if defined(MVTOOLS_X86)

#include <cstdint>
#include <immintrin.h>

#define zeroes _mm256_setzero_si256()

/* TODO: port these
   extern "C" void  VerticalBicubic_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nWidth, intptr_t nHeight);
   extern "C" void  HorizontalBicubic_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nWidth, intptr_t nHeight);
   extern "C" void  RB2F_iSSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nDstPitch,
   intptr_t nSrcPitch, intptr_t nWidth, intptr_t nHeight);
   extern "C" void  RB2FilteredVerticalLine_SSE(uint8_t *pDst, const uint8_t *pSrc, intptr_t nSrcPitch, intptr_t nWidthMMX);
   extern "C" void  RB2FilteredHorizontalInplaceLine_SSE(uint8_t *pSrc, intptr_t nWidthMMX);
   */

void Average2_avx2(uint8_t *pDst, const uint8_t *pSrc1, const uint8_t *pSrc2, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight) {
    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x += 32) {
            __m256i m0 = _mm256_loadu_si256((const __m256i *)&pSrc1[x]);
            __m256i m1 = _mm256_loadu_si256((const __m256i *)&pSrc2[x]);

            m0 = _mm256_avg_epu8(m0, m1);
            _mm256_storeu_si256((__m256i *)&pDst[x], m0);
        }

        pSrc1 += nPitch;
        pSrc2 += nPitch;
        pDst += nPitch;
    }
}


void VerticalBilinear_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                           intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight - 1; y++) {
        for (int x = 0; x < nWidth; x += 32) {
            __m256i m0 = _mm256_loadu_si256((const __m256i *)&pSrc[x]);
            __m256i m1 = _mm256_loadu_si256((const __m256i *)&pSrc[x + nPitch]);

            m0 = _mm256_avg_epu8(m0, m1);
            _mm256_storeu_si256((__m256i *)&pDst[x], m0);
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int x = 0; x < nWidth; x++)
        pDst[x] = pSrc[x];
}


void HorizontalBilinear_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                             intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x += 32) {
            __m256i m0 = _mm256_loadu_si256((const __m256i *)&pSrc[x]);
            __m256i m1 = _mm256_loadu_si256((const __m256i *)&pSrc[x + 1]);

            m0 = _mm256_avg_epu8(m0, m1);
            _mm256_storeu_si256((__m256i *)&pDst[x], m0);
        }

        pDst[nWidth - 1] = pSrc[nWidth - 1];

        pSrc += nPitch;
        pDst += nPitch;
    }
}


void DiagonalBilinear_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                           intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight - 1; y++) {
        for (int x = 0; x < nWidth; x += 16) {
            __m256i m0 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x]));
            __m256i m1 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + 1]));
            __m256i m2 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + nPitch]));
            __m256i m3 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + nPitch + 1]));

            m0 = _mm256_add_epi16(m0, m1);
            m2 = _mm256_add_epi16(m2, m3);
            m0 = _mm256_add_epi16(m0, _mm256_set1_epi16(2));
            m0 = _mm256_add_epi16(m0, m2);

            m0 = _mm256_srli_epi16(m0, 2);

            m0 = _mm256_packus_epi16(m0, m0);
            m0 = _mm256_permute4x64_epi64(m0, _MM_SHUFFLE(0, 0, 2, 0));
            _mm_storeu_si128((__m128i *)&pDst[x], _mm256_castsi256_si128(m0));
        }

        pDst[nWidth - 1] = (pSrc[nWidth - 1] + pSrc[nWidth - 1 + nPitch] + 1) >> 1;

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int x = 0; x < nWidth; x += 32) {
        __m256i m0 = _mm256_loadu_si256((const __m256i *)&pSrc[x]);
        __m256i m1 = _mm256_loadu_si256((const __m256i *)&pSrc[x + 1]);

        m0 = _mm256_avg_epu8(m0, m1);
        _mm256_storeu_si256((__m256i *)&pDst[x], m0);
    }

    pDst[nWidth - 1] = pSrc[nWidth - 1];
}

void VerticalWiener_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                         intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < 2; y++) {
        for (int x = 0; x < nWidth; x += 32) {
            __m256i m0 = _mm256_loadu_si256((const __m256i *)&pSrc[x]);
            __m256i m1 = _mm256_loadu_si256((const __m256i *)&pSrc[x + nPitch]);

            m0 = _mm256_avg_epu8(m0, m1);
            _mm256_storeu_si256((__m256i *)&pDst[x], m0);
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int y = 2; y < nHeight - 4; y++) {
        for (int x = 0; x < nWidth; x += 16) {
            __m256i m0 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x - nPitch * 2]));
            __m256i m1 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x - nPitch]));
            __m256i m2 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x]));
            __m256i m3 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + nPitch]));
            __m256i m4 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + nPitch * 2]));
            __m256i m5 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + nPitch * 3]));

            m2 = _mm256_add_epi16(m2, m3);
            m2 = _mm256_slli_epi16(m2, 2);

            m1 = _mm256_add_epi16(m1, m4);

            m2 = _mm256_sub_epi16(m2, m1);
            m3 = _mm256_slli_epi16(m2, 2);
            m2 = _mm256_add_epi16(m2, m3);

            m0 = _mm256_add_epi16(m0, m5);
            m0 = _mm256_add_epi16(m0, m2);
            m0 = _mm256_add_epi16(m0, _mm256_set1_epi16(16));

            m0 = _mm256_srai_epi16(m0, 5);
            m0 = _mm256_packus_epi16(m0, m0);
            m0 = _mm256_permute4x64_epi64(m0, _MM_SHUFFLE(0, 0, 2, 0));
            _mm_storeu_si128((__m128i *)&pDst[x], _mm256_castsi256_si128(m0));
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int y = nHeight - 4; y < nHeight - 1; y++) {
        for (int x = 0; x < nWidth; x += 32) {
            __m256i m0 = _mm256_loadu_si256((const __m256i *)&pSrc[x]);
            __m256i m1 = _mm256_loadu_si256((const __m256i *)&pSrc[x + nPitch]);

            m0 = _mm256_avg_epu8(m0, m1);
            _mm256_storeu_si256((__m256i *)&pDst[x], m0);
        }

        pSrc += nPitch;
        pDst += nPitch;
    }

    for (int x = 0; x < nWidth; x++)
        pDst[x] = pSrc[x];
}


void HorizontalWiener_avx2(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch,
                           intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample) {
    (void)bitsPerSample;

    for (int y = 0; y < nHeight; y++) {
        pDst[0] = (pSrc[0] + pSrc[1] + 1) >> 1;
        pDst[1] = (pSrc[1] + pSrc[2] + 1) >> 1;

        for (int x = 2; x < nWidth - 4; x += 16) {
            __m256i m0 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x - 2]));
            __m256i m1 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x - 1]));
            __m256i m2 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x]));
            __m256i m3 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + 1]));
            __m256i m4 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + 2]));
            __m256i m5 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)&pSrc[x + 3]));

            m2 = _mm256_add_epi16(m2, m3);
            m2 = _mm256_slli_epi16(m2, 2);

            m1 = _mm256_add_epi16(m1, m4);

            m2 = _mm256_sub_epi16(m2, m1);
            m3 = _mm256_slli_epi16(m2, 2);
            m2 = _mm256_add_epi16(m2, m3);

            m0 = _mm256_add_epi16(m0, m5);
            m0 = _mm256_add_epi16(m0, m2);
            m0 = _mm256_add_epi16(m0, _mm256_set1_epi16(16));

            m0 = _mm256_srai_epi16(m0, 5);
            m0 = _mm256_packus_epi16(m0, m0);
            m0 = _mm256_permute4x64_epi64(m0, _MM_SHUFFLE(0, 0, 2, 0));
            _mm_storeu_si128((__m128i *)&pDst[x], _mm256_castsi256_si128(m0));
        }

        for (int x = nWidth - 4; x < nWidth - 1; x++)
            pDst[x] = (pSrc[x] + pSrc[x + 1] + 1) >> 1;

        pDst[nWidth - 1] = pSrc[nWidth - 1];

        pDst += nPitch;
        pSrc += nPitch;
    }
}

#endif // MVTOOLS_X86