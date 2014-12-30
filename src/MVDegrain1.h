#ifndef __MV_DEGRAIN1__
#define __MV_DEGRAIN1__

#include <emmintrin.h>


typedef void (*Denoise1Function)(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
        const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
        int WSrc, int WRefB, int WRefF);


template <int width, int height>
void Degrain1_C(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
        const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
        int WSrc, int WRefB, int WRefF)
{
    for (int h=0; h<height; h++)
    {
        for (int x=0; x<width; x++)
        {
            pDst[x] = (pRefF[x]*WRefF + pSrc[x]*WSrc + pRefB[x]*WRefB + 128)>>8;// weighted (by SAD) average
        }
        pDst += nDstPitch;
        pSrc += nSrcPitch;
        pRefB += BPitch;
        pRefF += FPitch;
    }
}


template<int blockWidth, int blockHeight>
void Degrain1_sse2(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
        const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
        int WSrc, int WRefB, int WRefF)
{
    __m128i zero = _mm_setzero_si128();
    __m128i wsrc = _mm_set1_epi16(WSrc);
    __m128i wrefb = _mm_set1_epi16(WRefB);
    __m128i wreff = _mm_set1_epi16(WRefF);

    __m128i src, reff, refb;

    for (int y = 0; y < blockHeight; y++) {
        for (int x = 0; x < blockWidth; x += 8) {
            // pDst[x] = (pRefF[x]*WRefF + pSrc[x]*WSrc + pRefB[x]*WRefB + 128)>>8;// weighted (by SAD) average

            if (blockWidth == 4) {
                src  = _mm_cvtsi32_si128(*(const int *)pSrc);
                refb = _mm_cvtsi32_si128(*(const int *)pRefB);
                reff = _mm_cvtsi32_si128(*(const int *)pRefF);
            } else {
                src  = _mm_loadl_epi64((const __m128i *)(pSrc   + x));
                refb = _mm_loadl_epi64((const __m128i *)(pRefB  + x));
                reff = _mm_loadl_epi64((const __m128i *)(pRefF  + x));
            }

            src  = _mm_unpacklo_epi8(src, zero);
            refb = _mm_unpacklo_epi8(refb, zero);
            reff = _mm_unpacklo_epi8(reff, zero);

            src  = _mm_mullo_epi16(src, wsrc);
            refb = _mm_mullo_epi16(refb, wrefb);
            reff = _mm_mullo_epi16(reff, wreff);

            __m128i accum = _mm_set1_epi16(128);

            accum = _mm_add_epi16(accum, src);
            accum = _mm_add_epi16(accum, refb);
            accum = _mm_add_epi16(accum, reff);

            accum = _mm_srli_epi16(accum, 8);
            accum = _mm_packus_epi16(accum, zero);

            if (blockWidth == 4) {
                *(int *)pDst = _mm_cvtsi128_si32(accum);
            } else {
                _mm_storel_epi64((__m128i *)(pDst + x), accum);
            }
        }
        pDst += nDstPitch;
        pSrc += nSrcPitch;
        pRefB += BPitch;
        pRefF += FPitch;
    }
}

#endif
