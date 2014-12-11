#ifndef __MV_DEGRAIN3__
#define __MV_DEGRAIN3__

#include <emmintrin.h>


typedef void (*Denoise3Function)(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
						const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
						const uint8_t *pRefB2, int B2Pitch, const uint8_t *pRefF2, int F2Pitch,
						const uint8_t *pRefB3, int B3Pitch, const uint8_t *pRefF3, int F3Pitch,
						int WSrc, int WRefB, int WRefF, int WRefB2, int WRefF2, int WRefB3, int WRefF3);


template<int blockWidth, int blockHeight>
void Degrain3_C(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
						const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
						const uint8_t *pRefB2, int B2Pitch, const uint8_t *pRefF2, int F2Pitch,
						const uint8_t *pRefB3, int B3Pitch, const uint8_t *pRefF3, int F3Pitch,
						int WSrc, int WRefB, int WRefF, int WRefB2, int WRefF2, int WRefB3, int WRefF3)
{
	for (int h=0; h<blockHeight; h++)
	{
		for (int x=0; x<blockWidth; x++)
		{
			 pDst[x] = (pRefF[x]*WRefF + pSrc[x]*WSrc + pRefB[x]*WRefB + pRefF2[x]*WRefF2 + pRefB2[x]*WRefB2 + pRefF3[x]*WRefF3 + pRefB3[x]*WRefB3 + 128)>>8;
		}
		pDst += nDstPitch;
		pSrc += nSrcPitch;
		pRefB += BPitch;
		pRefF += FPitch;
		pRefB2 += B2Pitch;
		pRefF2 += F2Pitch;
		pRefB3 += B3Pitch;
		pRefF3 += F3Pitch;
	}
}


template<int blockWidth, int blockHeight>
void Degrain3_sse2(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
						const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
						const uint8_t *pRefB2, int B2Pitch, const uint8_t *pRefF2, int F2Pitch,
						const uint8_t *pRefB3, int B3Pitch, const uint8_t *pRefF3, int F3Pitch,
						int WSrc, int WRefB, int WRefF, int WRefB2, int WRefF2, int WRefB3, int WRefF3)
{
    __m128i zero = _mm_setzero_si128();
    __m128i wsrc = _mm_set1_epi16(WSrc);
    __m128i wrefb1 = _mm_set1_epi16(WRefB);
    __m128i wreff1 = _mm_set1_epi16(WRefF);
    __m128i wrefb2 = _mm_set1_epi16(WRefB2);
    __m128i wreff2 = _mm_set1_epi16(WRefF2);
    __m128i wrefb3 = _mm_set1_epi16(WRefB3);
    __m128i wreff3 = _mm_set1_epi16(WRefF3);

    __m128i src, reff1, refb1, reff2, refb2, reff3, refb3;

    for (int y = 0; y < blockHeight; y++) {
        for (int x = 0; x < blockWidth; x += 8) {
            // pDst[x] = (pRefF[x]*WRefF + pSrc[x]*WSrc + pRefB[x]*WRefB + pRefF2[x]*WRefF2 + pRefB2[x]*WRefB2 + pRefF3[x]*WRefF3 + pRefB3[x]*WRefB3 + 128)>>8;

            if (blockWidth == 4) {
                src   = _mm_cvtsi32_si128(*(const int *)pSrc);
                refb1 = _mm_cvtsi32_si128(*(const int *)pRefB);
                reff1 = _mm_cvtsi32_si128(*(const int *)pRefF);
                refb2 = _mm_cvtsi32_si128(*(const int *)pRefB2);
                reff2 = _mm_cvtsi32_si128(*(const int *)pRefF2);
                refb3 = _mm_cvtsi32_si128(*(const int *)pRefB3);
                reff3 = _mm_cvtsi32_si128(*(const int *)pRefF3);
            } else {
                src   = _mm_loadl_epi64((const __m128i *)(pSrc + x));
                refb1 = _mm_loadl_epi64((const __m128i *)(pRefB + x));
                reff1 = _mm_loadl_epi64((const __m128i *)(pRefF + x));
                refb2 = _mm_loadl_epi64((const __m128i *)(pRefB2 + x));
                reff2 = _mm_loadl_epi64((const __m128i *)(pRefF2 + x));
                refb3 = _mm_loadl_epi64((const __m128i *)(pRefB3 + x));
                reff3 = _mm_loadl_epi64((const __m128i *)(pRefF3 + x));
            }

            src   = _mm_unpacklo_epi8(src, zero);
            refb1 = _mm_unpacklo_epi8(refb1, zero);
            reff1 = _mm_unpacklo_epi8(reff1, zero);
            refb2 = _mm_unpacklo_epi8(refb2, zero);
            reff2 = _mm_unpacklo_epi8(reff2, zero);
            refb3 = _mm_unpacklo_epi8(refb3, zero);
            reff3 = _mm_unpacklo_epi8(reff3, zero);

            src   = _mm_mullo_epi16(src, wsrc);
            refb1 = _mm_mullo_epi16(refb1, wrefb1);
            reff1 = _mm_mullo_epi16(reff1, wreff1);
            refb2 = _mm_mullo_epi16(refb2, wrefb2);
            reff2 = _mm_mullo_epi16(reff2, wreff2);
            refb3 = _mm_mullo_epi16(refb3, wrefb3);
            reff3 = _mm_mullo_epi16(reff3, wreff3);

            __m128i accum = _mm_set1_epi16(128);

            accum = _mm_add_epi16(accum, src);
            accum = _mm_add_epi16(accum, refb1);
            accum = _mm_add_epi16(accum, reff1);
            accum = _mm_add_epi16(accum, refb2);
            accum = _mm_add_epi16(accum, reff2);
            accum = _mm_add_epi16(accum, refb3);
            accum = _mm_add_epi16(accum, reff3);

            accum = _mm_srli_epi16(accum, 8);
            accum = _mm_packus_epi16(accum, zero);

            if (blockWidth == 4)
                *(int *)pDst = _mm_cvtsi128_si32(accum);
            else
                _mm_storel_epi64((__m128i *)(pDst + x), accum);
        }
        pDst += nDstPitch;
        pSrc += nSrcPitch;
        pRefB += BPitch;
        pRefF += FPitch;
        pRefB2 += B2Pitch;
        pRefF2 += F2Pitch;
        pRefB3 += B3Pitch;
        pRefF3 += F3Pitch;
    }
}

#endif
