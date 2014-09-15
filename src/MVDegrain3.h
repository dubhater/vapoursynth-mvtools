#ifndef __MV_DEGRAIN3__
#define __MV_DEGRAIN3__


typedef void (Denoise3Function)(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
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

#include <mmintrin.h>
template<int blockWidth, int blockHeight>
void Degrain3_mmx(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
						const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
						const uint8_t *pRefB2, int B2Pitch, const uint8_t *pRefF2, int F2Pitch,
						const uint8_t *pRefB3, int B3Pitch, const uint8_t *pRefF3, int F3Pitch,
						int WSrc, int WRefB, int WRefF, int WRefB2, int WRefF2, int WRefB3, int WRefF3)
{
	__m64 z = _mm_setzero_si64();
	__m64 o = _mm_set1_pi16(128);
	__m64 ws = _mm_set1_pi16(WSrc);
	__m64 wb1 = _mm_set1_pi16(WRefB);
	__m64 wf1 = _mm_set1_pi16(WRefF);
	__m64 wb2 = _mm_set1_pi16(WRefB2);
	__m64 wf2 = _mm_set1_pi16(WRefF2);
	__m64 wb3 = _mm_set1_pi16(WRefB3);
	__m64 wf3 = _mm_set1_pi16(WRefF3);

	for (int h=0; h<blockHeight; h++)
	{
		for (int x=0; x<blockWidth; x+=4)
		{
			 *(int*)(pDst + x) = _m_to_int(_m_packuswb(_m_psrlwi(
				 _m_paddw(_m_pmullw(_m_punpcklbw(*(__m64*)(pSrc   + x), z), ws),
				 _m_paddw(_m_pmullw(_m_punpcklbw(*(__m64*)(pRefB  + x), z), wb1),
				 _m_paddw(_m_pmullw(_m_punpcklbw(*(__m64*)(pRefF  + x), z), wf1),
				 _m_paddw(_m_pmullw(_m_punpcklbw(*(__m64*)(pRefB2 + x), z), wb2),
				 _m_paddw(_m_pmullw(_m_punpcklbw(*(__m64*)(pRefF2 + x), z), wf2),
				 _m_paddw(_m_pmullw(_m_punpcklbw(*(__m64*)(pRefB3 + x), z), wb3),
				 _m_paddw(_m_pmullw(_m_punpcklbw(*(__m64*)(pRefF3 + x), z), wf3),
				 o))))))), 8), z));
//			 pDst[x] = (pRefF[x]*WRefF + pSrc[x]*WSrc + pRefB[x]*WRefB + pRefF2[x]*WRefF2 + pRefB2[x]*WRefB2 + pRefF3[x]*WRefF3 + pRefB3[x]*WRefB3 + 128)>>8;
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
	_m_empty();
}

#include <emmintrin.h>
template<int blockWidth, int blockHeight>
void Degrain3_sse2(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch,
						const uint8_t *pRefB, int BPitch, const uint8_t *pRefF, int FPitch,
						const uint8_t *pRefB2, int B2Pitch, const uint8_t *pRefF2, int F2Pitch,
						const uint8_t *pRefB3, int B3Pitch, const uint8_t *pRefF3, int F3Pitch,
						int WSrc, int WRefB, int WRefF, int WRefB2, int WRefF2, int WRefB3, int WRefF3)
{
	__m128i z = _mm_setzero_si128();
	__m128i o = _mm_set1_epi16(128);
	__m128i ws = _mm_set1_epi16(WSrc);
	__m128i wb1 = _mm_set1_epi16(WRefB);
	__m128i wf1 = _mm_set1_epi16(WRefF);
	__m128i wb2 = _mm_set1_epi16(WRefB2);
	__m128i wf2 = _mm_set1_epi16(WRefF2);
	__m128i wb3 = _mm_set1_epi16(WRefB3);
	__m128i wf3 = _mm_set1_epi16(WRefF3);

	for (int h=0; h<blockHeight; h++)
	{
		for (int x=0; x<blockWidth; x+=8)
		{
			 _mm_storel_epi64((__m128i*)(pDst + x), _mm_packus_epi16(_mm_srli_epi16(
				 _mm_add_epi16(_mm_mullo_epi16(_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(pSrc   + x)), z), ws),
				 _mm_add_epi16(_mm_mullo_epi16(_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(pRefB  + x)), z), wb1),
				 _mm_add_epi16(_mm_mullo_epi16(_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(pRefF  + x)), z), wf1),
				 _mm_add_epi16(_mm_mullo_epi16(_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(pRefB2 + x)), z), wb2),
				 _mm_add_epi16(_mm_mullo_epi16(_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(pRefF2 + x)), z), wf2),
				 _mm_add_epi16(_mm_mullo_epi16(_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(pRefB3 + x)), z), wb3),
				 _mm_add_epi16(_mm_mullo_epi16(_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(pRefF3 + x)), z), wf3),
				 o))))))), 8), z));
//			 pDst[x] = (pRefF[x]*WRefF + pSrc[x]*WSrc + pRefB[x]*WRefB + pRefF2[x]*WRefF2 + pRefB2[x]*WRefB2 + pRefF3[x]*WRefF3 + pRefB3[x]*WRefB3 + 128)>>8;
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
