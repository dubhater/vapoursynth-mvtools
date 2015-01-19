#ifndef __MV_DEGRAINS__
#define __MV_DEGRAINS__

#include <cstdint>
#include <cstring>

#include <emmintrin.h>

#include <VSHelper.h>

#include "MVInterface.h"


enum VectorOrder {
    Backward1 = 0,
    Forward1,
    Backward2,
    Forward2,
    Backward3,
    Forward3
};


typedef void (*DenoiseFunction)(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch, const uint8_t **_pRefs, const int *nRefPitches, int WSrc, const int *WRefs);


// XXX Moves the pointers passed in pRefs. This is okay because they are not
// used after this function is done with them.
template <int radius, int blockWidth, int blockHeight>
void Degrain_C(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch, const uint8_t **pRefs, const int *nRefPitches, int WSrc, const int *WRefs) {
    for (int y = 0; y < blockHeight; y++) {
        for (int x = 0; x < blockWidth; x++) {
            int sum = 128 + pSrc[x] * WSrc + pRefs[0][x] * WRefs[0] + pRefs[1][x] * WRefs[1];

            if (radius > 1)
                sum += pRefs[2][x] * WRefs[2] + pRefs[3][x] * WRefs[3];

            if (radius > 2)
                sum += pRefs[4][x] * WRefs[4] + pRefs[5][x] * WRefs[5];

            pDst[x] = sum >> 8;
        }

        pDst += nDstPitch;
        pSrc += nSrcPitch;
        pRefs[0] += nRefPitches[0];
        pRefs[1] += nRefPitches[1];

        if (radius > 1) {
            pRefs[2] += nRefPitches[2];
            pRefs[3] += nRefPitches[3];
        }

        if (radius > 2) {
            pRefs[4] += nRefPitches[4];
            pRefs[5] += nRefPitches[5];
        }
    }
}


// XXX Moves the pointers passed in pRefs. This is okay because they are not
// used after this function is done with them.
template <int radius, int blockWidth, int blockHeight>
void Degrain_sse2(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch, const uint8_t **pRefs, const int *nRefPitches, int WSrc, const int *WRefs) {
    __m128i zero = _mm_setzero_si128();
    __m128i wsrc = _mm_set1_epi16(WSrc);
    __m128i wrefs[6];
    wrefs[0] = _mm_set1_epi16(WRefs[0]);
    wrefs[1] = _mm_set1_epi16(WRefs[1]);
    if (radius > 1) {
        wrefs[2] = _mm_set1_epi16(WRefs[2]);
        wrefs[3] = _mm_set1_epi16(WRefs[3]);
    }
    if (radius > 2) {
        wrefs[4] = _mm_set1_epi16(WRefs[4]);
        wrefs[5] = _mm_set1_epi16(WRefs[5]);
    }

    __m128i src, refs[6];

    for (int y = 0; y < blockHeight; y++) {
        for (int x = 0; x < blockWidth; x += 8) {
            // pDst[x] = (pRefF[x]*WRefF + pSrc[x]*WSrc + pRefB[x]*WRefB + pRefF2[x]*WRefF2 + pRefB2[x]*WRefB2 + pRefF3[x]*WRefF3 + pRefB3[x]*WRefB3 + 128)>>8;

            if (blockWidth == 4) {
                src   = _mm_cvtsi32_si128(*(const int *)pSrc);
                refs[0] = _mm_cvtsi32_si128(*(const int *)pRefs[0]);
                refs[1] = _mm_cvtsi32_si128(*(const int *)pRefs[1]);
                if (radius > 1) {
                    refs[2] = _mm_cvtsi32_si128(*(const int *)pRefs[2]);
                    refs[3] = _mm_cvtsi32_si128(*(const int *)pRefs[3]);
                }
                if (radius > 2) {
                    refs[4] = _mm_cvtsi32_si128(*(const int *)pRefs[4]);
                    refs[5] = _mm_cvtsi32_si128(*(const int *)pRefs[5]);
                }
            } else {
                src   = _mm_loadl_epi64((const __m128i *)(pSrc + x));
                refs[0] = _mm_loadl_epi64((const __m128i *)(pRefs[0] + x));
                refs[1] = _mm_loadl_epi64((const __m128i *)(pRefs[1] + x));
                if (radius > 1) {
                    refs[2] = _mm_loadl_epi64((const __m128i *)(pRefs[2] + x));
                    refs[3] = _mm_loadl_epi64((const __m128i *)(pRefs[3] + x));
                }
                if (radius > 2) {
                    refs[4] = _mm_loadl_epi64((const __m128i *)(pRefs[4] + x));
                    refs[5] = _mm_loadl_epi64((const __m128i *)(pRefs[5] + x));
                }
            }

            src   = _mm_unpacklo_epi8(src, zero);
            refs[0] = _mm_unpacklo_epi8(refs[0], zero);
            refs[1] = _mm_unpacklo_epi8(refs[1], zero);
            if (radius > 1) {
                refs[2] = _mm_unpacklo_epi8(refs[2], zero);
                refs[3] = _mm_unpacklo_epi8(refs[3], zero);
            }
            if (radius > 2) {
                refs[4] = _mm_unpacklo_epi8(refs[4], zero);
                refs[5] = _mm_unpacklo_epi8(refs[5], zero);
            }

            src   = _mm_mullo_epi16(src, wsrc);
            refs[0] = _mm_mullo_epi16(refs[0], wrefs[0]);
            refs[1] = _mm_mullo_epi16(refs[1], wrefs[1]);
            if (radius > 1) {
                refs[2] = _mm_mullo_epi16(refs[2], wrefs[2]);
                refs[3] = _mm_mullo_epi16(refs[3], wrefs[3]);
            }
            if (radius > 2) {
                refs[4] = _mm_mullo_epi16(refs[4], wrefs[4]);
                refs[5] = _mm_mullo_epi16(refs[5], wrefs[5]);
            }

            __m128i accum = _mm_set1_epi16(128);

            accum = _mm_add_epi16(accum, src);
            accum = _mm_add_epi16(accum, refs[0]);
            accum = _mm_add_epi16(accum, refs[1]);
            if (radius > 1) {
                accum = _mm_add_epi16(accum, refs[2]);
                accum = _mm_add_epi16(accum, refs[3]);
            }
            if (radius > 2) {
                accum = _mm_add_epi16(accum, refs[4]);
                accum = _mm_add_epi16(accum, refs[5]);
            }

            accum = _mm_srli_epi16(accum, 8);
            accum = _mm_packus_epi16(accum, zero);

            if (blockWidth == 4)
                *(int *)pDst = _mm_cvtsi128_si32(accum);
            else
                _mm_storel_epi64((__m128i *)(pDst + x), accum);
        }
        pDst += nDstPitch;
        pSrc += nSrcPitch;
        pRefs[0] += nRefPitches[0];
        pRefs[1] += nRefPitches[1];
        if (radius > 1) {
            pRefs[2] += nRefPitches[2];
            pRefs[3] += nRefPitches[3];
        }
        if (radius > 2) {
            pRefs[4] += nRefPitches[4];
            pRefs[5] += nRefPitches[5];
        }
    }
}


extern "C" void mvtools_LimitChanges_sse2(unsigned char *pDst, intptr_t nDstPitch, const unsigned char *pSrc, intptr_t nSrcPitch, intptr_t nWidth, intptr_t nHeight, intptr_t nLimit);


inline void LimitChanges_c(unsigned char *pDst, int nDstPitch, const unsigned char *pSrc, int nSrcPitch, int nWidth, int nHeight, int nLimit) {
    for (int h = 0; h < nHeight; h++) {
        for (int i = 0; i < nWidth; i++)
            pDst[i] = VSMIN( VSMAX(pDst[i], (pSrc[i] - nLimit)), (pSrc[i] + nLimit));
        pDst += nDstPitch;
        pSrc += nSrcPitch;
    }
}


inline int DegrainWeight(int thSAD, int blockSAD) {
    int a = ( (thSAD - blockSAD) > 0 ? (thSAD - blockSAD) : 0 ) * (thSAD + blockSAD);
    int b = a < (1 << 23) ? (a << 8) / (thSAD * thSAD + blockSAD * blockSAD)  // small a
                          : a / ((thSAD * thSAD + blockSAD * blockSAD) >> 8); // very large a, prevent overflow
    return b;
}


inline void useBlock(const uint8_t * &p, int &np, int &WRef, bool isUsable, const MVClipBalls *mvclip, int i, const MVPlane *pPlane, const uint8_t **pSrcCur, int xx, const int *nSrcPitch, int nLogPel, int plane, int xSubUV, int ySubUV, const int *thSAD) {
    if (isUsable) {
        const FakeBlockData &block = mvclip->GetBlock(0, i);
        int blx = (block.GetX() << nLogPel) + block.GetMV().x;
        int bly = (block.GetY() << nLogPel) + block.GetMV().y;
        p = pPlane->GetPointer(plane ? blx >> xSubUV : blx, plane ? bly >> ySubUV : bly);
        np = pPlane->GetPitch();
        int blockSAD = block.GetSAD();
        WRef = DegrainWeight(thSAD[plane], blockSAD);
    } else {
        p = pSrcCur[plane] + xx;
        np = nSrcPitch[plane];
        WRef = 0;
    }
}


template <int radius>
static inline void normaliseWeights(int &WSrc, int *WRefs) {
    // normalise weights to 256
    WSrc = 256;
    int WSum = WSrc + 1;
    for (int r = 0; r < radius*2; r++)
        WSum += WRefs[r];

    for (int r = 0; r < radius*2; r++) {
        WRefs[r] = WRefs[r] * 256 / WSum;
        WSrc -= WRefs[r];
    }
}


#endif
