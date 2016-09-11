
#include <stdint.h>

#include <immintrin.h>

#include "CommonFunctions.h"
#include "MaskFun.h"

#ifdef _WIN32
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE inline __attribute__((always_inline))
#endif


template <typename PixelType>
static FORCE_INLINE __m256i mm256_min_epu(const __m256i &a, const __m256i &b) {
    if (sizeof(PixelType) == 1)
        return _mm256_min_epu8(a, b);
    else
        return _mm256_min_epu16(a, b);
}


template <typename PixelType>
static FORCE_INLINE __m256i mm256_max_epu(const __m256i &a, const __m256i &b) {
    if (sizeof(PixelType) == 1)
        return _mm256_max_epu8(a, b);
    else
        return _mm256_max_epu16(a, b);
}


template <typename PixelType>
static FORCE_INLINE __m256i lookup_double_AVX2(const int16_t *VXFull, const int16_t *VYFull, const PixelType *pref, int w, const __m256i &dwords_ref_pitch, const __m256i &dwords_hoffsets) {
    __m256i vx = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i *)&VXFull[w]));
    vx = _mm256_srai_epi32(vx, 1);

    __m256i vy = _mm256_cvtepu16_epi32(_mm_loadu_si128((const __m128i *)&VYFull[w]));
    vy = _mm256_srai_epi16(vy, 1);

    __m256i addr = _mm256_madd_epi16(vy, dwords_ref_pitch);
    addr = _mm256_add_epi32(addr, vx);
    addr = _mm256_add_epi32(addr, dwords_hoffsets);

    // It's okay to read two or three bytes more than needed. pref is always padded, unless the user chooses a horizontal padding of 0, which would be stupid.
    __m256i gathered = _mm256_i32gather_epi32((const int *)pref, addr, sizeof(PixelType));
    gathered = _mm256_and_si256(gathered, _mm256_set1_epi32((1 << (sizeof(PixelType) * 8)) - 1));

    return gathered;
}


template <typename PixelType>
static FORCE_INLINE __m256i lookup_AVX2(const int16_t *VXFull, const int16_t *VYFull, const PixelType *pref, int w, const __m256i &dwords_time256, const __m256i &dwords_ref_pitch, const __m256i &dwords_hoffsets) {
    __m256i vx = _mm256_cvtepu16_epi32(_mm_loadu_si128((const __m128i *)&VXFull[w]));
    __m256i vy = _mm256_cvtepu16_epi32(_mm_loadu_si128((const __m128i *)&VYFull[w]));

    vx = _mm256_madd_epi16(vx, dwords_time256);
    vx = _mm256_srai_epi32(vx, 8);

    vy = _mm256_madd_epi16(vy, dwords_time256);
    vy = _mm256_srai_epi32(vy, 8);
    __m256i addr = _mm256_madd_epi16(vy, dwords_ref_pitch);
    addr = _mm256_add_epi32(addr, vx);
    addr = _mm256_add_epi32(addr, dwords_hoffsets);

    // It's okay to read two or three bytes more than needed. pref is always padded, unless the user chooses a horizontal padding of 0, which would be stupid.
    __m256i gathered = _mm256_i32gather_epi32((const int *)pref, addr, sizeof(PixelType));
    gathered = _mm256_and_si256(gathered, _mm256_set1_epi32((1 << (sizeof(PixelType) * 8)) - 1));

    return gathered;
}


template <typename PixelType>
static FORCE_INLINE void FlowInterSimple_double_8px_AVX2(
        int w, PixelType *pdst,
        const PixelType *prefB, const PixelType *prefF,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF,
        int nPelLog,
        const __m256i &dwords_ref_pitch, const __m256i &dwords_hoffsets) {

    __m256i dwords_w = _mm256_add_epi32(_mm256_set1_epi32(w << nPelLog), dwords_hoffsets); /// maybe do it another way

    __m256i dstF = lookup_double_AVX2(VXFullF, VYFullF, prefF, w, dwords_ref_pitch, dwords_w);
    __m256i dstB = lookup_double_AVX2(VXFullB, VYFullB, prefB, w, dwords_ref_pitch, dwords_w);

    __m256i maskf = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskF[w]));
    __m256i maskb = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskB[w]));

    __m256i dstF_dstB = _mm256_add_epi32(dstF, dstB);
    dstF_dstB = _mm256_slli_epi32(dstF_dstB, 8);

    __m256i dst;
    if (sizeof(PixelType) == 1) {
        __m256i dstB_dstF = _mm256_sub_epi16(dstB, dstF);
        __m256i maskf_maskb = _mm256_sub_epi16(maskf, maskb);
        dst = _mm256_madd_epi16(dstB_dstF, maskf_maskb);
    } else {
        __m256i dstB_dstF = _mm256_sub_epi32(dstB, dstF);
        __m256i maskf_maskb = _mm256_sub_epi32(maskf, maskb);
        dst = _mm256_mullo_epi32(dstB_dstF, maskf_maskb);
    }

    dst = _mm256_add_epi32(dst, dstF_dstB);
    dst = _mm256_srai_epi32(dst, 9);

    dst = _mm256_packus_epi32(dst, dst);
    dst = _mm256_permute4x64_epi64(dst, 0xe8); // 0b11101000 - copy third qword to second qword
    __m128i dst128 = _mm256_castsi256_si128(dst);

    if (sizeof(PixelType) == 1) {
        dst128 = _mm_packus_epi16(dst128, dst128);
        _mm_storel_epi64((__m128i *)&pdst[w], dst128);
    } else {
        _mm_storeu_si128((__m128i *)&pdst[w], dst128);
    }
}


template <typename PixelType>
static FORCE_INLINE void FlowInterSimple_generic_8px_AVX2(
        int w, PixelType *pdst,
        const PixelType *prefB, const PixelType *prefF,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF,
        int nPelLog,
        const __m256i &dwords_time256, const __m256i &dwords_256_time256,
        const __m256i &dwords_ref_pitch, const __m256i &dwords_hoffsets) {

    __m256i dwords_w = _mm256_add_epi32(_mm256_set1_epi32(w << nPelLog), dwords_hoffsets);

    __m256i dstF = lookup_AVX2(VXFullF, VYFullF, prefF, w, dwords_time256, dwords_ref_pitch, dwords_w);
    __m256i dstB = lookup_AVX2(VXFullB, VYFullB, prefB, w, dwords_256_time256, dwords_ref_pitch, dwords_w);

    __m256i maskf = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskF[w]));
    __m256i maskb = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskB[w]));

    const __m256i dwords_255 = _mm256_set1_epi32(255);

    __m256i maskf_inv = _mm256_sub_epi32(dwords_255, maskf);
    __m256i maskb_inv = _mm256_sub_epi32(dwords_255, maskb);

    __m256i f, b;

    if (sizeof(PixelType) == 1) {
        __m256i dstF_dstB = _mm256_or_si256(dstF, _mm256_slli_epi32(dstB, 16));

        maskf = _mm256_or_si256(_mm256_slli_epi32(maskf, 16), maskf_inv);
        maskb = _mm256_or_si256(maskb, _mm256_slli_epi32(maskb_inv, 16));

        f = _mm256_madd_epi16(dstF_dstB, maskf);
        b = _mm256_madd_epi16(dstF_dstB, maskb);
    } else {
        __m256i dstF_maskf_inv = _mm256_mullo_epi32(dstF, maskf_inv);
        __m256i dstB_maskb_inv = _mm256_mullo_epi32(dstB, maskb_inv);

        __m256i dstB_maskf = _mm256_mullo_epi32(dstB, maskf);
        __m256i dstF_maskb = _mm256_mullo_epi32(dstF, maskb);

        f = _mm256_add_epi32(dstF_maskf_inv, dstB_maskf);
        b = _mm256_add_epi32(dstB_maskb_inv, dstF_maskb);
    }

    f = _mm256_add_epi32(f, dwords_255);
    b = _mm256_add_epi32(b, dwords_255);

    f = _mm256_srai_epi32(f, 8);
    b = _mm256_srai_epi32(b, 8);

    if (sizeof(PixelType) == 1) {
        f = _mm256_madd_epi16(f, dwords_256_time256);
        b = _mm256_madd_epi16(b, dwords_time256);
    } else {
        f = _mm256_mullo_epi32(f, dwords_256_time256);
        b = _mm256_mullo_epi32(b, dwords_time256);
    }

    __m256i dst = _mm256_add_epi32(f, b);
    dst = _mm256_srai_epi32(dst, 8);

    dst = _mm256_packus_epi32(dst, dst);
    dst = _mm256_permute4x64_epi64(dst, 0xe8); // 0b11101000 - copy third qword to second qword
    __m128i dst128 = _mm256_castsi256_si128(dst);

    if (sizeof(PixelType) == 1) {
        dst128 = _mm_packus_epi16(dst128, dst128);
        _mm_storel_epi64((__m128i *)&pdst[w], dst128);
    } else {
        _mm_storeu_si128((__m128i *)&pdst[w], dst128);
    }
}


template <typename PixelType>
static void FlowInterSimple_AVX2(
        uint8_t *pdst8, int dst_pitch,
        const uint8_t *prefB8, const uint8_t *prefF8, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel) {

    const PixelType *prefB = (const PixelType *)prefB8;
    const PixelType *prefF = (const PixelType *)prefF8;
    PixelType *pdst = (PixelType *)pdst8;

    ref_pitch /= sizeof(PixelType);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    const __m256i dwords_time256 = _mm256_set1_epi32(time256);
    const __m256i dwords_256_time256 = _mm256_set1_epi32(256 - time256);
    const __m256i dwords_ref_pitch = _mm256_set1_epi32(ref_pitch);
    const __m256i dwords_hoffsets = _mm256_set_epi32(7 << nPelLog, 6 << nPelLog, 5 << nPelLog, 4 << nPelLog, 3 << nPelLog, 2 << nPelLog, 1 << nPelLog, 0);

    const int pixels_per_iteration = 8;
    const int width_avx2 = width & ~(pixels_per_iteration - 1);

    if (time256 == 128) { /* special case double fps - fastest */
        for (int h = 0; h < height; h++) {
            for (int w = 0; w < width_avx2; w += pixels_per_iteration)
                FlowInterSimple_double_8px_AVX2(w, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, dwords_ref_pitch, dwords_hoffsets);

            if (width_avx2 < width)
                FlowInterSimple_double_8px_AVX2(width - pixels_per_iteration, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, dwords_ref_pitch, dwords_hoffsets);

            pdst += dst_pitch;
            prefB += ref_pitch << nPelLog;
            prefF += ref_pitch << nPelLog;
            VXFullB += VPitch;
            VYFullB += VPitch;
            VXFullF += VPitch;
            VYFullF += VPitch;
            MaskB += VPitch;
            MaskF += VPitch;
        }
    } else { /* general case */
        for (int h = 0; h < height; h++) {
            for (int w = 0; w < width_avx2; w += pixels_per_iteration)
                FlowInterSimple_generic_8px_AVX2(w, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, dwords_time256, dwords_256_time256, dwords_ref_pitch, dwords_hoffsets);

            if (width_avx2 < width)
                FlowInterSimple_generic_8px_AVX2(width - pixels_per_iteration, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, dwords_time256, dwords_256_time256, dwords_ref_pitch, dwords_hoffsets);

            pdst += dst_pitch;
            prefB += ref_pitch << nPelLog;
            prefF += ref_pitch << nPelLog;
            VXFullB += VPitch;
            VYFullB += VPitch;
            VXFullF += VPitch;
            VYFullF += VPitch;
            MaskB += VPitch;
            MaskF += VPitch;
        }
    }
}


template <typename PixelType>
static FORCE_INLINE void FlowInter_8px_AVX2(
        int w, PixelType *pdst,
        const PixelType *prefB, const PixelType *prefF,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF,
        int nPelLog,
        const __m256i &dwords_time256, const __m256i &dwords_256_time256,
        const __m256i &dwords_ref_pitch, const __m256i &dwords_hoffsets) {

    __m256i dwords_w = _mm256_add_epi32(_mm256_set1_epi32(w << nPelLog), dwords_hoffsets);

    __m256i dstF = lookup_AVX2(VXFullF, VYFullF, prefF, w, dwords_time256, dwords_ref_pitch, dwords_w);
    __m256i dstB = lookup_AVX2(VXFullB, VYFullB, prefB, w, dwords_256_time256, dwords_ref_pitch, dwords_w);

    __m256i dstF0 = _mm256_i32gather_epi32((const int *)prefF, dwords_w, sizeof(PixelType));
    __m256i dstB0 = _mm256_i32gather_epi32((const int *)prefB, dwords_w, sizeof(PixelType));
    dstF0 = _mm256_and_si256(dstF0, _mm256_set1_epi32((1 << (sizeof(PixelType) * 8)) - 1));
    dstB0 = _mm256_and_si256(dstB0, _mm256_set1_epi32((1 << (sizeof(PixelType) * 8)) - 1));

    __m256i maskf = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskF[w]));
    __m256i maskb = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskB[w]));

    const __m256i dwords_255 = _mm256_set1_epi32(255);

    __m256i maskf_inv = _mm256_sub_epi32(dwords_255, maskf);
    __m256i maskb_inv = _mm256_sub_epi32(dwords_255, maskb);

    __m256i dstF_maskf_inv, dstB_maskb_inv, dstF0_maskb, dstB0_maskf;

    if (sizeof(PixelType) == 1) {
        dstF_maskf_inv = _mm256_mullo_epi16(dstF, maskf_inv);
        dstB_maskb_inv = _mm256_mullo_epi16(dstB, maskb_inv);

        dstF0_maskb = _mm256_mullo_epi16(dstF0, maskb);
        dstB0_maskf = _mm256_mullo_epi16(dstB0, maskf);
    } else {
        dstF_maskf_inv = _mm256_mullo_epi32(dstF, maskf_inv);
        dstB_maskb_inv = _mm256_mullo_epi32(dstB, maskb_inv);

        dstF0_maskb = _mm256_mullo_epi32(dstF0, maskb);
        dstB0_maskf = _mm256_mullo_epi32(dstB0, maskf);
    }

    __m256i f = _mm256_add_epi32(dstF0_maskb, dstB_maskb_inv);
    __m256i b = _mm256_add_epi32(dstB0_maskf, dstF_maskf_inv);

    if (sizeof(PixelType) == 1) {
        f = _mm256_mullo_epi32(f, maskf);
        b = _mm256_mullo_epi32(b, maskb);

        f = _mm256_add_epi32(f, dwords_255);
        b = _mm256_add_epi32(b, dwords_255);

        f = _mm256_srai_epi32(f, 8);
        b = _mm256_srai_epi32(b, 8);
    } else {
        const __m256i qwords_255 = _mm256_set1_epi64x(255);

        __m256i tempf = _mm256_mul_epu32(f, maskf);
        __m256i tempb = _mm256_mul_epu32(b, maskb);
        tempf = _mm256_add_epi64(tempf, qwords_255);
        tempb = _mm256_add_epi64(tempb, qwords_255);
        tempf = _mm256_srli_epi64(tempf, 8);
        tempb = _mm256_srli_epi64(tempb, 8);

        f = _mm256_srli_epi64(f, 32);
        b = _mm256_srli_epi64(b, 32);
        f = _mm256_mul_epu32(f, _mm256_srli_epi64(maskf, 32));
        b = _mm256_mul_epu32(b, _mm256_srli_epi64(maskb, 32));
        f = _mm256_add_epi64(f, qwords_255);
        b = _mm256_add_epi64(b, qwords_255);
        f = _mm256_srli_epi64(f, 8);
        b = _mm256_srli_epi64(b, 8);
        f = _mm256_or_si256(tempf, _mm256_slli_epi64(f, 32));
        b = _mm256_or_si256(tempb, _mm256_slli_epi64(b, 32));
    }

    f = _mm256_add_epi32(f, dstF_maskf_inv);
    b = _mm256_add_epi32(b, dstB_maskb_inv);

    f = _mm256_add_epi32(f, dwords_255);
    b = _mm256_add_epi32(b, dwords_255);

    f = _mm256_srai_epi32(f, 8);
    b = _mm256_srai_epi32(b, 8);

    if (sizeof(PixelType) == 1) {
        f = _mm256_madd_epi16(f, dwords_256_time256);
        b = _mm256_madd_epi16(b, dwords_time256);
    } else {
        f = _mm256_mullo_epi32(f, dwords_256_time256);
        b = _mm256_mullo_epi32(b, dwords_time256);
    }

    __m256i dst = _mm256_add_epi32(f, b);
    dst = _mm256_srai_epi32(dst, 8);

    dst = _mm256_packus_epi32(dst, dst);
    dst = _mm256_permute4x64_epi64(dst, 0xe8); // 0b11101000 - copy third qword to second qword
    __m128i dst128 = _mm256_castsi256_si128(dst);

    if (sizeof(PixelType) == 1) {
        dst128 = _mm_packus_epi16(dst128, dst128);
        _mm_storel_epi64((__m128i *)&pdst[w], dst128);
    } else {
        _mm_storeu_si128((__m128i *)&pdst[w], dst128);
    }
}


template <typename PixelType>
static void FlowInter_AVX2(
        uint8_t *pdst8, int dst_pitch,
        const uint8_t *prefB8, const uint8_t *prefF8, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel) {

    const PixelType *prefB = (const PixelType *)prefB8;
    const PixelType *prefF = (const PixelType *)prefF8;
    PixelType *pdst = (PixelType *)pdst8;

    ref_pitch /= sizeof(PixelType);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    const __m256i dwords_time256 = _mm256_set1_epi32(time256);
    const __m256i dwords_256_time256 = _mm256_set1_epi32(256 - time256);
    const __m256i dwords_ref_pitch = _mm256_set1_epi32(ref_pitch);
    const __m256i dwords_hoffsets = _mm256_set_epi32(7 << nPelLog, 6 << nPelLog, 5 << nPelLog, 4 << nPelLog, 3 << nPelLog, 2 << nPelLog, 1 << nPelLog, 0);

    const int pixels_per_iteration = 8;
    const int width_avx2 = width & ~(pixels_per_iteration - 1);

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width_avx2; w += pixels_per_iteration)
            FlowInter_8px_AVX2(w, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, dwords_time256, dwords_256_time256, dwords_ref_pitch, dwords_hoffsets);

        if (width_avx2 < width)
            FlowInter_8px_AVX2(width - pixels_per_iteration, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, dwords_time256, dwords_256_time256, dwords_ref_pitch, dwords_hoffsets);

        pdst += dst_pitch;
        prefB += ref_pitch << nPelLog;
        prefF += ref_pitch << nPelLog;
        VXFullB += VPitch;
        VYFullB += VPitch;
        VXFullF += VPitch;
        VYFullF += VPitch;
        MaskB += VPitch;
        MaskF += VPitch;
    }
}


template <typename PixelType>
static FORCE_INLINE void FlowInterExtra_8px_AVX2(
        int w, PixelType *pdst,
        const PixelType *prefB, const PixelType *prefF,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF,
        int nPelLog,
        const int16_t *VXFullBB, const int16_t *VXFullFF,
        const int16_t *VYFullBB, const int16_t *VYFullFF,
        const __m256i &dwords_time256, const __m256i &dwords_256_time256,
        const __m256i &dwords_ref_pitch, const __m256i &dwords_hoffsets) {

    __m256i dwords_w = _mm256_add_epi32(_mm256_set1_epi32(w << nPelLog), dwords_hoffsets);

    __m256i dstF = lookup_AVX2(VXFullF, VYFullF, prefF, w, dwords_time256, dwords_ref_pitch, dwords_w);
    __m256i dstB = lookup_AVX2(VXFullB, VYFullB, prefB, w, dwords_256_time256, dwords_ref_pitch, dwords_w);
    __m256i dstFF = lookup_AVX2(VXFullFF, VYFullFF, prefF, w, dwords_time256, dwords_ref_pitch, dwords_w);
    __m256i dstBB = lookup_AVX2(VXFullBB, VYFullBB, prefB, w, dwords_256_time256, dwords_ref_pitch, dwords_w);

    __m256i minfb = mm256_min_epu<PixelType>(dstF, dstB);
    __m256i maxfb = mm256_max_epu<PixelType>(dstF, dstB);

    __m256i medianBB = mm256_max_epu<PixelType>(minfb, mm256_min_epu<PixelType>(maxfb, dstBB));
    __m256i medianFF = mm256_max_epu<PixelType>(minfb, mm256_min_epu<PixelType>(maxfb, dstFF));

    __m256i maskf = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskF[w]));
    __m256i maskb = _mm256_cvtepu8_epi32(_mm_loadl_epi64((const __m128i *)&MaskB[w]));

    const __m256i dwords_255 = _mm256_set1_epi32(255);

    __m256i maskf_inv = _mm256_sub_epi32(dwords_255, maskf);
    __m256i maskb_inv = _mm256_sub_epi32(dwords_255, maskb);

    if (sizeof(PixelType) == 1) {
        dstF = _mm256_mullo_epi16(dstF, maskf_inv);
        dstB = _mm256_mullo_epi16(dstB, maskb_inv);

        medianBB = _mm256_mullo_epi16(medianBB, maskf);
        medianFF = _mm256_mullo_epi16(medianFF, maskb);
    } else {
        dstF = _mm256_mullo_epi32(dstF, maskf_inv);
        dstB = _mm256_mullo_epi32(dstB, maskb_inv);

        medianBB = _mm256_mullo_epi32(medianBB, maskf);
        medianFF = _mm256_mullo_epi32(medianFF, maskb);
    }

    dstF = _mm256_add_epi32(dstF, dwords_255);
    dstB = _mm256_add_epi32(dstB, dwords_255);

    dstF = _mm256_add_epi32(dstF, medianBB);
    dstB = _mm256_add_epi32(dstB, medianFF);

    dstF = _mm256_srai_epi32(dstF, 8);
    dstB = _mm256_srai_epi32(dstB, 8);

    if (sizeof(PixelType) == 2) {
        dstF = _mm256_sub_epi16(dstF, _mm256_set1_epi32(32768));
        dstB = _mm256_sub_epi16(dstB, _mm256_set1_epi32(32768));
    }
    dstF = _mm256_madd_epi16(dstF, dwords_256_time256);
    dstB = _mm256_madd_epi16(dstB, dwords_time256);
    if (sizeof(PixelType) == 2) {
//        dstF = _mm256_add_epi32(dstF, _mm256_slli_epi32(dwords_256_time256, 15));
//        dstB = _mm256_add_epi32(dstB, _mm256_slli_epi32(dwords_time256, 15));
        // Knowing that they add up to 256, the two additions can be combined.
        dstF = _mm256_add_epi32(dstF, _mm256_set1_epi32(256 << 15));
    }

    __m256i dst = _mm256_add_epi32(dstF, dstB);
    dst = _mm256_srai_epi32(dst, 8);

    dst = _mm256_packus_epi32(dst, dst);
    dst = _mm256_permute4x64_epi64(dst, 0xe8); // 0b11101000 - copy third qword to second qword
    __m128i dst128 = _mm256_castsi256_si128(dst);

    if (sizeof(PixelType) == 1) {
        dst128 = _mm_packus_epi16(dst128, dst128);
        _mm_storel_epi64((__m128i *)&pdst[w], dst128);
    } else {
        _mm_storeu_si128((__m128i *)&pdst[w], dst128);
    }
}


template <typename PixelType>
static void FlowInterExtra_AVX2(
        uint8_t *pdst8, int dst_pitch,
        const uint8_t *prefB8, const uint8_t *prefF8, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel,
        const int16_t *VXFullBB, const int16_t *VXFullFF,
        const int16_t *VYFullBB, const int16_t *VYFullFF) {

    const PixelType *prefB = (const PixelType *)prefB8;
    const PixelType *prefF = (const PixelType *)prefF8;
    PixelType *pdst = (PixelType *)pdst8;

    ref_pitch /= sizeof(PixelType);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    const __m256i dwords_time256 = _mm256_set1_epi32(time256);
    const __m256i dwords_256_time256 = _mm256_set1_epi32(256 - time256);
    const __m256i dwords_ref_pitch = _mm256_set1_epi32(ref_pitch);
    const __m256i dwords_hoffsets = _mm256_set_epi32(7 << nPelLog, 6 << nPelLog, 5 << nPelLog, 4 << nPelLog, 3 << nPelLog, 2 << nPelLog, 1 << nPelLog, 0);

    const int pixels_per_iteration = 8;
    const int width_avx2 = width & ~(pixels_per_iteration - 1);

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width_avx2; w += pixels_per_iteration)
            FlowInterExtra_8px_AVX2(w, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, VXFullBB, VXFullFF, VYFullBB, VYFullFF, dwords_time256, dwords_256_time256, dwords_ref_pitch, dwords_hoffsets);

        if (width_avx2 < width)
            FlowInterExtra_8px_AVX2(width - pixels_per_iteration, pdst, prefB, prefF, VXFullB, VXFullF, VYFullB, VYFullF, MaskB, MaskF, nPelLog, VXFullBB, VXFullFF, VYFullBB, VYFullFF, dwords_time256, dwords_256_time256, dwords_ref_pitch, dwords_hoffsets);

        pdst += dst_pitch;
        prefB += ref_pitch << nPelLog;
        prefF += ref_pitch << nPelLog;
        VXFullB += VPitch;
        VYFullB += VPitch;
        VXFullF += VPitch;
        VYFullF += VPitch;
        MaskB += VPitch;
        MaskF += VPitch;
        VXFullBB += VPitch;
        VYFullBB += VPitch;
        VXFullFF += VPitch;
        VYFullFF += VPitch;
    }
}


void selectFlowInterFunctions_AVX2(FlowInterSimpleFunction *simple, FlowInterFunction *regular, FlowInterExtraFunction *extra, int bitsPerSample) {
    if (bitsPerSample == 8) {
        *simple = FlowInterSimple_AVX2<uint8_t>;
        *regular = FlowInter_AVX2<uint8_t>;
        *extra = FlowInterExtra_AVX2<uint8_t>;
    } else {
        *simple = FlowInterSimple_AVX2<uint16_t>;
        *regular = FlowInter_AVX2<uint16_t>;
        *extra = FlowInterExtra_AVX2<uint16_t>;
    }
}
