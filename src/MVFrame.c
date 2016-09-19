// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - bicubic, wiener
// See legal notice in Copying.txt for more information
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .

#include <stdio.h>

#include <VSHelper.h>

#include "MVFrame.h"
#include "Interpolation.h"


int PlaneHeightLuma(int src_height, int level, int yRatioUV, int vpad) {
    int height = src_height;

    for (int i = 1; i <= level; i++) {
        height = vpad >= yRatioUV ? ((height / yRatioUV + 1) / 2) * yRatioUV : ((height / yRatioUV) / 2) * yRatioUV;
    }
    return height;
}


int PlaneWidthLuma(int src_width, int level, int xRatioUV, int hpad) {
    int width = src_width;

    for (int i = 1; i <= level; i++) {
        width = hpad >= xRatioUV ? ((width / xRatioUV + 1) / 2) * xRatioUV : ((width / xRatioUV) / 2) * xRatioUV;
    }
    return width;
}


unsigned int PlaneSuperOffset(int chroma, int src_height, int level, int pel, int vpad, int plane_pitch, int yRatioUV) {
    // storing subplanes in superframes may be implemented by various ways
    int height = src_height; // luma or chroma

    unsigned int offset;

    if (level == 0)
        offset = 0;
    else {
        offset = pel * pel * plane_pitch * (src_height + vpad * 2);

        for (int i = 1; i < level; i++) {
            height = chroma ? PlaneHeightLuma(src_height * yRatioUV, i, yRatioUV, vpad * yRatioUV) / yRatioUV : PlaneHeightLuma(src_height, i, yRatioUV, vpad);

            offset += plane_pitch * (height + vpad * 2);
        }
    }
    return offset;
}


#define PadCorner(PixelType) \
void PadCorner_##PixelType(PixelType *p, PixelType v, int hPad, int vPad, int refPitch) { \
    for (int i = 0; i < vPad; i++) {                    \
        if (sizeof(PixelType) == 1)                     \
            memset(p, v, hPad); /* faster than loop */  \
        else                                            \
            for (int j = 0; j < hPad; j++)              \
                p[j] = v;                               \
                                                        \
        p += refPitch;                                  \
    }                                                   \
}

PadCorner(uint8_t)
PadCorner(uint16_t)


#define PadReferenceFrame(PixelType) \
void PadReferenceFrame_##PixelType(uint8_t *refFrame8, int refPitch, int hPad, int vPad, int width, int height) {   \
    refPitch /= sizeof(PixelType);                                                                      \
    PixelType *refFrame = (PixelType *)refFrame8;                                                       \
    PixelType value;                                                                                    \
    PixelType *pfoff = refFrame + vPad * refPitch + hPad;                                               \
    PixelType *p;                                                                                       \
                                                                                                        \
    /* Up-Left */                                                                                       \
    PadCorner_##PixelType(refFrame, pfoff[0], hPad, vPad, refPitch);                                    \
    /* Up-Right */                                                                                      \
    PadCorner_##PixelType(refFrame + hPad + width, pfoff[width - 1], hPad, vPad, refPitch);             \
    /* Down-Left */                                                                                     \
    PadCorner_##PixelType(refFrame + (vPad + height) * refPitch,                                        \
                          pfoff[(height - 1) * refPitch], hPad, vPad, refPitch);                        \
    /* Down-Right */                                                                                    \
    PadCorner_##PixelType(refFrame + hPad + width + (vPad + height) * refPitch,                         \
                          pfoff[(height - 1) * refPitch + width - 1], hPad, vPad, refPitch);            \
                                                                                                        \
    /* Up */                                                                                            \
    for (int i = 0; i < width; i++) {                                                                   \
        value = pfoff[i];                                                                               \
        p = refFrame + hPad + i;                                                                        \
        for (int j = 0; j < vPad; j++) {                                                                \
            p[0] = value;                                                                               \
            p += refPitch;                                                                              \
        }                                                                                               \
    }                                                                                                   \
                                                                                                        \
    /* Left */                                                                                          \
    for (int i = 0; i < height; i++) {                                                                  \
        value = pfoff[i * refPitch];                                                                    \
        p = refFrame + (vPad + i) * refPitch;                                                           \
        for (int j = 0; j < hPad; j++)                                                                  \
            p[j] = value;                                                                               \
    }                                                                                                   \
                                                                                                        \
    /* Right */                                                                                         \
    for (int i = 0; i < height; i++) {                                                                  \
        value = pfoff[i * refPitch + width - 1];                                                        \
        p = refFrame + (vPad + i) * refPitch + width + hPad;                                            \
        for (int j = 0; j < hPad; j++)                                                                  \
            p[j] = value;                                                                               \
    }                                                                                                   \
                                                                                                        \
    /* Down */                                                                                          \
    for (int i = 0; i < width; i++) {                                                                   \
        value = pfoff[i + (height - 1) * refPitch];                                                     \
        p = refFrame + hPad + i + (height + vPad) * refPitch;                                           \
        for (int j = 0; j < vPad; j++) {                                                                \
            p[0] = value;                                                                               \
            p += refPitch;                                                                              \
        }                                                                                               \
    }                                                                                                   \
}

PadReferenceFrame(uint8_t)
PadReferenceFrame(uint16_t)


/******************************************************************************
 *                                                                             *
 *  MVPlane : manages a single plane, allowing padding and refinin             *
 *                                                                             *
 ******************************************************************************/

void mvpInit(MVPlane *mvp, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int opt, int bitsPerSample) {
    mvp->nWidth = nWidth;
    mvp->nHeight = nHeight;
    mvp->nPel = nPel;
    mvp->nHPadding = nHPad;
    mvp->nVPadding = nVPad;
    mvp->opt = opt;
    mvp->nHPaddingPel = nHPad * nPel;
    mvp->nVPaddingPel = nVPad * nPel;
    mvp->bitsPerSample = bitsPerSample;
    mvp->bytesPerSample = (bitsPerSample + 7) / 8; // Who would ever want to process 32 bit video?

    mvp->nPaddedWidth = nWidth + 2 * mvp->nHPadding;
    mvp->nPaddedHeight = nHeight + 2 * mvp->nVPadding;

    mvp->pPlane = (uint8_t **)malloc(nPel * nPel * sizeof(uint8_t *));
}


void mvpDeinit(MVPlane *mvp) {
    free(mvp->pPlane);
}


void mvpResetState(MVPlane *mvp) {
    mvp->isRefined = mvp->isFilled = mvp->isPadded = 0;
}


void mvpUpdate(MVPlane *mvp, uint8_t *pSrc, int _nPitch) { //v2.0
    mvp->nPitch = _nPitch;
    mvp->nOffsetPadding = mvp->nPitch * mvp->nVPadding + mvp->nHPadding * mvp->bytesPerSample;

    for (int i = 0; i < mvp->nPel * mvp->nPel; i++)
        mvp->pPlane[i] = pSrc + i * mvp->nPitch * mvp->nPaddedHeight;

    mvpResetState(mvp);
}


void mvpFillPlane(MVPlane *mvp, const uint8_t *pNewPlane, int nNewPitch) {
    if (!mvp->isFilled)
        vs_bitblt(mvp->pPlane[0] + mvp->nOffsetPadding, mvp->nPitch, pNewPlane, nNewPitch, mvp->nWidth * mvp->bytesPerSample, mvp->nHeight);
    mvp->isFilled = 1;
}


void mvpPad(MVPlane *mvp) {
    if (!mvp->isPadded) {
        if (mvp->bytesPerSample == 1)
            PadReferenceFrame_uint8_t(mvp->pPlane[0], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight);
        else
            PadReferenceFrame_uint16_t(mvp->pPlane[0], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight);
    }

    mvp->isPadded = 1;
}


void mvpRefine(MVPlane *mvp, int sharp) {
    if (mvp->isRefined)
        return;

    if (mvp->nPel == 1) {
        mvp->isRefined = 1;
        return;
    }

    typedef void (*RefineFunction)(uint8_t *pDst, const uint8_t *pSrc, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight, intptr_t bitsPerSample);

    RefineFunction refine[3];

    if (sharp == SharpBilinear) {
        if (mvp->bytesPerSample == 1) {
            refine[0] = HorizontalBilinear_uint8_t;
            refine[1] = VerticalBilinear_uint8_t;
            refine[2] = DiagonalBilinear_uint8_t;

            if (mvp->opt) {
#if defined(MVTOOLS_X86)
                refine[0] = mvtools_HorizontalBilinear_sse2;
                refine[1] = mvtools_VerticalBilinear_sse2;
                refine[2] = mvtools_DiagonalBilinear_sse2;
#endif
            }
        } else {
            refine[0] = HorizontalBilinear_uint16_t;
            refine[1] = VerticalBilinear_uint16_t;
            refine[2] = DiagonalBilinear_uint16_t;
        }
    } else if (sharp == SharpBicubic) {
        if (mvp->bytesPerSample == 1) {
            /* TODO: port the asm
               if (opt)
               {
               f[0] = f[2] = HorizontalBicubic_iSSE;
               f[1] = VerticalBicubic_iSSE;
               }
               else
               */
            refine[0] = refine[2] = HorizontalBicubic_uint8_t;
            refine[1] = VerticalBicubic_uint8_t;
        } else {
            refine[0] = refine[2] = HorizontalBicubic_uint16_t;
            refine[1] = VerticalBicubic_uint16_t;
        }
    } else { // Wiener
        if (mvp->bytesPerSample == 1) {
            refine[0] = refine[2] = HorizontalWiener_uint8_t;
            refine[1] = VerticalWiener_uint8_t;

            if (mvp->opt) {
#if defined(MVTOOLS_X86)
                refine[0] = refine[2] = mvtools_HorizontalWiener_sse2;
                refine[1] = mvtools_VerticalWiener_sse2;
#endif
            }
        } else {
            refine[0] = refine[2] = HorizontalWiener_uint16_t;
            refine[1] = VerticalWiener_uint16_t;
        }
    }

    const uint8_t *src[3];
    uint8_t *dst[3];

    if (mvp->nPel == 2) {
        dst[0] = mvp->pPlane[1];
        dst[1] = mvp->pPlane[2];
        dst[2] = mvp->pPlane[3];
        src[0] = src[1] = mvp->pPlane[0];
        if (sharp == 0)
            src[2] = mvp->pPlane[0];
        else
            src[2] = mvp->pPlane[2];
    } else if (mvp->nPel == 4) {
        dst[0] = mvp->pPlane[2];
        dst[1] = mvp->pPlane[8];
        dst[2] = mvp->pPlane[10];
        src[0] = src[1] = mvp->pPlane[0];
        if (sharp == 0)
            src[2] = mvp->pPlane[0];
        else
            src[2] = mvp->pPlane[8];
    }

    for (int i = 0; i < 3; i++)
        refine[i](dst[i], src[i], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight, mvp->bitsPerSample);

    if (mvp->nPel == 4) {
        typedef void (*AverageFunction)(uint8_t *pDst, const uint8_t *pSrc1, const uint8_t *pSrc2, intptr_t nPitch, intptr_t nWidth, intptr_t nHeight);

        AverageFunction avg;

        if (mvp->bytesPerSample == 1) {
            avg = Average2_uint8_t;

            if (mvp->opt) {
#if defined(MVTOOLS_X86)
                avg = mvtools_Average2_sse2;
#endif
            }
        } else {
            avg = Average2_uint16_t;
        }

        // now interpolate intermediate
        avg(mvp->pPlane[1], mvp->pPlane[0], mvp->pPlane[2], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[9], mvp->pPlane[8], mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[4], mvp->pPlane[0], mvp->pPlane[8], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[6], mvp->pPlane[2], mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[5], mvp->pPlane[4], mvp->pPlane[6], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);

        avg(mvp->pPlane[3], mvp->pPlane[0] + mvp->bytesPerSample, mvp->pPlane[2], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
        avg(mvp->pPlane[11], mvp->pPlane[8] + mvp->bytesPerSample, mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
        avg(mvp->pPlane[12], mvp->pPlane[0] + mvp->nPitch, mvp->pPlane[8], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight - 1);
        avg(mvp->pPlane[14], mvp->pPlane[2] + mvp->nPitch, mvp->pPlane[10], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight - 1);
        avg(mvp->pPlane[13], mvp->pPlane[12], mvp->pPlane[14], mvp->nPitch, mvp->nPaddedWidth, mvp->nPaddedHeight);
        avg(mvp->pPlane[7], mvp->pPlane[4] + mvp->bytesPerSample, mvp->pPlane[6], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
        avg(mvp->pPlane[15], mvp->pPlane[12] + mvp->bytesPerSample, mvp->pPlane[14], mvp->nPitch, mvp->nPaddedWidth - 1, mvp->nPaddedHeight);
    }

    mvp->isRefined = 1;
}


#define mvpRefineExtPel2(PixelType) \
void mvpRefineExtPel2_##PixelType(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded) { \
    const PixelType *pSrc2x = (const PixelType *)pSrc2x8;               \
    PixelType *pp1 = (PixelType *)mvp->pPlane[1];                       \
    PixelType *pp2 = (PixelType *)mvp->pPlane[2];                       \
    PixelType *pp3 = (PixelType *)mvp->pPlane[3];                       \
                                                                        \
    nSrc2xPitch /= sizeof(PixelType);                                   \
    int nPitchTmp = mvp->nPitch / sizeof(PixelType);                    \
                                                                        \
    /* pel clip may be already padded (i.e. is finest clip) */          \
    if (!isExtPadded) {                                                 \
        int offset = nPitchTmp * mvp->nVPadding + mvp->nHPadding;       \
        pp1 += offset;                                                  \
        pp2 += offset;                                                  \
        pp3 += offset;                                                  \
    }                                                                   \
                                                                        \
    for (int h = 0; h < mvp->nHeight; h++) {                            \
        for (int w = 0; w < mvp->nWidth; w++) {                         \
            pp1[w] = pSrc2x[(w << 1) + 1];                              \
            pp2[w] = pSrc2x[(w << 1) + nSrc2xPitch];                    \
            pp3[w] = pSrc2x[(w << 1) + nSrc2xPitch + 1];                \
        }                                                               \
        pp1 += nPitchTmp;                                               \
        pp2 += nPitchTmp;                                               \
        pp3 += nPitchTmp;                                               \
        pSrc2x += nSrc2xPitch * 2;                                      \
    }                                                                   \
                                                                        \
    if (!isExtPadded) {                                                 \
        for (int i = 1; i < 4; i++)                                     \
            PadReferenceFrame_##PixelType(mvp->pPlane[1], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight); \
    }                                                                   \
                                                                        \
    mvp->isPadded = 1;                                               \
}

mvpRefineExtPel2(uint8_t)
mvpRefineExtPel2(uint16_t)


#define mvpRefineExtPel4(PixelType) \
void mvpRefineExtPel4_##PixelType(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded) { \
    const PixelType *pSrc2x = (const PixelType *)pSrc2x8;               \
    PixelType *pp[16];                                                  \
    for (int i = 1; i < 16; i++)                                        \
        pp[i] = (PixelType *)mvp->pPlane[i];                            \
                                                                        \
    nSrc2xPitch /= sizeof(PixelType);                                   \
    int nPitchTmp = mvp->nPitch / sizeof(PixelType);                    \
                                                                        \
    if (!isExtPadded) {                                                 \
        int offset = nPitchTmp * mvp->nVPadding + mvp->nHPadding;       \
        for (int i = 1; i < 16; i++)                                    \
            pp[i] += offset;                                            \
    }                                                                   \
                                                                        \
    for (int h = 0; h < mvp->nHeight; h++) {                            \
        for (int w = 0; w < mvp->nWidth; w++) {                         \
            pp[1][w] = pSrc2x[(w << 2) + 1];                            \
            pp[2][w] = pSrc2x[(w << 2) + 2];                            \
            pp[3][w] = pSrc2x[(w << 2) + 3];                            \
            pp[4][w] = pSrc2x[(w << 2) + nSrc2xPitch];                  \
            pp[5][w] = pSrc2x[(w << 2) + nSrc2xPitch + 1];              \
            pp[6][w] = pSrc2x[(w << 2) + nSrc2xPitch + 2];              \
            pp[7][w] = pSrc2x[(w << 2) + nSrc2xPitch + 3];              \
            pp[8][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2];              \
            pp[9][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 1];          \
            pp[10][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 2];         \
            pp[11][w] = pSrc2x[(w << 2) + nSrc2xPitch * 2 + 3];         \
            pp[12][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3];             \
            pp[13][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 1];         \
            pp[14][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 2];         \
            pp[15][w] = pSrc2x[(w << 2) + nSrc2xPitch * 3 + 3];         \
        }                                                               \
        for (int i = 1; i < 16; i++)                                    \
            pp[i] += nPitchTmp;                                         \
        pSrc2x += nSrc2xPitch * 4;                                      \
    }                                                                   \
    if (!isExtPadded) {                                                 \
        for (int i = 1; i < 16; i++)                                    \
            PadReferenceFrame_##PixelType(mvp->pPlane[i], mvp->nPitch, mvp->nHPadding, mvp->nVPadding, mvp->nWidth, mvp->nHeight); \
    }                                                                   \
                                                                        \
    mvp->isPadded = 1;                                               \
}

mvpRefineExtPel4(uint8_t)
mvpRefineExtPel4(uint16_t)


void mvpRefineExt(MVPlane *mvp, const uint8_t *pSrc2x, int nSrc2xPitch, int isExtPadded) // copy from external upsized clip
{
    if ((mvp->nPel == 2) && (!mvp->isRefined)) {
        if (mvp->bytesPerSample == 1)
            mvpRefineExtPel2_uint8_t(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
        else
            mvpRefineExtPel2_uint16_t(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
    } else if ((mvp->nPel == 4) && (!mvp->isRefined)) {
        if (mvp->bytesPerSample == 1)
            mvpRefineExtPel4_uint8_t(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
        else
            mvpRefineExtPel4_uint16_t(mvp, pSrc2x, nSrc2xPitch, isExtPadded);
    }

    mvp->isRefined = 1;
}


void mvpReduceTo(MVPlane *mvp, MVPlane *pReducedPlane, int rfilter) {
    if (pReducedPlane->isFilled)
        return;

    typedef void (*ReduceFunction)(uint8_t *pDst, const uint8_t *pSrc, int nDstPitch, int nSrcPitch, int nWidth, int nHeight, int opt);

    ReduceFunction reduce = NULL;

    if (rfilter == RfilterSimple) {
        /* TODO: port the asm
               if (opt)
               {
               RB2F_iSSE(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, pPlane[0] + nOffsetPadding,
               pReducedPlane->nPitch, nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight);
               }
               else
               */
        {
            if (mvp->bytesPerSample == 1)
                reduce = RB2F_C_uint8_t;
            else
                reduce = RB2F_C_uint16_t;
        }
    } else if (rfilter == RfilterTriangle) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2Filtered_uint8_t;
        else
            reduce = RB2Filtered_uint16_t;
    } else if (rfilter == RfilterBilinear) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2BilinearFiltered_uint8_t;
        else
            reduce = RB2BilinearFiltered_uint16_t;
    } else if (rfilter == RfilterQuadratic) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2Quadratic_uint8_t;
        else
            reduce = RB2Quadratic_uint16_t;
    } else if (rfilter == RfilterCubic) {
        if (mvp->bytesPerSample == 1)
            reduce = RB2Cubic_uint8_t;
        else
            reduce = RB2Cubic_uint16_t;
    }

    reduce(pReducedPlane->pPlane[0] + pReducedPlane->nOffsetPadding, mvp->pPlane[0] + mvp->nOffsetPadding,
           pReducedPlane->nPitch, mvp->nPitch, pReducedPlane->nWidth, pReducedPlane->nHeight, mvp->opt);

    pReducedPlane->isFilled = 1;
}


const uint8_t *mvpGetAbsolutePointer(const MVPlane *mvp, int nX, int nY) {
    if (mvp->nPel == 1)
        return mvp->pPlane[0] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    else if (mvp->nPel == 2) {
        int idx = (nX & 1) | ((nY & 1) << 1);

        nX >>= 1;
        nY >>= 1;

        return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    } else { // nPel = 4
        int idx = (nX & 3) | ((nY & 3) << 2);

        nX >>= 2;
        nY >>= 2;

        return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    }
}


const uint8_t *mvpGetAbsolutePointerPel1(const MVPlane *mvp, int nX, int nY) {
    return mvp->pPlane[0] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
}


const uint8_t *mvpGetAbsolutePointerPel2(const MVPlane *mvp, int nX, int nY) {
    int idx = (nX & 1) | ((nY & 1) << 1);

    nX >>= 1;
    nY >>= 1;

    return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
}


const uint8_t *mvpGetAbsolutePointerPel4(const MVPlane *mvp, int nX, int nY) {
    int idx = (nX & 3) | ((nY & 3) << 2);

    nX >>= 2;
    nY >>= 2;

    return mvp->pPlane[idx] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
}


const uint8_t *mvpGetPointer(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointer(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetPointerPel1(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointerPel1(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetPointerPel2(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointerPel2(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetPointerPel4(const MVPlane *mvp, int nX, int nY) {
    return mvpGetAbsolutePointerPel4(mvp, nX + mvp->nHPaddingPel, nY + mvp->nVPaddingPel);
}


const uint8_t *mvpGetAbsolutePelPointer(const MVPlane *mvp, int nX, int nY) {
    const uint8_t *ret = mvp->pPlane[0] + nX * mvp->bytesPerSample + nY * mvp->nPitch;
    return ret;
}

/******************************************************************************
 *                                                                             *
 *  MVFrame : a MVFrame is a threesome of MVPlane, some undefined, some        *
 *  defined, according to the nMode value                                      *
 *                                                                             *
 ******************************************************************************/

void mvfInit(MVFrame *mvf, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int nMode, int opt, int xRatioUV, int yRatioUV, int bitsPerSample) {
    mvf->nMode = nMode;

    mvf->planes[0] = mvf->planes[1] = mvf->planes[2] = NULL;

    int width[3] = { nWidth };
    width[1] = width[2] = nWidth / xRatioUV;

    int height[3] = { nHeight };
    height[1] = height[2] = nHeight / yRatioUV;

    int hpad[3] = { nHPad };
    hpad[1] = hpad[2] = nHPad / xRatioUV;

    int vpad[3] = { nVPad };
    vpad[1] = vpad[2] = nVPad / yRatioUV;

    for (int i = 0; i < 3; i++) {
        if (nMode & (1 << i)) {
            mvf->planes[i] = (MVPlane *)malloc(sizeof(MVPlane));
            mvpInit(mvf->planes[i], width[i], height[i], nPel, hpad[i], vpad[i], opt, bitsPerSample);
        }
    }
}


void mvfDeinit(MVFrame *mvf) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i]) {
            mvpDeinit(mvf->planes[i]);
            free(mvf->planes[i]);
            mvf->planes[i] = NULL;
        }
    }
}


void mvfUpdate(MVFrame *mvf, uint8_t **pSrc, int *pitch) {
    for (int i = 0; i < 3; i++) {
        if (pSrc[i] && mvf->planes[i])
            mvpUpdate(mvf->planes[i], pSrc[i], pitch[i]);
    }
}


void mvfFillPlane(MVFrame *mvf, const uint8_t *pNewPlane, int nNewPitch, int plane) {
    if (mvf->planes[plane])
        mvpFillPlane(mvf->planes[plane], pNewPlane, nNewPitch);
}


void mvfRefine(MVFrame *mvf, MVPlaneSet nMode, int sharp) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i] && (nMode & (1 << i)))
            mvpRefine(mvf->planes[i], sharp);
    }
}


void mvfPad(MVFrame *mvf, MVPlaneSet nMode) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i] && (nMode & (1 << i)))
            mvpPad(mvf->planes[i]);
    }
}


void mvfResetState(MVFrame *mvf) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i])
            mvpResetState(mvf->planes[i]);
    }
}


void mvfReduceTo(MVFrame *mvf, MVFrame *pFrame, MVPlaneSet nMode, int rfilter) {
    for (int i = 0; i < 3; i++) {
        if (mvf->planes[i] && (nMode & (1 << i)))
            mvpReduceTo(mvf->planes[i], pFrame->planes[i], rfilter);
    }
}

/******************************************************************************
 *                                                                             *
 *  MVGroupOfFrames : manage a hierachal frame structure                       *
 *                                                                             *
 ******************************************************************************/

void mvgofInit(MVGroupOfFrames *mvgof, int nLevelCount, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int nMode, int opt, int xRatioUV, int yRatioUV, int bitsPerSample) {
    mvgof->nLevelCount = nLevelCount;
    mvgof->nWidth[0] = nWidth;
    mvgof->nWidth[1] = mvgof->nWidth[2] = nWidth / xRatioUV;
    mvgof->nHeight[0] = nHeight;
    mvgof->nHeight[1] = mvgof->nHeight[2] = nHeight / yRatioUV;
    mvgof->nPel = nPel;
    mvgof->nHPad[0] = nHPad;
    mvgof->nHPad[1] = mvgof->nHPad[2] = nHPad / xRatioUV;
    mvgof->nVPad[0] = nVPad;
    mvgof->nVPad[1] = mvgof->nVPad[2] = nVPad / yRatioUV;
    mvgof->xRatioUV = xRatioUV;
    mvgof->yRatioUV = yRatioUV;

    mvgof->frames = (MVFrame **)malloc(mvgof->nLevelCount * sizeof(MVFrame *));

    mvgof->frames[0] = (MVFrame *)malloc(sizeof(MVFrame));
    mvfInit(mvgof->frames[0], mvgof->nWidth[0], mvgof->nHeight[0], mvgof->nPel, mvgof->nHPad[0], mvgof->nVPad[0], nMode, opt, mvgof->xRatioUV, mvgof->yRatioUV, bitsPerSample);

    for (int i = 1; i < mvgof->nLevelCount; i++) {
        int nWidthi = PlaneWidthLuma(mvgof->nWidth[0], i, mvgof->xRatioUV, mvgof->nHPad[0]);    //(nWidthi / 2) - ((nWidthi / 2) % xRatioUV); //  even for YV12
        int nHeighti = PlaneHeightLuma(mvgof->nHeight[0], i, mvgof->yRatioUV, mvgof->nVPad[0]); //(nHeighti / 2) - ((nHeighti / 2) % yRatioUV); // even for YV12

        mvgof->frames[i] = (MVFrame *)malloc(sizeof(MVFrame));
        mvfInit(mvgof->frames[i], nWidthi, nHeighti, 1, mvgof->nHPad[0], mvgof->nVPad[0], nMode, opt, mvgof->xRatioUV, mvgof->yRatioUV, bitsPerSample);
    }
}


void mvgofDeinit(MVGroupOfFrames *mvgof) {
    for (int i = 0; i < mvgof->nLevelCount; i++) {
        mvfDeinit(mvgof->frames[i]);
        free(mvgof->frames[i]);
    }

    free(mvgof->frames);
    mvgof->frames = NULL;
}


void mvgofUpdate(MVGroupOfFrames *mvgof, uint8_t **pSrc, int *pitch) {
    for (int i = 0; i < mvgof->nLevelCount; i++) {
        uint8_t *planes[3] = { NULL };

        for (int plane = 0; plane < 3; plane++) {
            if (pSrc[plane])
                planes[plane] = pSrc[plane] + PlaneSuperOffset(plane, mvgof->nHeight[plane], i, mvgof->nPel, mvgof->nVPad[plane], pitch[plane], mvgof->yRatioUV);
        }

        mvfUpdate(mvgof->frames[i], planes, pitch);
    }
}


MVFrame *mvgofGetFrame(MVGroupOfFrames *mvgof, int nLevel) {
    if ((nLevel < 0) || (nLevel >= mvgof->nLevelCount))
        return NULL;
    return mvgof->frames[nLevel];
}


void mvgofSetPlane(MVGroupOfFrames *mvgof, const uint8_t *pNewSrc, int nNewPitch, int plane) {
    mvfFillPlane(mvgof->frames[0], pNewSrc, nNewPitch, plane);
}


void mvgofRefine(MVGroupOfFrames *mvgof, MVPlaneSet nMode, int sharp) {
    mvfRefine(mvgof->frames[0], nMode, sharp);
}


void mvgofPad(MVGroupOfFrames *mvgof, MVPlaneSet nMode) {
    mvfPad(mvgof->frames[0], nMode);
}


void mvgofReduce(MVGroupOfFrames *mvgof, MVPlaneSet nMode, int rfilter) {
    for (int i = 0; i < mvgof->nLevelCount - 1; i++) {
        mvfReduceTo(mvgof->frames[i], mvgof->frames[i + 1], nMode, rfilter);
        mvfPad(mvgof->frames[i + 1], YUVPLANES);
    }
}


void mvgofResetState(MVGroupOfFrames *mvgof) {
    for (int i = 0; i < mvgof->nLevelCount; i++)
        mvfResetState(mvgof->frames[i]);
}
