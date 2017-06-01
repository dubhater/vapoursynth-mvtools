// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - global motion, overlap,  mode, refineMVs
// See legal notice in Copying.txt for more information

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

// For posix_memalign.
#define _POSIX_C_SOURCE 200112L
#include <VSHelper.h>

#include "CPU.h"
#include "PlaneOfBlocks.h"



/* fetch the block in the reference frame, which is pointed by the vector (vx, vy) */
static inline const uint8_t *pobGetRefBlock(PlaneOfBlocks *pob, int nVx, int nVy) {
    if (pob->nPel == 2)
        return mvpGetAbsolutePointerPel2(pob->pRefFrame->planes[0],
                pob->x[0] * 2 + nVx,
                pob->y[0] * 2 + nVy);
    else if (pob->nPel == 1)
        return mvpGetAbsolutePointerPel1(pob->pRefFrame->planes[0],
                pob->x[0] + nVx,
                pob->y[0] + nVy);
    else
        return mvpGetAbsolutePointerPel4(pob->pRefFrame->planes[0],
                pob->x[0] * 4 + nVx,
                pob->y[0] * 4 + nVy);
}


static inline const uint8_t *pobGetRefBlockU(PlaneOfBlocks *pob, int nVx, int nVy) {
    if (pob->nPel == 2)
        return mvpGetAbsolutePointerPel2(pob->pRefFrame->planes[1],
                pob->x[1] * 2 + nVx / pob->xRatioUV,
                pob->y[1] * 2 + nVy / pob->yRatioUV);
    else if (pob->nPel == 1)
        return mvpGetAbsolutePointerPel1(pob->pRefFrame->planes[1],
                pob->x[1] + nVx / pob->xRatioUV,
                pob->y[1] + nVy / pob->yRatioUV);
    else
        return mvpGetAbsolutePointerPel4(pob->pRefFrame->planes[1],
                pob->x[1] * 4 + nVx / pob->xRatioUV,
                pob->y[1] * 4 + nVy / pob->yRatioUV);
}


static inline const uint8_t *pobGetRefBlockV(PlaneOfBlocks *pob, int nVx, int nVy) {
    if (pob->nPel == 2)
        return mvpGetAbsolutePointerPel2(pob->pRefFrame->planes[2],
                pob->x[1] * 2 + nVx / pob->xRatioUV,
                pob->y[1] * 2 + nVy / pob->yRatioUV);
    else if (pob->nPel == 1)
        return mvpGetAbsolutePointerPel1(pob->pRefFrame->planes[2],
                pob->x[1] + nVx / pob->xRatioUV,
                pob->y[1] + nVy / pob->yRatioUV);
    else
        return mvpGetAbsolutePointerPel4(pob->pRefFrame->planes[2],
                pob->x[1] * 4 + nVx / pob->xRatioUV,
                pob->y[1] * 4 + nVy / pob->yRatioUV);
}


/* computes square distance between two vectors */
static inline unsigned int SquareDifferenceNorm(const VECTOR *v1, const int v2x, const int v2y) {
    return (v1->x - v2x) * (v1->x - v2x) + (v1->y - v2y) * (v1->y - v2y);
}


/* computes the cost of a vector (vx, vy) */
static inline int pobMotionDistorsion(PlaneOfBlocks *pob, int vx, int vy) {
    int dist = SquareDifferenceNorm(&pob->predictor, vx, vy);
    return (int)((pob->nLambda * dist) >> 8);
}


static int64_t pobLumaSAD(PlaneOfBlocks *pob, const uint8_t *pRef0) {
    int64_t sad = 0;

    if (pob->dctmode == 0) {
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
    } else if (pob->dctmode == 1) { // dct SAD
        dctBytes2D(pob->DCT, pRef0, pob->nRefPitch[0], pob->dctRef, pob->dctpitch);
        if (pob->bytesPerSample == 1)
            sad = (pob->SAD(pob->dctSrc, pob->dctpitch, pob->dctRef, pob->dctpitch) + abs(pob->dctSrc[0] - pob->dctRef[0]) * 3) * (int64_t)pob->nBlkSizeX / 2; //correct reduced DC component
        else {
            uint16_t *dctSrc16 = (uint16_t *)pob->dctSrc;
            uint16_t *dctRef16 = (uint16_t *)pob->dctRef;

            sad = (pob->SAD(pob->dctSrc, pob->dctpitch, pob->dctRef, pob->dctpitch) + abs(dctSrc16[0] - dctRef16[0]) * 3) * (int64_t)pob->nBlkSizeX / 2; //correct reduced DC component
        }
    } else if (pob->dctmode == 2) { //  globally (lumaChange) weighted spatial and DCT
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (pob->dctweight16 > 0) {
            dctBytes2D(pob->DCT, pRef0, pob->nRefPitch[0], pob->dctRef, pob->dctpitch);
            int64_t dctsad;
            if (pob->bytesPerSample == 1)
                dctsad = (pob->SAD(pob->dctSrc, pob->dctpitch, pob->dctRef, pob->dctpitch) + abs(pob->dctSrc[0] - pob->dctRef[0]) * 3) * (int64_t)pob->nBlkSizeX / 2;
            else {
                uint16_t *dctSrc16 = (uint16_t *)pob->dctSrc;
                uint16_t *dctRef16 = (uint16_t *)pob->dctRef;

                dctsad = (pob->SAD(pob->dctSrc, pob->dctpitch, pob->dctRef, pob->dctpitch) + abs(dctSrc16[0] - dctRef16[0]) * 3) * (int64_t)pob->nBlkSizeX / 2;
            }
            sad = (sad * (16 - pob->dctweight16) + dctsad * pob->dctweight16) / 16;
        }
    } else if (pob->dctmode == 3) { // per block adaptive switched from spatial to equal mixed SAD (faster)
        pob->refLuma = pob->LUMA(pRef0, pob->nRefPitch[0]);
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (abs(pob->srcLuma - pob->refLuma) > (pob->srcLuma + pob->refLuma) >> 5) {
            dctBytes2D(pob->DCT, pRef0, pob->nRefPitch[0], pob->dctRef, pob->dctpitch);
            int64_t dctsad = pob->SAD(pob->dctSrc, pob->dctpitch, pob->dctRef, pob->dctpitch) * (int64_t)pob->nBlkSizeX / 2;
            sad = sad / 2 + dctsad / 2;
        }
    } else if (pob->dctmode == 4) { //  per block adaptive switched from spatial to mixed SAD with more weight of DCT (best?)
        pob->refLuma = pob->LUMA(pRef0, pob->nRefPitch[0]);
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (abs(pob->srcLuma - pob->refLuma) > (pob->srcLuma + pob->refLuma) >> 5) {
            dctBytes2D(pob->DCT, pRef0, pob->nRefPitch[0], pob->dctRef, pob->dctpitch);
            int64_t dctsad = pob->SAD(pob->dctSrc, pob->dctpitch, pob->dctRef, pob->dctpitch) * (int64_t)pob->nBlkSizeX / 2;
            sad = sad / 4 + dctsad / 2 + dctsad / 4;
        }
    } else if (pob->dctmode == 5) { // dct SAD (SATD)
        sad = pob->SATD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
    } else if (pob->dctmode == 6) { //  globally (lumaChange) weighted spatial and DCT (better estimate)
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (pob->dctweight16 > 0) {
            int64_t dctsad = pob->SATD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
            sad = (sad * (16 - pob->dctweight16) + dctsad * pob->dctweight16) / 16;
        }
    } else if (pob->dctmode == 7) { // per block adaptive switched from spatial to equal mixed SAD (faster?)
        pob->refLuma = pob->LUMA(pRef0, pob->nRefPitch[0]);
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (abs(pob->srcLuma - pob->refLuma) > (pob->srcLuma + pob->refLuma) >> 5) {
            int64_t dctsad = pob->SATD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
            sad = sad / 2 + dctsad / 2;
        }
    } else if (pob->dctmode == 8) { //  per block adaptive switched from spatial to mixed SAD with more weight of DCT (faster?)
        pob->refLuma = pob->LUMA(pRef0, pob->nRefPitch[0]);
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (abs(pob->srcLuma - pob->refLuma) > (pob->srcLuma + pob->refLuma) >> 5) {
            int64_t dctsad = pob->SATD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
            sad = sad / 4 + dctsad / 2 + dctsad / 4;
        }
    } else if (pob->dctmode == 9) { //  globally (lumaChange) weighted spatial and DCT (better estimate, only half weight on SATD)
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (pob->dctweight16 > 1) {
            int dctweighthalf = pob->dctweight16 / 2;
            int64_t dctsad = pob->SATD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
            sad = (sad * (16 - dctweighthalf) + dctsad * dctweighthalf) / 16;
        }
    } else if (pob->dctmode == 10) { // per block adaptive switched from spatial to mixed SAD, weighted to SAD (faster)
        pob->refLuma = pob->LUMA(pRef0, pob->nRefPitch[0]);
        sad = pob->SAD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
        if (abs(pob->srcLuma - pob->refLuma) > (pob->srcLuma + pob->refLuma) >> 4) {
            int64_t dctsad = pob->SATD(pob->pSrc[0], pob->nSrcPitch[0], pRef0, pob->nRefPitch[0]);
            sad = sad / 2 + dctsad / 4 + sad / 4;
        }
    }

    return sad;
}


/* check if a vector is inside search boundaries */
static inline int pobIsVectorOK(PlaneOfBlocks *pob, int vx, int vy) {
    return ((vx >= pob->nDxMin) &&
            (vy >= pob->nDyMin) &&
            (vx < pob->nDxMax) &&
            (vy < pob->nDyMax));
}


/* check if the vector (vx, vy) is better than the best vector found so far without penalty new - renamed in v.2.11*/
static inline void pobCheckMV0(PlaneOfBlocks *pob, int vx, int vy) { //here the chance for default values are high especially for zeroMVfieldShifted (on left/top border)
    if (
        #ifdef ONLY_CHECK_NONDEFAULT_MV
            ((vx != 0) || (vy != zeroMVfieldShifted.y)) &&
            ((vx != predictor.x) || (vy != predictor.y)) &&
            ((vx != globalMVPredictor.x) || (vy != globalMVPredictor.y)) &&
        #endif
            pobIsVectorOK(pob, vx, vy)) {
        int64_t cost = pobMotionDistorsion(pob, vx, vy);
        if (cost >= pob->nMinCost)
            return;

        int64_t sad = pobLumaSAD(pob, pobGetRefBlock(pob, vx, vy));
        cost += sad;
        if (cost >= pob->nMinCost)
            return;

        int64_t saduv = 0;
        if (pob->chroma) {
            saduv += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, vx, vy), pob->nRefPitch[1]);
            saduv += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, vx, vy), pob->nRefPitch[2]);

            cost += saduv;
            if (cost >= pob->nMinCost)
                return;
        }

        pob->bestMV.x = vx;
        pob->bestMV.y = vy;
        pob->nMinCost = cost;
        pob->bestMV.sad = sad + saduv;
    }
}


/* check if the vector (vx, vy) is better than the best vector found so far */
static inline void pobCheckMV(PlaneOfBlocks *pob, int vx, int vy) { //here the chance for default values are high especially for zeroMVfieldShifted (on left/top border)
    if (
        #ifdef ONLY_CHECK_NONDEFAULT_MV
            ((vx != 0) || (vy != zeroMVfieldShifted.y)) &&
            ((vx != predictor.x) || (vy != predictor.y)) &&
            ((vx != globalMVPredictor.x) || (vy != globalMVPredictor.y)) &&
        #endif
            pobIsVectorOK(pob, vx, vy)) {
        int64_t cost = pobMotionDistorsion(pob, vx, vy);
        if (cost >= pob->nMinCost)
            return;

        int64_t sad = pobLumaSAD(pob, pobGetRefBlock(pob, vx, vy));
        cost += sad + ((pob->penaltyNew * sad) >> 8);
        if (cost >= pob->nMinCost)
            return;

        int64_t saduv = 0;
        if (pob->chroma) {
            saduv += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, vx, vy), pob->nRefPitch[1]);
            saduv += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, vx, vy), pob->nRefPitch[2]);

            cost += saduv + ((pob->penaltyNew * saduv) >> 8);
            if (cost >= pob->nMinCost)
                return;
        }

        pob->bestMV.x = vx;
        pob->bestMV.y = vy;
        pob->nMinCost = cost;
        pob->bestMV.sad = sad + saduv;
    }
}


/* check if the vector (vx, vy) is better, and update dir accordingly */
static inline void pobCheckMV2(PlaneOfBlocks *pob, int vx, int vy, int *dir, int val) {
    if (
        #ifdef ONLY_CHECK_NONDEFAULT_MV
            ((vx != 0) || (vy != zeroMVfieldShifted.y)) &&
            ((vx != predictor.x) || (vy != predictor.y)) &&
            ((vx != globalMVPredictor.x) || (vy != globalMVPredictor.y)) &&
        #endif
            pobIsVectorOK(pob, vx, vy)) {
        int64_t cost = pobMotionDistorsion(pob, vx, vy);
        if (cost >= pob->nMinCost)
            return;

        int64_t sad = pobLumaSAD(pob, pobGetRefBlock(pob, vx, vy));
        cost += sad + ((pob->penaltyNew * sad) >> 8);
        if (cost >= pob->nMinCost)
            return;

        int64_t saduv = 0;
        if (pob->chroma) {
            saduv += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, vx, vy), pob->nRefPitch[1]);
            saduv += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, vx, vy), pob->nRefPitch[2]);

            cost += saduv + ((pob->penaltyNew * saduv) >> 8);
            if (cost >= pob->nMinCost)
                return;
        }

        pob->bestMV.x = vx;
        pob->bestMV.y = vy;
        pob->nMinCost = cost;
        pob->bestMV.sad = sad + saduv;
        *dir = val;
    }
}


/* check if the vector (vx, vy) is better, and update dir accordingly, but not bestMV.x, y */
static inline void pobCheckMVdir(PlaneOfBlocks *pob, int vx, int vy, int *dir, int val) {
    if (
        #ifdef ONLY_CHECK_NONDEFAULT_MV
            ((vx != 0) || (vy != zeroMVfieldShifted.y)) &&
            ((vx != predictor.x) || (vy != predictor.y)) &&
            ((vx != globalMVPredictor.x) || (vy != globalMVPredictor.y)) &&
        #endif
            pobIsVectorOK(pob, vx, vy)) {
        int64_t cost = pobMotionDistorsion(pob, vx, vy);
        if (cost >= pob->nMinCost)
            return;

        int64_t sad = pobLumaSAD(pob, pobGetRefBlock(pob, vx, vy));
        cost += sad + ((pob->penaltyNew * sad) >> 8);
        if (cost >= pob->nMinCost)
            return;

        int64_t saduv = 0;
        if (pob->chroma) {
            saduv += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, vx, vy), pob->nRefPitch[1]);
            saduv += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, vx, vy), pob->nRefPitch[2]);

            cost += saduv + ((pob->penaltyNew * saduv) >> 8);
            if (cost >= pob->nMinCost)
                return;
        }

        pob->nMinCost = cost;
        pob->bestMV.sad = sad + saduv;
        *dir = val;
    }
}


/* clip a vector to the horizontal boundaries */
static inline int pobClipMVx(PlaneOfBlocks *pob, int vx) {
    if (vx < pob->nDxMin)
        return pob->nDxMin;
    else if (vx >= pob->nDxMax)
        return pob->nDxMax - 1;
    else
        return vx;
}


/* clip a vector to the vertical boundaries */
static inline int pobClipMVy(PlaneOfBlocks *pob, int vy) {
    if (vy < pob->nDyMin)
        return pob->nDyMin;
    else if (vy >= pob->nDyMax)
        return pob->nDyMax - 1;
    else
        return vy;
}


/* clip a vector to the search boundaries */
static inline VECTOR pobClipMV(PlaneOfBlocks *pob, VECTOR v) {
    VECTOR v2;
    v2.x = pobClipMVx(pob, v.x);
    v2.y = pobClipMVy(pob, v.y);
    v2.sad = v.sad;
    return v2;
}


/* find the median between a, b and c */
static inline int Median(int a, int b, int c) {
    if (a < b) {
        if (b < c)
            return b;
        else if (a < c)
            return c;
        else
            return a;
    } else {
        if (a < c)
            return a;
        else if (b < c)
            return c;
        else
            return b;
    }
}


void pobInit(PlaneOfBlocks *pob, int _nBlkX, int _nBlkY, int _nBlkSizeX, int _nBlkSizeY, int _nPel, int _nLevel, int nMotionFlags, int nCPUFlags, int _nOverlapX, int _nOverlapY, int _xRatioUV, int _yRatioUV, int bitsPerSample) {

    /* constant fields */

    pob->nPel = _nPel;
    pob->nLogPel = ilog2(pob->nPel);
    // nLogPel=0 for nPel=1, 1 for nPel=2, 2 for nPel=4, i.e. (x*nPel) = (x<<nLogPel)
    pob->nLogScale = _nLevel;
    pob->nScale = iexp2(pob->nLogScale);

    pob->nBlkSizeX = _nBlkSizeX;
    pob->nBlkSizeY = _nBlkSizeY;
    pob->nOverlapX = _nOverlapX;
    pob->nOverlapY = _nOverlapY;

    pob->nBlkX = _nBlkX;
    pob->nBlkY = _nBlkY;
    pob->nBlkCount = pob->nBlkX * pob->nBlkY;

    pob->xRatioUV = _xRatioUV;
    pob->yRatioUV = _yRatioUV;
    pob->nLogxRatioUV = ilog2(pob->xRatioUV);
    pob->nLogyRatioUV = ilog2(pob->yRatioUV);

    pob->bytesPerSample = (bitsPerSample + 7) / 8;

    pob->smallestPlane = !!(nMotionFlags & MOTION_SMALLEST_PLANE);
    int opt = !!(nMotionFlags & MOTION_USE_SIMD);
    pob->chroma = !!(nMotionFlags & MOTION_USE_CHROMA_MOTION);

    pob->globalMVPredictor = zeroMV;

    /* arrays memory allocation */

    pob->vectors = (VECTOR *)malloc(pob->nBlkCount * sizeof(VECTOR));
    memset(pob->vectors, 0, pob->nBlkCount * sizeof(VECTOR));

    /* function pointers initialization */
    pob->SAD = selectSADFunction(pob->nBlkSizeX, pob->nBlkSizeY, pob->bytesPerSample * 8, opt, nCPUFlags);
    pob->LUMA = selectLumaFunction(pob->nBlkSizeX, pob->nBlkSizeY, pob->bytesPerSample * 8, opt);
    pob->BLITLUMA = selectCopyFunction(pob->nBlkSizeX, pob->nBlkSizeY, pob->bytesPerSample * 8);

    pob->SADCHROMA = selectSADFunction(pob->nBlkSizeX / pob->xRatioUV, pob->nBlkSizeY / pob->yRatioUV, pob->bytesPerSample * 8, opt, nCPUFlags);
    pob->BLITCHROMA = selectCopyFunction(pob->nBlkSizeX / pob->xRatioUV, pob->nBlkSizeY / pob->yRatioUV, pob->bytesPerSample * 8);

    if (pob->nBlkSizeX == 16 && pob->nBlkSizeY == 2)
        pob->SATD = NULL;
    else
        pob->SATD = selectSATDFunction(pob->nBlkSizeX, pob->nBlkSizeY, pob->bytesPerSample * 8, opt, nCPUFlags);

    if (!pob->chroma)
        pob->SADCHROMA = NULL;


    pob->dctpitch = VSMAX(pob->nBlkSizeX, 16) * pob->bytesPerSample;

    // 64 required for effective use of x264 sad on Core2
#define ALIGN_PLANES 64

    VS_ALIGNED_MALLOC(&pob->dctSrc, pob->nBlkSizeY * pob->dctpitch, ALIGN_PLANES);
    VS_ALIGNED_MALLOC(&pob->dctRef, pob->nBlkSizeY * pob->dctpitch, ALIGN_PLANES);

    pob->nSrcPitch_temp[0] = pob->nBlkSizeX * pob->bytesPerSample;
    pob->nSrcPitch_temp[1] = pob->nBlkSizeX / pob->xRatioUV * pob->bytesPerSample;
    pob->nSrcPitch_temp[2] = pob->nSrcPitch_temp[1];

    // Four extra bytes because pixel_sad_4x4_mmx2 reads four bytes more than it should (but doesn't use them in any way).
    VS_ALIGNED_MALLOC(&pob->pSrc_temp[0], pob->nBlkSizeY * pob->nSrcPitch_temp[0] + 4, ALIGN_PLANES);
    VS_ALIGNED_MALLOC(&pob->pSrc_temp[1], pob->nBlkSizeY / pob->yRatioUV * pob->nSrcPitch_temp[1] + 4, ALIGN_PLANES);
    VS_ALIGNED_MALLOC(&pob->pSrc_temp[2], pob->nBlkSizeY / pob->yRatioUV * pob->nSrcPitch_temp[2] + 4, ALIGN_PLANES);

#undef ALIGN_PLANES

    pob->freqSize = 8192 * pob->nPel * 2; // half must be more than max vector length, which is (framewidth + Padding) * nPel
    pob->freqArray = (int *)malloc(pob->freqSize * sizeof(int));

    pob->verybigSAD = pob->nBlkSizeX * pob->nBlkSizeY * (1 << bitsPerSample);
}


void pobDeinit(PlaneOfBlocks *pob) {
    free(pob->vectors);
    free(pob->freqArray);

    VS_ALIGNED_FREE(pob->dctSrc);
    VS_ALIGNED_FREE(pob->dctRef);

    VS_ALIGNED_FREE(pob->pSrc_temp[0]);
    VS_ALIGNED_FREE(pob->pSrc_temp[1]);
    VS_ALIGNED_FREE(pob->pSrc_temp[2]);
}


static void pobWriteHeaderToArray(PlaneOfBlocks *pob, uint8_t *array) {
    MVArraySizeType size = sizeof(size) + pob->nBlkCount * sizeof(VECTOR);
    memcpy(array, &size, sizeof(size));
}


static void pobFetchPredictors(PlaneOfBlocks *pob) {
    // Left (or right) predictor
    if ((pob->blkScanDir == 1 && pob->blkx > 0) || (pob->blkScanDir == -1 && pob->blkx < pob->nBlkX - 1))
        pob->predictors[1] = pobClipMV(pob, pob->vectors[pob->blkIdx - pob->blkScanDir]);
    else
        pob->predictors[1] = pobClipMV(pob, pob->zeroMVfieldShifted); // v1.11.1 - values instead of pointer

    // Up predictor
    if (pob->blky > 0)
        pob->predictors[2] = pobClipMV(pob, pob->vectors[pob->blkIdx - pob->nBlkX]);
    else
        pob->predictors[2] = pobClipMV(pob, pob->zeroMVfieldShifted);

    // bottom-right pridictor (from coarse level)
    if ((pob->blky < pob->nBlkY - 1) && ((pob->blkScanDir == 1 && pob->blkx < pob->nBlkX - 1) || (pob->blkScanDir == -1 && pob->blkx > 0)))
        pob->predictors[3] = pobClipMV(pob, pob->vectors[pob->blkIdx + pob->nBlkX + pob->blkScanDir]);
    else
        // Up-right predictor
        if ((pob->blky > 0) && ((pob->blkScanDir == 1 && pob->blkx < pob->nBlkX - 1) || (pob->blkScanDir == -1 && pob->blkx > 0)))
            pob->predictors[3] = pobClipMV(pob, pob->vectors[pob->blkIdx - pob->nBlkX + pob->blkScanDir]);
        else
            pob->predictors[3] = pobClipMV(pob, pob->zeroMVfieldShifted);

    // Median predictor
    if (pob->blky > 0) { // replaced 1 by 0 - Fizick
        pob->predictors[0].x = Median(pob->predictors[1].x, pob->predictors[2].x, pob->predictors[3].x);
        pob->predictors[0].y = Median(pob->predictors[1].y, pob->predictors[2].y, pob->predictors[3].y);
        //      predictors[0].sad = Median(predictors[1].sad, predictors[2].sad, predictors[3].sad);
        // but it is not true median vector (x and y may be mixed) and not its sad ?!
        // we really do not know SAD, here is more safe estimation especially for phaseshift method - v1.6.0
        pob->predictors[0].sad = VSMAX(pob->predictors[1].sad, VSMAX(pob->predictors[2].sad, pob->predictors[3].sad));
    } else {
        //		predictors[0].x = (predictors[1].x + predictors[2].x + predictors[3].x);
        //		predictors[0].y = (predictors[1].y + predictors[2].y + predictors[3].y);
        //      predictors[0].sad = (predictors[1].sad + predictors[2].sad + predictors[3].sad);
        // but for top line we have only left predictor[1] - v1.6.0
        pob->predictors[0] = pob->predictors[1];
    }

    // if there are no other planes, predictor is the median
    if (pob->smallestPlane)
        pob->predictor = pob->predictors[0];
    pob->nLambda = pob->nLambda * pob->LSAD / (pob->LSAD + (pob->predictor.sad >> 1)) * pob->LSAD / (pob->LSAD + (pob->predictor.sad >> 1));
}


static void pobNStepSearch(PlaneOfBlocks *pob, int stp) {
    int dx, dy;
    int length = stp;
    while (length > 0) {
        dx = pob->bestMV.x;
        dy = pob->bestMV.y;

        pobCheckMV(pob, dx + length, dy + length);
        pobCheckMV(pob, dx + length, dy);
        pobCheckMV(pob, dx + length, dy - length);
        pobCheckMV(pob, dx, dy - length);
        pobCheckMV(pob, dx, dy + length);
        pobCheckMV(pob, dx - length, dy + length);
        pobCheckMV(pob, dx - length, dy);
        pobCheckMV(pob, dx - length, dy - length);

        length--;
    }
}


static void pobOneTimeSearch(PlaneOfBlocks *pob, int length) {
    int direction = 0;
    int dx = pob->bestMV.x;
    int dy = pob->bestMV.y;

    pobCheckMV2(pob, dx - length, dy, &direction, 2);
    pobCheckMV2(pob, dx + length, dy, &direction, 1);

    if (direction == 1) {
        while (direction) {
            direction = 0;
            dx += length;
            pobCheckMV2(pob, dx + length, dy, &direction, 1);
        }
    } else if (direction == 2) {
        while (direction) {
            direction = 0;
            dx -= length;
            pobCheckMV2(pob, dx - length, dy, &direction, 1);
        }
    }

    pobCheckMV2(pob, dx, dy - length, &direction, 2);
    pobCheckMV2(pob, dx, dy + length, &direction, 1);

    if (direction == 1) {
        while (direction) {
            direction = 0;
            dy += length;
            pobCheckMV2(pob, dx, dy + length, &direction, 1);
        }
    } else if (direction == 2) {
        while (direction) {
            direction = 0;
            dy -= length;
            pobCheckMV2(pob, dx, dy - length, &direction, 1);
        }
    }
}


static void pobDiamondSearch(PlaneOfBlocks *pob, int length) {
    enum Direction {
        Right = 1,
        Left = 2,
        Down = 4,
        Up = 8,
    };

    int dx;
    int dy;

    // We begin by making no assumption on which direction to search.
    int direction = 15;

    int lastDirection;

    while (direction > 0) {
        dx = pob->bestMV.x;
        dy = pob->bestMV.y;
        lastDirection = direction;
        direction = 0;

        // First, we look the directions that were hinted by the previous step
        // of the algorithm. If we find one, we add it to the set of directions
        // we'll test next
        if (lastDirection & Right)
            pobCheckMV2(pob, dx + length, dy, &direction, Right);
        if (lastDirection & Left)
            pobCheckMV2(pob, dx - length, dy, &direction, Left);
        if (lastDirection & Down)
            pobCheckMV2(pob, dx, dy + length, &direction, Down);
        if (lastDirection & Up)
            pobCheckMV2(pob, dx, dy - length, &direction, Up);

        // If one of the directions improves the SAD, we make further tests
        // on the diagonals
        if (direction) {
            lastDirection = direction;
            dx = pob->bestMV.x;
            dy = pob->bestMV.y;

            if (lastDirection & (Right + Left)) {
                pobCheckMV2(pob, dx, dy + length, &direction, Down);
                pobCheckMV2(pob, dx, dy - length, &direction, Up);
            } else {
                pobCheckMV2(pob, dx + length, dy, &direction, Right);
                pobCheckMV2(pob, dx - length, dy, &direction, Left);
            }
        }

        // If not, we do not stop here. We infer from the last direction the
        // diagonals to be checked, because we might be lucky.
        else {
            switch (lastDirection) {
                case Right:
                    pobCheckMV2(pob, dx + length, dy + length, &direction, Right + Down);
                    pobCheckMV2(pob, dx + length, dy - length, &direction, Right + Up);
                    break;
                case Left:
                    pobCheckMV2(pob, dx - length, dy + length, &direction, Left + Down);
                    pobCheckMV2(pob, dx - length, dy - length, &direction, Left + Up);
                    break;
                case Down:
                    pobCheckMV2(pob, dx + length, dy + length, &direction, Right + Down);
                    pobCheckMV2(pob, dx - length, dy + length, &direction, Left + Down);
                    break;
                case Up:
                    pobCheckMV2(pob, dx + length, dy - length, &direction, Right + Up);
                    pobCheckMV2(pob, dx - length, dy - length, &direction, Left + Up);
                    break;
                case Right + Down:
                    pobCheckMV2(pob, dx + length, dy + length, &direction, Right + Down);
                    pobCheckMV2(pob, dx - length, dy + length, &direction, Left + Down);
                    pobCheckMV2(pob, dx + length, dy - length, &direction, Right + Up);
                    break;
                case Left + Down:
                    pobCheckMV2(pob, dx + length, dy + length, &direction, Right + Down);
                    pobCheckMV2(pob, dx - length, dy + length, &direction, Left + Down);
                    pobCheckMV2(pob, dx - length, dy - length, &direction, Left + Up);
                    break;
                case Right + Up:
                    pobCheckMV2(pob, dx + length, dy + length, &direction, Right + Down);
                    pobCheckMV2(pob, dx - length, dy - length, &direction, Left + Up);
                    pobCheckMV2(pob, dx + length, dy - length, &direction, Right + Up);
                    break;
                case Left + Up:
                    pobCheckMV2(pob, dx - length, dy - length, &direction, Left + Up);
                    pobCheckMV2(pob, dx - length, dy + length, &direction, Left + Down);
                    pobCheckMV2(pob, dx + length, dy - length, &direction, Right + Up);
                    break;
                default:
                    // Even the default case may happen, in the first step of the
                    // algorithm for example.
                    pobCheckMV2(pob, dx + length, dy + length, &direction, Right + Down);
                    pobCheckMV2(pob, dx - length, dy + length, &direction, Left + Down);
                    pobCheckMV2(pob, dx + length, dy - length, &direction, Right + Up);
                    pobCheckMV2(pob, dx - length, dy - length, &direction, Left + Up);
                    break;
            }
        }
    }
}


static void pobExpandingSearch(PlaneOfBlocks *pob, int r, int s, int mvx, int mvy) {
    // diameter = 2*r + 1, step=s
    // part of true enhaustive search (thin expanding square) around mvx, mvy
    int i, j;

    // sides of square without corners
    for (i = -r + s; i < r; i += s) // without corners! - v2.1
    {
        pobCheckMV(pob, mvx + i, mvy - r);
        pobCheckMV(pob, mvx + i, mvy + r);
    }

    for (j = -r + s; j < r; j += s) {
        pobCheckMV(pob, mvx - r, mvy + j);
        pobCheckMV(pob, mvx + r, mvy + j);
    }

    // then corners - they are more far from cenrer
    pobCheckMV(pob, mvx - r, mvy - r);
    pobCheckMV(pob, mvx - r, mvy + r);
    pobCheckMV(pob, mvx + r, mvy - r);
    pobCheckMV(pob, mvx + r, mvy + r);
}


/* (x-1)%6 */
static const int mod6m1[8] = { 5, 0, 1, 2, 3, 4, 5, 0 };
/* radius 2 hexagon. repeated entries are to avoid having to compute mod6 every time. */
static const int hex2[8][2] = { { -1, -2 }, { -2, 0 }, { -1, 2 }, { 1, 2 }, { 2, 0 }, { 1, -2 }, { -1, -2 }, { -2, 0 } };

static void pobHex2Search(PlaneOfBlocks *pob, int i_me_range) { //adopted from x264
    int dir = -2;
    int bmx = pob->bestMV.x;
    int bmy = pob->bestMV.y;

    if (i_me_range > 1) {
        /* hexagon */
        //        COST_MV_X3_DIR( -2,0, -1, 2,  1, 2, costs   );
        //        COST_MV_X3_DIR(  2,0,  1,-2, -1,-2, costs+3 );
        //        COPY2_IF_LT( bcost, costs[0], dir, 0 );
        //        COPY2_IF_LT( bcost, costs[1], dir, 1 );
        //        COPY2_IF_LT( bcost, costs[2], dir, 2 );
        //        COPY2_IF_LT( bcost, costs[3], dir, 3 );
        //        COPY2_IF_LT( bcost, costs[4], dir, 4 );
        //        COPY2_IF_LT( bcost, costs[5], dir, 5 );
        pobCheckMVdir(pob, bmx - 2, bmy, &dir, 0);
        pobCheckMVdir(pob, bmx - 1, bmy + 2, &dir, 1);
        pobCheckMVdir(pob, bmx + 1, bmy + 2, &dir, 2);
        pobCheckMVdir(pob, bmx + 2, bmy, &dir, 3);
        pobCheckMVdir(pob, bmx + 1, bmy - 2, &dir, 4);
        pobCheckMVdir(pob, bmx - 1, bmy - 2, &dir, 5);


        if (dir != -2) {
            bmx += hex2[dir + 1][0];
            bmy += hex2[dir + 1][1];
            /* half hexagon, not overlapping the previous iteration */
            for (int i = 1; i < i_me_range / 2 && pobIsVectorOK(pob, bmx, bmy); i++) {
                const int odir = mod6m1[dir + 1];
                //                COST_MV_X3_DIR( hex2[odir+0][0], hex2[odir+0][1],
                //                                hex2[odir+1][0], hex2[odir+1][1],
                //                                hex2[odir+2][0], hex2[odir+2][1],
                //                                costs );

                dir = -2;
                //                COPY2_IF_LT( bcost, costs[0], dir, odir-1 );
                //                COPY2_IF_LT( bcost, costs[1], dir, odir   );
                //                COPY2_IF_LT( bcost, costs[2], dir, odir+1 );

                pobCheckMVdir(pob, bmx + hex2[odir + 0][0], bmy + hex2[odir + 0][1], &dir, odir - 1);
                pobCheckMVdir(pob, bmx + hex2[odir + 1][0], bmy + hex2[odir + 1][1], &dir, odir);
                pobCheckMVdir(pob, bmx + hex2[odir + 2][0], bmy + hex2[odir + 2][1], &dir, odir + 1);
                if (dir == -2)
                    break;
                bmx += hex2[dir + 1][0];
                bmy += hex2[dir + 1][1];
            }
        }

        pob->bestMV.x = bmx;
        pob->bestMV.y = bmy;
    }
    /* square refine */
    //        omx = bmx; omy = bmy;
    //        COST_MV_X4(  0,-1,  0,1, -1,0, 1,0 );
    //        COST_MV_X4( -1,-1, -1,1, 1,-1, 1,1 );
    pobExpandingSearch(pob, 1, 1, bmx, bmy);
}


static void pobCrossSearch(PlaneOfBlocks *pob, int start, int x_max, int y_max, int mvx, int mvy) { // part of umh  search

    for (int i = start; i < x_max; i += 2) {
        pobCheckMV(pob, mvx - i, mvy);
        pobCheckMV(pob, mvx + i, mvy);
    }

    for (int j = start; j < y_max; j += 2) {
        pobCheckMV(pob, mvx, mvy + j);
        pobCheckMV(pob, mvx, mvy + j);
    }
}


static void pobUMHSearch(PlaneOfBlocks *pob, int i_me_range, int omx, int omy) { // radius
    // Uneven-cross Multi-Hexagon-grid Search (see x264)
    /* hexagon grid */

    //            int omx = bestMV.x;
    //            int omy = bestMV.y;
    // my mod: do not shift the center after Cross
    pobCrossSearch(pob, 1, i_me_range, i_me_range, omx, omy);


    int i = 1;
    do {
        static const int hex4[16][2] = {
            { -4, 2 }, { -4, 1 }, { -4, 0 }, { -4, -1 }, { -4, -2 }, { 4, -2 }, { 4, -1 }, { 4, 0 }, { 4, 1 }, { 4, 2 }, { 2, 3 }, { 0, 4 }, { -2, 3 }, { -2, -3 }, { 0, -4 }, { 2, -3 },
        };

        for (int j = 0; j < 16; j++) {
            int mx = omx + hex4[j][0] * i;
            int my = omy + hex4[j][1] * i;
            pobCheckMV(pob, mx, my);
        }
    } while (++i <= i_me_range / 4);

    //            if( bmy <= mv_y_max )
    //                goto me_hex2;
    pobHex2Search(pob, i_me_range);
}


static void pobRefine(PlaneOfBlocks *pob) {
    // then, we refine, according to the search type
    if (pob->searchType == SearchOnetime)
        for (int i = pob->nSearchParam; i > 0; i /= 2)
            pobOneTimeSearch(pob, i);

    if (pob->searchType == SearchNstep)
        pobNStepSearch(pob, pob->nSearchParam);

    if (pob->searchType == SearchLogarithmic)
        for (int i = pob->nSearchParam; i > 0; i /= 2)
            pobDiamondSearch(pob, i);

    if (pob->searchType == SearchExhaustive) {
        int mvx = pob->bestMV.x;
        int mvy = pob->bestMV.y;
        for (int i = 1; i <= pob->nSearchParam; i++) // region is same as enhausted, but ordered by radius (from near to far)
            pobExpandingSearch(pob, i, 1, mvx, mvy);
    }

    if (pob->searchType == SearchHex2)
        pobHex2Search(pob, pob->nSearchParam);

    if (pob->searchType == SearchUnevenMultiHexagon)
        pobUMHSearch(pob, pob->nSearchParam, pob->bestMV.x, pob->bestMV.y);

    if (pob->searchType == SearchHorizontal) {
        int mvx = pob->bestMV.x;
        int mvy = pob->bestMV.y;
        for (int i = 1; i <= pob->nSearchParam; i++) {
            pobCheckMV(pob, mvx - i, mvy);
            pobCheckMV(pob, mvx + i, mvy);
        }
    }

    if (pob->searchType == SearchVertical) {
        int mvx = pob->bestMV.x;
        int mvy = pob->bestMV.y;
        for (int i = 1; i <= pob->nSearchParam; i++) {
            pobCheckMV(pob, mvx, mvy - i);
            pobCheckMV(pob, mvx, mvy + i);
        }
    }
}


static void pobPseudoEPZSearch(PlaneOfBlocks *pob) {

    pobFetchPredictors(pob);

    if (pob->dctmode != 0) { // DCT method (luma only - currently use normal spatial SAD chroma)
        // make dct of source block
        if (pob->dctmode <= 4) //don't do the slow dct conversion if SATD used
            dctBytes2D(pob->DCT, pob->pSrc[0], pob->nSrcPitch[0], pob->dctSrc, pob->dctpitch);
    }
    if (pob->dctmode >= 3) // most use it and it should be fast anyway //if (dctmode == 3 || dctmode == 4) // check it
        pob->srcLuma = pob->LUMA(pob->pSrc[0], pob->nSrcPitch[0]);

    // We treat zero alone
    // Do we bias zero with not taking into account distorsion ?
    pob->bestMV.x = pob->zeroMVfieldShifted.x;
    pob->bestMV.y = pob->zeroMVfieldShifted.y;
    int64_t sad = pobLumaSAD(pob, pobGetRefBlock(pob, 0, pob->zeroMVfieldShifted.y));
    if (pob->chroma) {
        sad += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, 0, 0), pob->nRefPitch[1]);
        sad += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, 0, 0), pob->nRefPitch[2]);
    }
    pob->bestMV.sad = sad;
    pob->nMinCost = sad + ((pob->penaltyZero * sad) >> 8); // v.1.11.0.2

    VECTOR bestMVMany[8];
    int64_t nMinCostMany[8];

    if (pob->tryMany) {
        //  refine around zero
        pobRefine(pob);
        bestMVMany[0] = pob->bestMV; // save bestMV
        nMinCostMany[0] = pob->nMinCost;
    }

    // Global MV predictor  - added by Fizick
    pob->globalMVPredictor = pobClipMV(pob, pob->globalMVPredictor);
    sad = pobLumaSAD(pob, pobGetRefBlock(pob, pob->globalMVPredictor.x, pob->globalMVPredictor.y));
    if (pob->chroma) {
        sad += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, pob->globalMVPredictor.x, pob->globalMVPredictor.y), pob->nRefPitch[1]);
        sad += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, pob->globalMVPredictor.x, pob->globalMVPredictor.y), pob->nRefPitch[2]);
    }
    int64_t cost = sad + ((pob->pglobal * sad) >> 8);

    if (cost < pob->nMinCost || pob->tryMany) {
        pob->bestMV.x = pob->globalMVPredictor.x;
        pob->bestMV.y = pob->globalMVPredictor.y;
        pob->bestMV.sad = sad;
        pob->nMinCost = cost;
    }
    if (pob->tryMany) {
        // refine around global
        pobRefine(pob);               // reset bestMV
        bestMVMany[1] = pob->bestMV; // save bestMV
        nMinCostMany[1] = pob->nMinCost;
    }
    sad = pobLumaSAD(pob, pobGetRefBlock(pob, pob->predictor.x, pob->predictor.y));
    if (pob->chroma) {
        sad += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, pob->predictor.x, pob->predictor.y), pob->nRefPitch[1]);
        sad += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, pob->predictor.x, pob->predictor.y), pob->nRefPitch[2]);
    }
    cost = sad;

    if (cost < pob->nMinCost || pob->tryMany) {
        pob->bestMV.x = pob->predictor.x;
        pob->bestMV.y = pob->predictor.y;
        pob->bestMV.sad = sad;
        pob->nMinCost = cost;
    }
    if (pob->tryMany) {
        // refine around predictor
        pobRefine(pob);               // reset bestMV
        bestMVMany[2] = pob->bestMV; // save bestMV
        nMinCostMany[2] = pob->nMinCost;
    }

    // then all the other predictors
    int npred = 4;

    for (int i = 0; i < npred; i++) {
        if (pob->tryMany)
            pob->nMinCost = pob->verybigSAD + 1;
        pobCheckMV0(pob, pob->predictors[i].x, pob->predictors[i].y);
        if (pob->tryMany) {
            // refine around predictor
            pobRefine(pob);                   // reset bestMV
            bestMVMany[i + 3] = pob->bestMV; // save bestMV
            nMinCostMany[i + 3] = pob->nMinCost;
        }
    }


    if (pob->tryMany) { // select best of multi best
        pob->nMinCost = pob->verybigSAD + 1;
        for (int i = 0; i < npred + 3; i++) {
            if (nMinCostMany[i] < pob->nMinCost) {
                pob->bestMV = bestMVMany[i];
                pob->nMinCost = nMinCostMany[i];
            }
        }
    } else {
        // then, we refine, according to the search type
        pobRefine(pob);
    }

    int64_t foundSAD = pob->bestMV.sad;

#define BADCOUNT_LIMIT 16

    if (pob->blkIdx > 1 && foundSAD > (pob->badSAD + pob->badSAD * pob->badcount / BADCOUNT_LIMIT)) {
        // bad vector, try wide search
        // with some soft limit (BADCOUNT_LIMIT) of bad cured vectors (time consumed)
        pob->badcount++;

        if (pob->badrange > 0) { // UMH
            // rathe good is not found, lets try around zero
            pobUMHSearch(pob, pob->badrange * pob->nPel, 0, 0);
        } else if (pob->badrange < 0) { // ESA
            for (int i = 1; i < -pob->badrange * pob->nPel; i += pob->nPel) { // at radius
                pobExpandingSearch(pob, i, pob->nPel, 0, 0);
                if (pob->bestMV.sad < foundSAD / 4)
                    break; // stop search if rathe good is found
            }
        }

        int mvx = pob->bestMV.x; // refine in small area
        int mvy = pob->bestMV.y;
        for (int i = 1; i < pob->nPel; i++) { // small radius
            pobExpandingSearch(pob, i, 1, mvx, mvy);
        }
    }


    // we store the result
    pob->vectors[pob->blkIdx] = pob->bestMV;
}


void pobSearchMVs(PlaneOfBlocks *pob, MVFrame *pSrcFrame, MVFrame *pRefFrame,
                  SearchType st, int stp, int lambda, int lsad, int pnew,
                  int plevel, uint8_t *out, VECTOR *globalMVec,
                  int fieldShift, DCTFFTW *DCT, int dctmode, int *pmeanLumaChange,
                  int pzero, int pglobal, int64_t badSAD, int badrange, int meander, int tryMany) {
    pob->DCT = DCT;
    if (pob->DCT == 0)
        pob->dctmode = 0;
    else
        pob->dctmode = dctmode;
    pob->dctweight16 = VSMIN(16, abs(*pmeanLumaChange) / (pob->nBlkSizeX * pob->nBlkSizeY)); //equal dct and spatial weights for meanLumaChange=8 (empirical)
    pob->badSAD = badSAD;
    pob->badrange = badrange;
    pob->zeroMVfieldShifted.x = 0;
    pob->zeroMVfieldShifted.y = fieldShift;
    pob->zeroMVfieldShifted.sad = 0;
    pob->globalMVPredictor.x = pob->nPel * globalMVec->x; // v1.8.2
    pob->globalMVPredictor.y = pob->nPel * globalMVec->y + fieldShift;
    pob->globalMVPredictor.sad = globalMVec->sad;

    // write the plane's header
    pobWriteHeaderToArray(pob, out);

    VECTOR *pBlkData = (VECTOR *)(out + sizeof(MVArraySizeType));

    pob->pSrcFrame = pSrcFrame;
    pob->pRefFrame = pRefFrame;


    pob->y[0] = pob->pSrcFrame->planes[0]->nVPadding;

    if (pob->pSrcFrame->nMode & UPLANE) {
        pob->y[1] = pob->pSrcFrame->planes[1]->nVPadding;
    }
    if (pob->pSrcFrame->nMode & VPLANE) {
        pob->y[2] = pob->pSrcFrame->planes[2]->nVPadding;
    }

    pob->nSrcPitch[0] = pob->pSrcFrame->planes[0]->nPitch;
    if (pob->chroma) {
        pob->nSrcPitch[1] = pob->pSrcFrame->planes[1]->nPitch;
        pob->nSrcPitch[2] = pob->pSrcFrame->planes[2]->nPitch;
    }

    pob->nRefPitch[0] = pob->pRefFrame->planes[0]->nPitch;
    if (pob->chroma) {
        pob->nRefPitch[1] = pob->pRefFrame->planes[1]->nPitch;
        pob->nRefPitch[2] = pob->pRefFrame->planes[2]->nPitch;
    }

    pob->searchType = st;    //( nLogScale == 0 ) ? st : EXHAUSTIVE;
    pob->nSearchParam = stp; //*nPel; // v1.8.2 - redesigned in v1.8.5

    int nLambdaLevel = lambda / (pob->nPel * pob->nPel);
    if (plevel == 1)
        nLambdaLevel = nLambdaLevel * pob->nScale; // scale lambda - Fizick
    else if (plevel == 2)
        nLambdaLevel = nLambdaLevel * pob->nScale * pob->nScale;

    pob->penaltyZero = pzero;
    pob->pglobal = pglobal;
    pob->badcount = 0;
    pob->tryMany = tryMany;
    // Functions using float must not be used here

    for (pob->blky = 0; pob->blky < pob->nBlkY; pob->blky++) {
        pob->blkScanDir = (pob->blky % 2 == 0 || meander == 0) ? 1 : -1;
        // meander (alternate) scan blocks (even row left to right, odd row right to left)
        int blkxStart = (pob->blky % 2 == 0 || meander == 0) ? 0 : pob->nBlkX - 1;
        if (pob->blkScanDir == 1) { // start with leftmost block
            pob->x[0] = pob->pSrcFrame->planes[0]->nHPadding;
            if (pob->chroma) {
                pob->x[1] = pob->pSrcFrame->planes[1]->nHPadding;
                pob->x[2] = pob->pSrcFrame->planes[2]->nHPadding;
            }
        } else { // start with rightmost block, but it is already set at prev row
            pob->x[0] = pob->pSrcFrame->planes[0]->nHPadding + (pob->nBlkSizeX - pob->nOverlapX) * (pob->nBlkX - 1);
            if (pob->chroma) {
                pob->x[1] = pob->pSrcFrame->planes[1]->nHPadding + ((pob->nBlkSizeX - pob->nOverlapX) / pob->xRatioUV) * (pob->nBlkX - 1);
                pob->x[2] = pob->pSrcFrame->planes[2]->nHPadding + ((pob->nBlkSizeX - pob->nOverlapX) / pob->xRatioUV) * (pob->nBlkX - 1);
            }
        }
        for (int iblkx = 0; iblkx < pob->nBlkX; iblkx++) {
            pob->blkx = blkxStart + iblkx * pob->blkScanDir;
            pob->blkIdx = pob->blky * pob->nBlkX + pob->blkx;

            pob->pSrc[0] = mvpGetAbsolutePelPointer(pob->pSrcFrame->planes[0], pob->x[0], pob->y[0]);
            if (pob->chroma) {
                pob->pSrc[1] = mvpGetAbsolutePelPointer(pob->pSrcFrame->planes[1], pob->x[1], pob->y[1]);
                pob->pSrc[2] = mvpGetAbsolutePelPointer(pob->pSrcFrame->planes[2], pob->x[2], pob->y[2]);
            }

            pob->nSrcPitch[0] = pob->pSrcFrame->planes[0]->nPitch;
            //create aligned copy
            pob->BLITLUMA(pob->pSrc_temp[0], pob->nSrcPitch_temp[0], pob->pSrc[0], pob->nSrcPitch[0]);
            //set the to the aligned copy
            pob->pSrc[0] = pob->pSrc_temp[0];
            pob->nSrcPitch[0] = pob->nSrcPitch_temp[0];
            if (pob->chroma) {
                pob->nSrcPitch[1] = pob->pSrcFrame->planes[1]->nPitch;
                pob->nSrcPitch[2] = pob->pSrcFrame->planes[2]->nPitch;
                pob->BLITCHROMA(pob->pSrc_temp[1], pob->nSrcPitch_temp[1], pob->pSrc[1], pob->nSrcPitch[1]);
                pob->BLITCHROMA(pob->pSrc_temp[2], pob->nSrcPitch_temp[2], pob->pSrc[2], pob->nSrcPitch[2]);
                pob->pSrc[1] = pob->pSrc_temp[1];
                pob->pSrc[2] = pob->pSrc_temp[2];
                pob->nSrcPitch[1] = pob->nSrcPitch_temp[1];
                pob->nSrcPitch[2] = pob->nSrcPitch_temp[2];
            }

            if (pob->blky == 0)
                pob->nLambda = 0;
            else
                pob->nLambda = nLambdaLevel;

            pob->penaltyNew = pnew; // penalty for new vector
            pob->LSAD = lsad;       // SAD limit for lambda using
            // may be they must be scaled by nPel ?

            // decreased padding of coarse levels
            int nHPaddingScaled = pob->pSrcFrame->planes[0]->nHPadding >> pob->nLogScale;
            int nVPaddingScaled = pob->pSrcFrame->planes[0]->nVPadding >> pob->nLogScale;
            /* computes search boundaries */
            pob->nDxMax = pob->nPel * (pob->pSrcFrame->planes[0]->nPaddedWidth - pob->x[0] - pob->nBlkSizeX - pob->pSrcFrame->planes[0]->nHPadding + nHPaddingScaled);
            pob->nDyMax = pob->nPel * (pob->pSrcFrame->planes[0]->nPaddedHeight - pob->y[0] - pob->nBlkSizeY - pob->pSrcFrame->planes[0]->nVPadding + nVPaddingScaled);
            pob->nDxMin = -pob->nPel * (pob->x[0] - pob->pSrcFrame->planes[0]->nHPadding + nHPaddingScaled);
            pob->nDyMin = -pob->nPel * (pob->y[0] - pob->pSrcFrame->planes[0]->nVPadding + nVPaddingScaled);

            /* search the mv */
            pob->predictor = pobClipMV(pob, pob->vectors[pob->blkIdx]);
            pob->predictors[4] = pobClipMV(pob, zeroMV);

            pobPseudoEPZSearch(pob);

            /* write the results */
            pBlkData[pob->blkx] = pob->bestMV;


            if (pob->smallestPlane)
                pob->sumLumaChange += pob->LUMA(pobGetRefBlock(pob, 0, 0), pob->nRefPitch[0]) - pob->LUMA(pob->pSrc[0], pob->nSrcPitch[0]);

            /* increment indexes & pointers */
            if (iblkx < pob->nBlkX - 1) {
                pob->x[0] += (pob->nBlkSizeX - pob->nOverlapX) * pob->blkScanDir;
                if (pob->pSrcFrame->nMode & UPLANE)
                    pob->x[1] += ((pob->nBlkSizeX - pob->nOverlapX) >> pob->nLogxRatioUV) * pob->blkScanDir;
                if (pob->pSrcFrame->nMode & VPLANE)
                    pob->x[2] += ((pob->nBlkSizeX - pob->nOverlapX) >> pob->nLogxRatioUV) * pob->blkScanDir;
            }
        }
        pBlkData += pob->nBlkX;

        pob->y[0] += (pob->nBlkSizeY - pob->nOverlapY);
        if (pob->pSrcFrame->nMode & UPLANE)
            pob->y[1] += ((pob->nBlkSizeY - pob->nOverlapY) >> pob->nLogyRatioUV);
        if (pob->pSrcFrame->nMode & VPLANE)
            pob->y[2] += ((pob->nBlkSizeY - pob->nOverlapY) >> pob->nLogyRatioUV);
    }
    if (pob->smallestPlane)
        *pmeanLumaChange = pob->sumLumaChange / pob->nBlkCount; // for all finer planes
}


void pobRecalculateMVs(PlaneOfBlocks *pob, const FakeGroupOfPlanes *fgop, MVFrame *pSrcFrame, MVFrame *pRefFrame,
                       SearchType st, int stp, int lambda, int pnew, uint8_t *out,
                       int fieldShift, int64_t thSAD, DCTFFTW *DCT, int dctmode, int smooth, int meander) {
    pob->DCT = DCT;
    if (pob->DCT == 0)
        pob->dctmode = 0;
    else
        pob->dctmode = dctmode;
    pob->dctweight16 = 8; //min(16,abs(*pmeanLumaChange)/(nBlkSizeX*nBlkSizeY)); //equal dct and spatial weights for meanLumaChange=8 (empirical)
    pob->zeroMVfieldShifted.x = 0;
    pob->zeroMVfieldShifted.y = fieldShift;
    pob->globalMVPredictor.x = 0;          //nPel*globalMVec->x;// there is no global
    pob->globalMVPredictor.y = fieldShift; //nPel*globalMVec->y + fieldShift;
    pob->globalMVPredictor.sad = 9999999;  //globalMVec->sad;

    // write the plane's header
    pobWriteHeaderToArray(pob, out);

    VECTOR *pBlkData = (VECTOR *)(out + sizeof(MVArraySizeType));

    pob->pSrcFrame = pSrcFrame;
    pob->pRefFrame = pRefFrame;

    pob->x[0] = pob->pSrcFrame->planes[0]->nHPadding;
    pob->y[0] = pob->pSrcFrame->planes[0]->nVPadding;
    if (pob->chroma) {
        pob->x[1] = pob->pSrcFrame->planes[1]->nHPadding;
        pob->x[2] = pob->pSrcFrame->planes[2]->nHPadding;
        pob->y[1] = pob->pSrcFrame->planes[1]->nVPadding;
        pob->y[2] = pob->pSrcFrame->planes[2]->nVPadding;
    }

    pob->nSrcPitch[0] = pob->pSrcFrame->planes[0]->nPitch;
    if (pob->chroma) {
        pob->nSrcPitch[1] = pob->pSrcFrame->planes[1]->nPitch;
        pob->nSrcPitch[2] = pob->pSrcFrame->planes[2]->nPitch;
    }

    pob->nRefPitch[0] = pob->pRefFrame->planes[0]->nPitch;
    if (pob->chroma) {
        pob->nRefPitch[1] = pob->pRefFrame->planes[1]->nPitch;
        pob->nRefPitch[2] = pob->pRefFrame->planes[2]->nPitch;
    }

    pob->searchType = st;
    pob->nSearchParam = stp; //*nPel; // v1.8.2 - redesigned in v1.8.5

    int nLambdaLevel = lambda / (pob->nPel * pob->nPel);

    // get old vectors plane
    const FakePlaneOfBlocks *plane = fgopGetPlane(fgop, 0);
    int nBlkXold = plane->nBlkX;
    int nBlkYold = plane->nBlkY;
    int nBlkSizeXold = plane->nBlkSizeX;
    int nBlkSizeYold = plane->nBlkSizeY;
    int nOverlapXold = plane->nOverlapX;
    int nOverlapYold = plane->nOverlapY;
    int nStepXold = nBlkSizeXold - nOverlapXold;
    int nStepYold = nBlkSizeYold - nOverlapYold;
    int nPelold = plane->nPel;
    int nLogPelold = ilog2(nPelold);

    // Functions using float must not be used here
    for (pob->blky = 0; pob->blky < pob->nBlkY; pob->blky++) {
        pob->blkScanDir = (pob->blky % 2 == 0 || meander == 0) ? 1 : -1;
        // meander (alternate) scan blocks (even row left to right, odd row right to left)
        int blkxStart = (pob->blky % 2 == 0 || meander == 0) ? 0 : pob->nBlkX - 1;
        if (pob->blkScanDir == 1) { // start with leftmost block
            pob->x[0] = pob->pSrcFrame->planes[0]->nHPadding;
            if (pob->chroma) {
                pob->x[1] = pob->pSrcFrame->planes[1]->nHPadding;
                pob->x[2] = pob->pSrcFrame->planes[2]->nHPadding;
            }
        } else { // start with rightmost block, but it is already set at prev row
            pob->x[0] = pob->pSrcFrame->planes[0]->nHPadding + (pob->nBlkSizeX - pob->nOverlapX) * (pob->nBlkX - 1);
            if (pob->chroma) {
                pob->x[1] = pob->pSrcFrame->planes[1]->nHPadding + ((pob->nBlkSizeX - pob->nOverlapX) / pob->xRatioUV) * (pob->nBlkX - 1);
                pob->x[2] = pob->pSrcFrame->planes[2]->nHPadding + ((pob->nBlkSizeX - pob->nOverlapX) / pob->xRatioUV) * (pob->nBlkX - 1);
            }
        }
        for (int iblkx = 0; iblkx < pob->nBlkX; iblkx++) {
            pob->blkx = blkxStart + iblkx * pob->blkScanDir;
            pob->blkIdx = pob->blky * pob->nBlkX + pob->blkx;

            pob->pSrc[0] = mvpGetAbsolutePelPointer(pob->pSrcFrame->planes[0], pob->x[0], pob->y[0]);
            if (pob->chroma) {
                pob->pSrc[1] = mvpGetAbsolutePelPointer(pob->pSrcFrame->planes[1], pob->x[1], pob->y[1]);
                pob->pSrc[2] = mvpGetAbsolutePelPointer(pob->pSrcFrame->planes[2], pob->x[2], pob->y[2]);
            }

            pob->nSrcPitch[0] = pob->pSrcFrame->planes[0]->nPitch;
            //create aligned copy
            pob->BLITLUMA(pob->pSrc_temp[0], pob->nSrcPitch_temp[0], pob->pSrc[0], pob->nSrcPitch[0]);
            //set the to the aligned copy
            pob->pSrc[0] = pob->pSrc_temp[0];
            pob->nSrcPitch[0] = pob->nSrcPitch_temp[0];
            if (pob->chroma) {
                pob->nSrcPitch[1] = pob->pSrcFrame->planes[1]->nPitch;
                pob->nSrcPitch[2] = pob->pSrcFrame->planes[2]->nPitch;
                pob->BLITCHROMA(pob->pSrc_temp[1], pob->nSrcPitch_temp[1], pob->pSrc[1], pob->nSrcPitch[1]);
                pob->BLITCHROMA(pob->pSrc_temp[2], pob->nSrcPitch_temp[2], pob->pSrc[2], pob->nSrcPitch[2]);
                pob->pSrc[1] = pob->pSrc_temp[1];
                pob->pSrc[2] = pob->pSrc_temp[2];
                pob->nSrcPitch[1] = pob->nSrcPitch_temp[1];
                pob->nSrcPitch[2] = pob->nSrcPitch_temp[2];
            }

            if (pob->blky == 0)
                pob->nLambda = 0;
            else
                pob->nLambda = nLambdaLevel;

            pob->penaltyNew = pnew; // penalty for new vector
            // may be they must be scaled by nPel ?

            /* computes search boundaries */
            pob->nDxMax = pob->nPel * (pob->pSrcFrame->planes[0]->nPaddedWidth - pob->x[0] - pob->nBlkSizeX);
            pob->nDyMax = pob->nPel * (pob->pSrcFrame->planes[0]->nPaddedHeight - pob->y[0] - pob->nBlkSizeY);
            pob->nDxMin = -pob->nPel * pob->x[0];
            pob->nDyMin = -pob->nPel * pob->y[0];

            // get and interplolate old vectors
            int centerX = pob->nBlkSizeX / 2 + (pob->nBlkSizeX - pob->nOverlapX) * pob->blkx; // center of new block
            int blkxold = (centerX - nBlkSizeXold / 2) / nStepXold;       // centerXold less or equal to new
            int centerY = pob->nBlkSizeY / 2 + (pob->nBlkSizeY - pob->nOverlapY) * pob->blky;
            int blkyold = (centerY - nBlkSizeYold / 2) / nStepYold;

            int deltaX = VSMAX(0, centerX - (nBlkSizeXold / 2 + nStepXold * blkxold)); // distance from old to new
            int deltaY = VSMAX(0, centerY - (nBlkSizeYold / 2 + nStepYold * blkyold));

            int blkxold1 = VSMIN(nBlkXold - 1, VSMAX(0, blkxold));
            int blkxold2 = VSMIN(nBlkXold - 1, VSMAX(0, blkxold + 1));
            int blkyold1 = VSMIN(nBlkYold - 1, VSMAX(0, blkyold));
            int blkyold2 = VSMIN(nBlkYold - 1, VSMAX(0, blkyold + 1));

            VECTOR vectorOld; // interpolated or nearest

            if (smooth == 1) { // interpolate
                VECTOR vectorOld1 = fgopGetBlock(fgop, 0, blkxold1 + blkyold1 * nBlkXold)->vector; // 4 old nearest vectors (may coinside)
                VECTOR vectorOld2 = fgopGetBlock(fgop, 0, blkxold2 + blkyold1 * nBlkXold)->vector;
                VECTOR vectorOld3 = fgopGetBlock(fgop, 0, blkxold1 + blkyold2 * nBlkXold)->vector;
                VECTOR vectorOld4 = fgopGetBlock(fgop, 0, blkxold2 + blkyold2 * nBlkXold)->vector;

                // interpolate
                int vector1_x = vectorOld1.x * nStepXold + deltaX * (vectorOld2.x - vectorOld1.x); // scaled by nStepXold to skip slow division
                int vector1_y = vectorOld1.y * nStepXold + deltaX * (vectorOld2.y - vectorOld1.y);
                int64_t vector1_sad = vectorOld1.sad * nStepXold + deltaX * (vectorOld2.sad - vectorOld1.sad);

                int vector2_x = vectorOld3.x * nStepXold + deltaX * (vectorOld4.x - vectorOld3.x);
                int vector2_y = vectorOld3.y * nStepXold + deltaX * (vectorOld4.y - vectorOld3.y);
                int64_t vector2_sad = vectorOld3.sad * nStepXold + deltaX * (vectorOld4.sad - vectorOld3.sad);

                vectorOld.x = (vector1_x + deltaY * (vector2_x - vector1_x) / nStepYold) / nStepXold;
                vectorOld.y = (vector1_y + deltaY * (vector2_y - vector1_y) / nStepYold) / nStepXold;
                vectorOld.sad = (vector1_sad + deltaY * (vector2_sad - vector1_sad) / nStepYold) / nStepXold;

            } else { // nearest
                if (deltaX * 2 < nStepXold && deltaY * 2 < nStepYold)
                    vectorOld = fgopGetBlock(fgop, 0, blkxold1 + blkyold1 * nBlkXold)->vector;
                else if (deltaX * 2 >= nStepXold && deltaY * 2 < nStepYold)
                    vectorOld = fgopGetBlock(fgop, 0, blkxold2 + blkyold1 * nBlkXold)->vector;
                else if (deltaX * 2 < nStepXold && deltaY * 2 >= nStepYold)
                    vectorOld = fgopGetBlock(fgop, 0, blkxold1 + blkyold2 * nBlkXold)->vector;
                else //(deltaX*2>=nStepXold && deltaY*2>=nStepYold )
                    vectorOld = fgopGetBlock(fgop, 0, blkxold2 + blkyold2 * nBlkXold)->vector;
            }

            // scale vector to new nPel
            vectorOld.x = (vectorOld.x * pob->nPel) >> nLogPelold;
            vectorOld.y = (vectorOld.y * pob->nPel) >> nLogPelold;

            pob->predictor = pobClipMV(pob, vectorOld);                                                                    // predictor
            pob->predictor.sad = vectorOld.sad * (pob->nBlkSizeX * pob->nBlkSizeY) / (nBlkSizeXold * nBlkSizeYold); // normalized to new block size

            pob->bestMV = pob->predictor;

            // update SAD
            if (pob->dctmode != 0) { // DCT method (luma only - currently use normal spatial SAD chroma)
                // make dct of source block
                if (pob->dctmode <= 4) //don't do the slow dct conversion if SATD used
                    dctBytes2D(pob->DCT, pob->pSrc[0], pob->nSrcPitch[0], pob->dctSrc, pob->dctpitch);
            }
            if (pob->dctmode >= 3) // most use it and it should be fast anyway //if (dctmode == 3 || dctmode == 4) // check it
                pob->srcLuma = pob->LUMA(pob->pSrc[0], pob->nSrcPitch[0]);

            int64_t sad = pobLumaSAD(pob, pobGetRefBlock(pob, pob->predictor.x, pob->predictor.y));
            if (pob->chroma) {
                sad += pob->SADCHROMA(pob->pSrc[1], pob->nSrcPitch[1], pobGetRefBlockU(pob, pob->predictor.x, pob->predictor.y), pob->nRefPitch[1]);
                sad += pob->SADCHROMA(pob->pSrc[2], pob->nSrcPitch[2], pobGetRefBlockV(pob, pob->predictor.x, pob->predictor.y), pob->nRefPitch[2]);
            }
            pob->bestMV.sad = sad;
            pob->nMinCost = sad;

            if (pob->bestMV.sad > thSAD) { // if old interpolated vector is bad
                // then, we refine, according to the search type
                if (pob->searchType == SearchOnetime)
                    for (int i = pob->nSearchParam; i > 0; i /= 2)
                        pobOneTimeSearch(pob, i);

                if (pob->searchType == SearchNstep)
                    pobNStepSearch(pob, pob->nSearchParam);

                if (pob->searchType == SearchLogarithmic)
                    for (int i = pob->nSearchParam; i > 0; i /= 2)
                        pobDiamondSearch(pob, i);

                if (pob->searchType == SearchExhaustive) {
                    int mvx = pob->bestMV.x;
                    int mvy = pob->bestMV.y;
                    for (int i = 1; i <= pob->nSearchParam; i++) // region is same as exhaustive, but ordered by radius (from near to far)
                        pobExpandingSearch(pob, i, 1, mvx, mvy);
                }

                if (pob->searchType == SearchHex2)
                    pobHex2Search(pob, pob->nSearchParam);

                if (pob->searchType == SearchUnevenMultiHexagon)
                    pobUMHSearch(pob, pob->nSearchParam, pob->bestMV.x, pob->bestMV.y);

                if (pob->searchType == SearchHorizontal) {
                    int mvx = pob->bestMV.x;
                    int mvy = pob->bestMV.y;
                    for (int i = 1; i <= pob->nSearchParam; i++) {
                        pobCheckMV(pob, mvx - i, mvy);
                        pobCheckMV(pob, mvx + i, mvy);
                    }
                }

                if (pob->searchType == SearchVertical) {
                    int mvx = pob->bestMV.x;
                    int mvy = pob->bestMV.y;
                    for (int i = 1; i <= pob->nSearchParam; i++) {
                        pobCheckMV(pob, mvx, mvy - i);
                        pobCheckMV(pob, mvx, mvy + i);
                    }
                }
            }

            // we store the result
            pob->vectors[pob->blkIdx] = pob->bestMV;


            /* write the results */
            pBlkData[pob->blkx] = pob->bestMV;


            if (pob->smallestPlane)
                pob->sumLumaChange += pob->LUMA(pobGetRefBlock(pob, 0, 0), pob->nRefPitch[0]) - pob->LUMA(pob->pSrc[0], pob->nSrcPitch[0]);

            if (iblkx < pob->nBlkX - 1) {
                pob->x[0] += (pob->nBlkSizeX - pob->nOverlapX) * pob->blkScanDir;
                if (pob->pSrcFrame->nMode & UPLANE)
                    pob->x[1] += ((pob->nBlkSizeX - pob->nOverlapX) >> pob->nLogxRatioUV) * pob->blkScanDir;
                if (pob->pSrcFrame->nMode & VPLANE)
                    pob->x[2] += ((pob->nBlkSizeX - pob->nOverlapX) >> pob->nLogxRatioUV) * pob->blkScanDir;
            }
        }
        pBlkData += pob->nBlkX;

        pob->y[0] += (pob->nBlkSizeY - pob->nOverlapY);
        if (pob->pSrcFrame->nMode & UPLANE)
            pob->y[1] += ((pob->nBlkSizeY - pob->nOverlapY) >> pob->nLogyRatioUV);
        if (pob->pSrcFrame->nMode & VPLANE)
            pob->y[2] += ((pob->nBlkSizeY - pob->nOverlapY) >> pob->nLogyRatioUV);
    }
}

void pobInterpolatePrediction(PlaneOfBlocks *pob, const PlaneOfBlocks *pob2) {
    int normFactor = 3 - pob->nLogPel + pob2->nLogPel;
    int mulFactor = (normFactor < 0) ? -normFactor : 0;
    normFactor = (normFactor < 0) ? 0 : normFactor;
    int normov = (pob->nBlkSizeX - pob->nOverlapX) * (pob->nBlkSizeY - pob->nOverlapY);
    int aoddx = (pob->nBlkSizeX * 3 - pob->nOverlapX * 2);
    int aevenx = (pob->nBlkSizeX * 3 - pob->nOverlapX * 4);
    int aoddy = (pob->nBlkSizeY * 3 - pob->nOverlapY * 2);
    int aeveny = (pob->nBlkSizeY * 3 - pob->nOverlapY * 4);
    // note: overlapping is still (v2.5.7) not processed properly
    for (int l = 0, index = 0; l < pob->nBlkY; l++) {
        for (int k = 0; k < pob->nBlkX; k++, index++) {
            VECTOR v1, v2, v3, v4;
            int i = k;
            int j = l;
            if (i >= 2 * pob2->nBlkX)
                i = 2 * pob2->nBlkX - 1;
            if (j >= 2 * pob2->nBlkY)
                j = 2 * pob2->nBlkY - 1;
            int offy = -1 + 2 * (j % 2);
            int offx = -1 + 2 * (i % 2);

            if ((i == 0) || (i >= 2 * pob2->nBlkX - 1)) {
                if ((j == 0) || (j >= 2 * pob2->nBlkY - 1)) {
                    v1 = v2 = v3 = v4 = pob2->vectors[i / 2 + (j / 2) * pob2->nBlkX];
                } else {
                    v1 = v2 = pob2->vectors[i / 2 + (j / 2) * pob2->nBlkX];
                    v3 = v4 = pob2->vectors[i / 2 + (j / 2 + offy) * pob2->nBlkX];
                }
            } else if ((j == 0) || (j >= 2 * pob2->nBlkY - 1)) {
                v1 = v2 = pob2->vectors[i / 2 + (j / 2) * pob2->nBlkX];
                v3 = v4 = pob2->vectors[i / 2 + offx + (j / 2) * pob2->nBlkX];
            } else {
                v1 = pob2->vectors[i / 2 + (j / 2) * pob2->nBlkX];
                v2 = pob2->vectors[i / 2 + offx + (j / 2) * pob2->nBlkX];
                v3 = pob2->vectors[i / 2 + (j / 2 + offy) * pob2->nBlkX];
                v4 = pob2->vectors[i / 2 + offx + (j / 2 + offy) * pob2->nBlkX];
            }

            int64_t temp_sad;

            if (pob->nOverlapX == 0 && pob->nOverlapY == 0) {
                pob->vectors[index].x = 9 * v1.x + 3 * v2.x + 3 * v3.x + v4.x;
                pob->vectors[index].y = 9 * v1.y + 3 * v2.y + 3 * v3.y + v4.y;
                temp_sad = 9 * v1.sad + 3 * v2.sad + 3 * v3.sad + v4.sad + 8;
            } else if (pob->nOverlapX <= (pob->nBlkSizeX >> 1) && pob->nOverlapY <= (pob->nBlkSizeY >> 1)) { // corrected in v1.4.11
                int ax1 = (offx > 0) ? aoddx : aevenx;
                int ax2 = (pob->nBlkSizeX - pob->nOverlapX) * 4 - ax1;
                int ay1 = (offy > 0) ? aoddy : aeveny;
                int ay2 = (pob->nBlkSizeY - pob->nOverlapY) * 4 - ay1;
                // 64 bit so that the multiplications by the SADs don't overflow with 16 bit input.
                int64_t a11 = ax1 * ay1, a12 = ax1 * ay2, a21 = ax2 * ay1, a22 = ax2 * ay2;
                pob->vectors[index].x = (int)((a11 * v1.x + a21 * v2.x + a12 * v3.x + a22 * v4.x) / normov);
                pob->vectors[index].y = (int)((a11 * v1.y + a21 * v2.y + a12 * v3.y + a22 * v4.y) / normov);
                temp_sad = (a11 * v1.sad + a21 * v2.sad + a12 * v3.sad + a22 * v4.sad) / normov;
            } else { // large overlap. Weights are not quite correct but let it be
                // Dead branch. The overlap is no longer allowed to be more than half the block size.
                pob->vectors[index].x = (v1.x + v2.x + v3.x + v4.x) << 2;
                pob->vectors[index].y = (v1.y + v2.y + v3.y + v4.y) << 2;
                temp_sad = (v1.sad + v2.sad + v3.sad + v4.sad + 2) << 2;
            }
            pob->vectors[index].x = (pob->vectors[index].x >> normFactor) * (1 << mulFactor);
            pob->vectors[index].y = (pob->vectors[index].y >> normFactor) * (1 << mulFactor);
            pob->vectors[index].sad = temp_sad >> 4;
        }
    }
}


MVArraySizeType pobGetArraySize(const PlaneOfBlocks *pob, int divideMode) {
    MVArraySizeType size = sizeof(size) + pob->nBlkCount * sizeof(VECTOR);

    if (pob->nLogScale == 0) {
        if (divideMode)
            size += sizeof(MVArraySizeType) + pob->nBlkCount * sizeof(VECTOR) * 4; // reserve space for divided subblocks extra level
    }

    return size;
}


MVArraySizeType pobWriteDefaultToArray(const PlaneOfBlocks *pob, uint8_t *array, int divideMode) {
    MVArraySizeType size = sizeof(size) + pob->nBlkCount * sizeof(VECTOR);

    memcpy(array, &size, sizeof(size));

    VECTOR def = { 0, 0, pob->verybigSAD };

    VECTOR *blocks = (VECTOR *)(array + sizeof(size));

    for (int i = 0; i < pob->nBlkCount; i++)
        blocks[i] = def;

    if (pob->nLogScale == 0) {
        if (divideMode) { // reserve space for divided subblocks extra level
            array += size;

            size = sizeof(size) + pob->nBlkCount * sizeof(VECTOR) * 4; // 4 subblocks

            memcpy(array, &size, sizeof(size));

            blocks = (VECTOR *)(array + sizeof(size));

            for (int i = 0; i < pob->nBlkCount * 4; i++)
                blocks[i] = def;
        }
    }
    return pobGetArraySize(pob, divideMode);
}


void pobEstimateGlobalMVDoubled(PlaneOfBlocks *pob, VECTOR *globalMVec) {
    // estimate global motion from current plane vectors data for using on next plane - added by Fizick
    // on input globalMVec is prev estimation
    // on output globalMVec is doubled for next scale plane using

    // use very simple but robust method
    // more advanced method (like MVDepan) can be implemented later

    // find most frequent x
    memset(&pob->freqArray[0], 0, pob->freqSize * sizeof(int)); // reset
    int indmin = pob->freqSize - 1;
    int indmax = 0;
    for (int i = 0; i < pob->nBlkCount; i++) {
        int ind = (pob->freqSize >> 1) + pob->vectors[i].x;
        if (ind >= 0 && ind < pob->freqSize) {
            pob->freqArray[ind] += 1;
            if (ind > indmax)
                indmax = ind;
            if (ind < indmin)
                indmin = ind;
        }
    }
    int count = pob->freqArray[indmin];
    int index = indmin;
    for (int i = indmin + 1; i <= indmax; i++) {
        if (pob->freqArray[i] > count) {
            count = pob->freqArray[i];
            index = i;
        }
    }
    int medianx = (index - (pob->freqSize >> 1)); // most frequent value

    // find most frequent y
    memset(&pob->freqArray[0], 0, pob->freqSize * sizeof(int)); // reset
    indmin = pob->freqSize - 1;
    indmax = 0;
    for (int i = 0; i < pob->nBlkCount; i++) {
        int ind = (pob->freqSize >> 1) + pob->vectors[i].y;
        if (ind >= 0 && ind < pob->freqSize) {
            pob->freqArray[ind] += 1;
            if (ind > indmax)
                indmax = ind;
            if (ind < indmin)
                indmin = ind;
        }
    }
    count = pob->freqArray[indmin];
    index = indmin;
    for (int i = indmin + 1; i <= indmax; i++) {
        if (pob->freqArray[i] > count) {
            count = pob->freqArray[i];
            index = i;
        }
    }
    int mediany = (index - (pob->freqSize >> 1)); // most frequent value


    // iteration to increase precision
    int meanvx = 0;
    int meanvy = 0;
    int num = 0;
    for (int i = 0; i < pob->nBlkCount; i++) {
        if (abs(pob->vectors[i].x - medianx) < 6 && abs(pob->vectors[i].y - mediany) < 6) {
            meanvx += pob->vectors[i].x;
            meanvy += pob->vectors[i].y;
            num += 1;
        }
    }

    // output vectors must be doubled for next (finer) scale level
    if (num > 0) {
        globalMVec->x = 2 * meanvx / num;
        globalMVec->y = 2 * meanvy / num;
    } else {
        globalMVec->x = 2 * medianx;
        globalMVec->y = 2 * mediany;
    }
}
