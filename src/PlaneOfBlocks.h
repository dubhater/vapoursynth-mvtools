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

#ifndef PLANEOFBLOCKS_H
#define PLANEOFBLOCKS_H

#include <stdlib.h>

#include "Fakery.h"
#include "MVFrame.h"
#include "CopyCode.h"
#include "SADFunctions.h"
#include "CommonFunctions.h"
#include "Luma.h"
#include "DCTFFTW.h"

#define MAX_PREDICTOR 20 // right now 5 should be enough (TSchniede)

//#define    ONLY_CHECK_NONDEFAULT_MV // make the check if it is no default reference (zero, global,...)


typedef struct PlaneOfBlocks {

    /* fields set at initialization */

    int nBlkX;        /* width in number of blocks */
    int nBlkY;        /* height in number of blocks */
    int nBlkSizeX;    /* size of a block */
    int nBlkSizeY;    /* size of a block */
    int nBlkCount;    /* number of blocks in the plane */
    int nPel;         /* pel refinement accuracy */
    int nLogPel;      /* logarithm of the pel refinement accuracy */
    int nScale;       /* scaling factor of the plane */
    int nLogScale;    /* logarithm of the scaling factor */
    int nOverlapX; // overlap size
    int nOverlapY; // overlap size
    int xRatioUV;
    int yRatioUV;
    int nLogxRatioUV; // log of xRatioUV (0 for 1 and 1 for 2)
    int nLogyRatioUV; // log of yRatioUV (0 for 1 and 1 for 2)
    int bytesPerSample;

    SADFunction SAD;   /* function which computes the sad */
    LUMAFunction LUMA; /* function which computes the mean luma */
    COPYFunction BLITLUMA;
    COPYFunction BLITCHROMA;
    SADFunction SADCHROMA;
    SADFunction SATD; /* SATD function, (similar to SAD), used as replacement to dct */

    VECTOR *vectors; /* motion vectors of the blocks */
    /* before the search, contains the hierachal predictor */
    /* after the search, contains the best motion vector */

    int smallestPlane; /* say whether vectors can used predictors from a smaller plane */
    int chroma;        /* do we do chroma me */

    /* working fields */

    MVFrame *pSrcFrame;
    MVFrame *pRefFrame;

    int nSrcPitch[3];
    const uint8_t *pSrc[3]; // the alignment of this array is important for speed for some reason (cacheline?)
    int nRefPitch[3];

    VECTOR bestMV;    /* best vector found so far during the search */
    int nBestSad;     /* sad linked to the best vector */
    int nMinCost;     /* minimum cost ( sad + mv cost ) found so far */
    VECTOR predictor; /* best predictor for the current vector */

    VECTOR predictors[MAX_PREDICTOR]; /* set of predictors for the current block */

    int nDxMin; /* minimum x coordinate for the vector */
    int nDyMin; /* minimum y coordinate for the vector */
    int nDxMax; /* maximum x corrdinate for the vector */
    int nDyMax; /* maximum y coordinate for the vector */

    int x[3];       /* absolute x coordinate of the origin of the block in the reference frame */
    int y[3];       /* absolute y coordinate of the origin of the block in the reference frame */
    int blkx;       /* x coordinate in blocks */
    int blky;       /* y coordinate in blocks */
    int blkIdx;     /* index of the block */
    int blkScanDir; // direction of scan (1 is left to rught, -1 is right to left)

    /* search parameters */

    SearchType searchType; /* search type used */
    int nSearchParam;      /* additionnal parameter for this search */
    int64_t nLambda;       /* vector cost factor */
    int64_t LSAD;          // SAD limit for lambda using - Fizick
    int penaltyNew;        // cost penalty factor for new candidates
    int penaltyZero;       // cost penalty factor for zero vector
    int pglobal;           // cost penalty factor for global predictor
    //   int nLambdaLen; //  penalty factor (lambda) for vector length
    int64_t badSAD;   // SAD threshold for more wide search
    int badrange;     // wide search radius
    int64_t planeSAD; // summary SAD of plane
    int badcount;     // number of bad blocks refined
    int temporal;    // use temporal predictor
    int tryMany;     // try refine around many predictors

    int iter;

    VECTOR globalMVPredictor;  // predictor of global motion vector
    VECTOR zeroMVfieldShifted; // zero motion vector for fieldbased video at finest level pel2

    DCTFFTW *DCT;
    uint8_t *dctSrc;
    uint8_t *dctRef;
    int dctpitch;
    int dctmode;
    int srcLuma;
    int refLuma;
    int sumLumaChange;
    int dctweight16;
    int *freqArray; // temporary array for global motion estimaton
    int freqSize;   // size of freqArray
    int verybigSAD;

    int nSrcPitch_temp[3];
    uint8_t *pSrc_temp[3]; //for easy WRITE access to temp block
} PlaneOfBlocks;


void pobInit(PlaneOfBlocks *pob, int _nBlkX, int _nBlkY, int _nBlkSizeX, int _nBlkSizeY, int _nPel, int _nLevel, int nMotionFlags, int nCPUFlags, int _nOverlapX, int _nOverlapY, int _xRatioUV, int _yRatioUV, int bitsPerSample);

void pobDeinit(PlaneOfBlocks *pob);

void pobEstimateGlobalMVDoubled(PlaneOfBlocks *pob, VECTOR *globalMVec);

int pobGetArraySize(PlaneOfBlocks *pob, int divideMode);

void pobInterpolatePrediction(PlaneOfBlocks *pob, const PlaneOfBlocks *pob2);

void pobRecalculateMVs(PlaneOfBlocks *pob, const FakeGroupOfPlanes *fgop, MVFrame *pSrcFrame, MVFrame *pRefFrame, SearchType st, int stp, int lambda, int pnew, int *out, int fieldShift, int thSAD, DCTFFTW *DCT, int dctmode, int smooth, int meander);

void pobSearchMVs(PlaneOfBlocks *pob, MVFrame *pSrcFrame, MVFrame *pRefFrame, SearchType st, int stp, int lambda, int lsad, int pnew, int plevel, int *out, VECTOR *globalMVec, int fieldShift, DCTFFTW *DCT, int dctmode, int *pmeanLumaChange, int pzero, int pglobal, int64_t badSAD, int badrange, int meander, int tryMany);

int pobWriteDefaultToArray(PlaneOfBlocks *pob, int *array, int divideMode);

#endif
