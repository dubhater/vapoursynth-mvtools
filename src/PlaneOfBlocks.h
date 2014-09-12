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

#ifndef __POBLOCKS__
#define __POBLOCKS__

#include "MVInterface.h"
//#include "Resizer.h"
#include "Interpolation.h"
#include "CopyCode.h"
#include "SADFunctions.h"
#include "commonfunctions.h"
#include "Variance.h"
#include "dct.h"

#define MAX_PREDICTOR 20 // right now 5 should be enough (TSchniede)

class PlaneOfBlocks {

/* fields set at initialization */

	int nBlkX;                  /* width in number of blocks */
	int nBlkY;                  /* height in number of blocks */
	int nBlkSizeX;               /* size of a block */
	int nBlkSizeY;               /* size of a block */
	int nBlkCount;              /* number of blocks in the plane */
	int nPel;                   /* pel refinement accuracy */
   int nLogPel;                /* logarithm of the pel refinement accuracy */
   int nScale;                 /* scaling factor of the plane */
   int nLogScale;              /* logarithm of the scaling factor */
   int nFlags;                 /* additionnal flags */
   int nOverlapX; // overlap size
   int nOverlapY; // overlap size
	int yRatioUV;
	int nLogyRatioUV;// log of yRatioUV (0 for 1 and 1 for 2)

	SADFunction *SAD;           /* function which computes the sad */
   LUMAFunction *LUMA;         /* function which computes the mean luma */
   VARFunction *VAR;           /* function which computes the variance */
   COPYFunction *BLITLUMA;
   COPYFunction *BLITCHROMA;
   SADFunction *SADCHROMA;
   SADFunction *SATD;		/* SATD function, (similar to SAD), used as replacement to dct */

   VECTOR *vectors;            /* motion vectors of the blocks */
                               /* before the search, contains the hierachal predictor */
                               /* after the search, contains the best motion vector */

   bool smallestPlane;         /* say whether vectors can used predictors from a smaller plane */
//   bool mmx;                   /* can we use mmx asm code */
   bool isse;                  /* can we use isse asm code */
   bool chroma;                /* do we do chroma me */

/* working fields */

   MVFrame *pSrcFrame;
   MVFrame *pRefFrame;

   int nSrcPitch[3];
   const uint8_t* pSrc[3]; // the alignment of this array is important for speed for some reason (cacheline?)
   int nRefPitch[3];

   VECTOR bestMV;              /* best vector found so far during the search */
   int nBestSad;               /* sad linked to the best vector */
   int nMinCost;               /* minimum cost ( sad + mv cost ) found so far */
   VECTOR predictor;           /* best predictor for the current vector */

   VECTOR predictors[MAX_PREDICTOR];   /* set of predictors for the current block */

   int nDxMin;                 /* minimum x coordinate for the vector */
   int nDyMin;                 /* minimum y coordinate for the vector */
   int nDxMax;                 /* maximum x corrdinate for the vector */
   int nDyMax;                 /* maximum y coordinate for the vector */

   int x[3];                   /* absolute x coordinate of the origin of the block in the reference frame */
   int y[3];                   /* absolute y coordinate of the origin of the block in the reference frame */
   int blkx;                   /* x coordinate in blocks */
   int blky;                   /* y coordinate in blocks */
   int blkIdx;                 /* index of the block */
   int blkScanDir; // direction of scan (1 is left to rught, -1 is right to left)

/* search parameters */

	SearchType searchType;      /* search type used */
   int nSearchParam;           /* additionnal parameter for this search */
   int nLambda;                /* vector cost factor */
   int LSAD; // SAD limit for lambda using - Fizick
   int penaltyNew; // cost penalty factor for new candidates
   int penaltyZero; // cost penalty factor for zero vector
   int pglobal; // cost penalty factor for global predictor
//   int nLambdaLen; //  penalty factor (lambda) for vector length
    int badSAD; // SAD threshold for more wide search
    int badrange; // wide search radius
    int planeSAD; // summary SAD of plane
    int badcount; // number of bad blocks refined
    bool temporal; // use temporal predictor
    bool tryMany; // try refine around many predictors

    int iter;

   VECTOR globalMVPredictor; // predictor of global motion vector
   VECTOR zeroMVfieldShifted; // zero motion vector for fieldbased video at finest level pel2

   DCTClass * DCT;
   uint8_t * dctSrc;
   uint8_t * dctRef;
   int dctpitch;
   int dctmode;
   int srcLuma;
   int refLuma;
   int sumLumaChange;
   int dctweight16;
   int *freqArray; // temporary array for global motion estimaton
   int freqSize;// size of freqArray
   int verybigSAD;

#ifdef ALIGN_SOURCEBLOCK
	int nSrcPitch_plane[3];	//stores the pitch of the whole plane for easy access (nSrcPitch in non-aligned mode)
	uint8_t* pSrc_temp[3];	//for easy WRITE access to temp block
	uint8_t* pSrc_temp_base;	//stores base memory pointer to non _base pointer
	uint8_t * dctSrc_base;		//stores base memory pointer to non _base pointer
	uint8_t * dctRef_base;		//stores base memory pointer to non _base pointer
#endif

   /* inline functions */

   /* fetch the block in the reference frame, which is pointed by the vector (vx, vy) */
   inline const uint8_t *GetRefBlock(int nVx, int nVy)
   {
//      return pRefFrame->GetPlane(YPLANE)->GetAbsolutePointer((x[0]<<nLogPel) + nVx, (y[0]<<nLogPel) + nVy);
      return (nPel==2) ? pRefFrame->GetPlane(YPLANE)->GetAbsolutePointerPel2((x[0]<<1) + nVx, (y[0]<<1) + nVy) :
             (nPel==1) ? pRefFrame->GetPlane(YPLANE)->GetAbsolutePointerPel1((x[0]) + nVx, (y[0]) + nVy) :
                  pRefFrame->GetPlane(YPLANE)->GetAbsolutePointerPel4((x[0]<<2) + nVx, (y[0]<<2) + nVy);
   }

   inline const uint8_t *GetRefBlockU(int nVx, int nVy)
   {
//      return pRefFrame->GetPlane(UPLANE)->GetAbsolutePointer((x[1]<<nLogPel) + (nVx >> 1), (y[1]<<nLogPel) + (yRatioUV==1 ? nVy : nVy>>1) ); //v.1.2.1
      return (nPel==2) ? pRefFrame->GetPlane(UPLANE)->GetAbsolutePointerPel2((x[1]<<1) + (nVx >> 1), (y[1]<<1) + (yRatioUV==1 ? nVy : nVy>>1) ) :
      (nPel==1) ? pRefFrame->GetPlane(UPLANE)->GetAbsolutePointerPel1((x[1]) + (nVx >> 1), (y[1]) + (yRatioUV==1 ? nVy : nVy>>1) ) :
                 pRefFrame->GetPlane(UPLANE)->GetAbsolutePointerPel4((x[1]<<2) + (nVx >> 1), (y[1]<<2) + (yRatioUV==1 ? nVy : nVy>>1) );
   }

   inline const uint8_t *GetRefBlockV(int nVx, int nVy)
   {
// return pRefFrame->GetPlane(VPLANE)->GetAbsolutePointer((x[2]<<nLogPel) + (nVx >> 1), (y[2]<<nLogPel) + (yRatioUV==1 ? nVy : nVy>>1) );
	   return (nPel==2) ? pRefFrame->GetPlane(VPLANE)->GetAbsolutePointerPel2((x[2]<<1) + (nVx >> 1), (y[2]<<1) + (yRatioUV==1 ? nVy : nVy>>1) ) :
	   (nPel==1) ? pRefFrame->GetPlane(VPLANE)->GetAbsolutePointerPel1((x[2]) + (nVx >> 1), (y[2]) + (yRatioUV==1 ? nVy : nVy>>1) ) :
                   pRefFrame->GetPlane(VPLANE)->GetAbsolutePointerPel4((x[2]<<2) + (nVx >> 1), (y[2]<<2) + (yRatioUV==1 ? nVy : nVy>>1) );
   }

   inline const uint8_t *GetSrcBlock(int nX, int nY)
   {
      return pSrcFrame->GetPlane(YPLANE)->GetAbsolutePelPointer(nX, nY);
   }

   /* computes the cost of a vector (vx, vy) */
	inline int MotionDistorsion(int vx, int vy)
	{
		int dist = SquareDifferenceNorm(predictor, vx, vy);
		return (nLambda * dist) >> 8;
	}

   /* computes the length cost of a vector (vx, vy) */
//	inline int LengthPenalty(int vx, int vy)
//	{
//		return ( (vx*vx + vy*vy)*nLambdaLen) >> 8;
//	}


	int LumaSADx (const unsigned char *pRef0)
	{
		int sad;
		switch (dctmode)
		{
		case 1: // dct SAD
			DCT->DCTBytes2D(pRef0, nRefPitch[0], dctRef, dctpitch);
			sad = (SAD(dctSrc, dctpitch, dctRef, dctpitch) + abs(dctSrc[0]-dctRef[0])*3)*nBlkSizeX/2; //correct reduced DC component
			break;
		case 2: //  globally (lumaChange) weighted spatial and DCT
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (dctweight16 > 0)
			{
				DCT->DCTBytes2D(pRef0, nRefPitch[0], dctRef, dctpitch);
				int dctsad = (SAD(dctSrc, dctpitch, dctRef, dctpitch)+ abs(dctSrc[0]-dctRef[0])*3)*nBlkSizeX/2;
				sad = (sad*(16-dctweight16) + dctsad*dctweight16)/16;
			}
			break;
		case 3: // per block adaptive switched from spatial to equal mixed SAD (faster)
			refLuma = LUMA(pRef0, nRefPitch[0]);
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (abs(srcLuma - refLuma) > (srcLuma + refLuma)>>5)
			{
				DCT->DCTBytes2D(pRef0, nRefPitch[0], dctRef, dctpitch);
				int dctsad = SAD(dctSrc, dctpitch, dctRef, dctpitch)*nBlkSizeX/2;
				sad = sad/2 + dctsad/2;
			}
			break;
		case 4: //  per block adaptive switched from spatial to mixed SAD with more weight of DCT (best?)
			refLuma = LUMA(pRef0, nRefPitch[0]);
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (abs(srcLuma - refLuma) > (srcLuma + refLuma)>>5)
			{
				DCT->DCTBytes2D(pRef0, nRefPitch[0], dctRef, dctpitch);
				int dctsad = SAD(dctSrc, dctpitch, dctRef, dctpitch)*nBlkSizeX/2;
				sad = sad/4 + dctsad/2 + dctsad/4;
			}
			break;
				case 5: // dct SAD (SATD)
			sad = SATD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			break;
		case 6: //  globally (lumaChange) weighted spatial and DCT (better estimate)
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (dctweight16 > 0)
			{
				int dctsad = SATD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
				sad = (sad*(16-dctweight16) + dctsad*dctweight16)/16;
			}
			break;
		case 7: // per block adaptive switched from spatial to equal mixed SAD (faster?)
			refLuma = LUMA(pRef0, nRefPitch[0]);
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (abs(srcLuma - refLuma) > (srcLuma + refLuma)>>5)
			{
				int dctsad = SATD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
				sad = sad/2 + dctsad/2;
			}
			break;
		case 8: //  per block adaptive switched from spatial to mixed SAD with more weight of DCT (faster?)
			refLuma = LUMA(pRef0, nRefPitch[0]);
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (abs(srcLuma - refLuma) > (srcLuma + refLuma)>>5)
			{
				int dctsad = SATD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
				sad = sad/4 + dctsad/2 + dctsad/4;
			}
			break;
		case 9: //  globally (lumaChange) weighted spatial and DCT (better estimate, only half weight on SATD)
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (dctweight16 > 1)
			{
				int dctweighthalf=dctweight16/2;
				int dctsad = SATD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
				sad = (sad*(16-dctweighthalf) + dctsad*dctweighthalf)/16;
			}
			break;
		case 10: // per block adaptive switched from spatial to mixed SAD, weighted to SAD (faster)
			refLuma = LUMA(pRef0, nRefPitch[0]);
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
			if (abs(srcLuma - refLuma) > (srcLuma + refLuma)>>4)
			{
				int dctsad = SATD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
				sad = sad/2 + dctsad/4 + sad/4;
			}
			break;
		default:
			sad = SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
		}
		return sad;
	}

	inline int LumaSAD (const unsigned char *pRef0)
	{
#ifdef MOTION_DEBUG
		iter++;
#endif
#ifdef ALLOW_DCT
		// made simple SAD more prominent (~1% faster) while keeping DCT support (TSchniede)
		return !dctmode ? SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]) : LumaSADx(pRef0);
#else
		return SAD(pSrc[0], nSrcPitch[0], pRef0, nRefPitch[0]);
#endif
	}


    /* check if the vector (vx, vy) is better than the best vector found so far without penalty new - renamed in v.2.11*/
	inline void CheckMV0(int vx, int vy)
	{		//here the chance for default values are high especially for zeroMVfieldShifted (on left/top border)
		if (
#ifdef ONLY_CHECK_NONDEFAULT_MV
		(( vx != 0 ) || ( vy != zeroMVfieldShifted.y )) &&
			 (( vx != predictor.x ) || ( vy != predictor.y )) &&
			 (( vx != globalMVPredictor.x ) || ( vy != globalMVPredictor.y )) &&
#endif
			 IsVectorOK(vx, vy) )
		{
			int saduv = (chroma) ? SADCHROMA(pSrc[1], nSrcPitch[1], GetRefBlockU(vx, vy), nRefPitch[1])
            + SADCHROMA(pSrc[2], nSrcPitch[2], GetRefBlockV(vx, vy), nRefPitch[2]) : 0;
			int sad = LumaSAD(GetRefBlock(vx, vy));
            sad += saduv;
			int cost = sad + MotionDistorsion(vx, vy);
//			int cost = sad + sad*MotionDistorsion(vx, vy)/(nBlkSizeX*nBlkSizeY*4);
//        if (sad>bigSAD) { DebugPrintf("%d %d %d %d %d %d", blkIdx, vx, vy, nMinCost, cost, sad);}
			if ( cost  < nMinCost )
			{
				bestMV.x = vx;
				bestMV.y = vy;
				nMinCost = cost;
				bestMV.sad = sad;
			}
		}
	}

    /* check if the vector (vx, vy) is better than the best vector found so far */
	inline void CheckMV(int vx, int vy)
	{		//here the chance for default values are high especially for zeroMVfieldShifted (on left/top border)
		if (
#ifdef ONLY_CHECK_NONDEFAULT_MV
		(( vx != 0 ) || ( vy != zeroMVfieldShifted.y )) &&
			 (( vx != predictor.x ) || ( vy != predictor.y )) &&
			 (( vx != globalMVPredictor.x ) || ( vy != globalMVPredictor.y )) &&
#endif
			 IsVectorOK(vx, vy) )
		{
			int saduv =
			!(chroma) ? 0:
			SADCHROMA(pSrc[1], nSrcPitch[1], GetRefBlockU(vx, vy), nRefPitch[1])
            + SADCHROMA(pSrc[2], nSrcPitch[2], GetRefBlockV(vx, vy), nRefPitch[2]);
			int sad = LumaSAD(GetRefBlock(vx, vy));
            sad += saduv;
			int cost = sad + MotionDistorsion(vx, vy)+ ((penaltyNew*sad)>>8); //v2
//			int cost = sad + sad*MotionDistorsion(vx, vy)/(nBlkSizeX*nBlkSizeY*4);
//        if (sad>bigSAD) { DebugPrintf("%d %d %d %d %d %d", blkIdx, vx, vy, nMinCost, cost, sad);}
			if ( cost  < nMinCost )
			{
				bestMV.x = vx;
				bestMV.y = vy;
				nMinCost = cost;
				bestMV.sad = sad;
			}
		}
	}

    /* check if the vector (vx, vy) is better, and update dir accordingly */
	inline void CheckMV2(int vx, int vy, int *dir, int val)
	{
		if (
#ifdef ONLY_CHECK_NONDEFAULT_MV
			(( vx != 0 ) || ( vy != zeroMVfieldShifted.y )) &&
			(( vx != predictor.x ) || ( vy != predictor.y )) &&
			(( vx != globalMVPredictor.x ) || ( vy != globalMVPredictor.y )) &&
#endif
			IsVectorOK(vx, vy) )
		{
			int saduv =
			!(chroma) ? 0:
			SADCHROMA(pSrc[1], nSrcPitch[1], GetRefBlockU(vx, vy), nRefPitch[1])
            + SADCHROMA(pSrc[2], nSrcPitch[2], GetRefBlockV(vx, vy), nRefPitch[2]);
			int sad = LumaSAD(GetRefBlock(vx, vy));
			sad += saduv;
			int cost = sad + MotionDistorsion(vx, vy) + ((penaltyNew*sad)>>8); // v1.5.8
//			if (sad > LSAD/4) DebugPrintf("%d %d %d %d %d %d %d", blkIdx, vx, vy, val, nMinCost, cost, sad);
//			int cost = sad + sad*MotionDistorsion(vx, vy)/(nBlkSizeX*nBlkSizeY*4) + ((penaltyNew*sad)>>8); // v1.5.8
			if ( cost  < nMinCost )
			{
				bestMV.x = vx;
				bestMV.y = vy;
				bestMV.sad = sad;
				nMinCost = cost;
				*dir = val;
			}
		}
	}

    /* check if the vector (vx, vy) is better, and update dir accordingly, but not bestMV.x, y */
	inline void CheckMVdir(int vx, int vy, int *dir, int val)
	{
		if (
#ifdef ONLY_CHECK_NONDEFAULT_MV
			(( vx != 0 ) || ( vy != zeroMVfieldShifted.y )) &&
			(( vx != predictor.x ) || ( vy != predictor.y )) &&
			(( vx != globalMVPredictor.x ) || ( vy != globalMVPredictor.y )) &&
#endif
			IsVectorOK(vx, vy) )
		{
			int saduv = (chroma) ? SADCHROMA(pSrc[1], nSrcPitch[1], GetRefBlockU(vx, vy), nRefPitch[1])
            + SADCHROMA(pSrc[2], nSrcPitch[2], GetRefBlockV(vx, vy), nRefPitch[2]) : 0;
			int sad = LumaSAD(GetRefBlock(vx, vy));
			sad += saduv;
			int cost = sad + MotionDistorsion(vx, vy) + ((penaltyNew*sad)>>8); // v1.5.8
//			if (sad > LSAD/4) DebugPrintf("%d %d %d %d %d %d %d", blkIdx, vx, vy, val, nMinCost, cost, sad);
//			int cost = sad + sad*MotionDistorsion(vx, vy)/(nBlkSizeX*nBlkSizeY*4) + ((penaltyNew*sad)>>8); // v1.5.8
			if ( cost  < nMinCost )
			{
				bestMV.sad = sad;
				nMinCost = cost;
				*dir = val;
			}
		}
	}

    /* clip a vector to the horizontal boundaries */
	inline int ClipMVx(int vx)
	{
//		return imin(nDxMax - 1, imax(nDxMin, vx));
		if ( vx < nDxMin ) return nDxMin;
		else if ( vx >= nDxMax ) return nDxMax - 1;
		else return vx;
	}

    /* clip a vector to the vertical boundaries */
	inline int ClipMVy(int vy)
	{
//		return imin(nDyMax - 1, imax(nDyMin, vy));
		if ( vy < nDyMin ) return nDyMin;
		else if ( vy >= nDyMax ) return nDyMax - 1;
		else return vy;
	}

    /* clip a vector to the search boundaries */
	inline VECTOR ClipMV(VECTOR v)
	{
		VECTOR v2;
		v2.x = ClipMVx(v.x);
		v2.y = ClipMVy(v.y);
        v2.sad = v.sad;
		return v2;
	}


    /* find the median between a, b and c */
	inline static int Median(int a, int b, int c)
	{
//		return a + b + c - imax(a, imax(b, c)) - imin(c, imin(a, b));
		if ( a < b )
		{
			if ( b < c ) return b;
			else if ( a < c ) return c;
			else return a;
		}
		else {
			if ( a < c ) return a;
			else if ( b < c ) return c;
			else return b;
		}
	}

    /* check if a vector is inside search boundaries */
	inline  bool IsVectorOK(int vx, int vy)
	{
		return (( vx >= nDxMin ) &&
			( vy >= nDyMin ) &&
			( vx < nDxMax ) &&
			( vy < nDyMax ));
	}

    /* computes square distance between two vectors */
	inline static unsigned int SquareDifferenceNorm(const VECTOR& v1, const VECTOR& v2)
	{ return (v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y); }

    /* computes square distance between two vectors */
	inline static unsigned int SquareDifferenceNorm(const VECTOR& v1, const int v2x, const int v2y)
	{ return (v1.x - v2x) * (v1.x - v2x) + (v1.y - v2y) * (v1.y - v2y); }

    /* check if an index is inside the block's min and max indexes */
	inline bool IsInFrame(int i)
	{
		return (( i >= 0 ) && ( i < nBlkCount ));
	}

    void Refine();

public :

	PlaneOfBlocks(int _nBlkX, int _nBlkY, int _nBlkSizeX, int _nBlkSizeY, int _nPel, int _nLevel, int _nFlags, int _nOverlapX, int _nOverlapY, int _yRatioUV);

	~PlaneOfBlocks();

/* mv search related functions */

    /* fill the predictors array */
	void FetchPredictors();

    /* performs a diamond search */
	void DiamondSearch(int step);

    /* performs a square search */
	void SquareSearch();

    /* performs an exhaustive search */
	void ExhaustiveSearch(int radius); // diameter = 2*radius - 1

    /* performs an n-step search */
	void NStepSearch(int stp);

    /* performs a one time search */
	void OneTimeSearch(int length);

    /* performs an epz search */
	void PseudoEPZSearch();

//	void PhaseShiftSearch(int vx, int vy);

    /* performs an exhaustive search */
	void ExpandingSearch(int radius, int step, int mvx, int mvy); // diameter = 2*radius + 1

    void Hex2Search(int i_me_range);
    void CrossSearch(int start, int x_max, int y_max, int mvx, int mvy);
    void UMHSearch(int i_me_range, int omx, int omy);

    /* search the vectors for the whole plane */
	void SearchMVs(MVFrame *_pSrcFrame, MVFrame *_pRefFrame, SearchType st,
                  int stp, int _lambda, int _lSAD, int _pennew, int _plevel,
				  int flags, int *out, VECTOR *globalMVec, short * outfilebuf, int _fieldShiftCur,
				  DCTClass * _DCT, int * _meanLumaChange, int _divideExtra,
				  int _pzero, int _pglobal, int badSAD, int badrange, bool meander, int *vecPrev, bool tryMany);


/* plane initialisation */

    /* compute the predictors from the upper plane */
	void InterpolatePrediction(const PlaneOfBlocks &pob);

 	void WriteHeaderToArray(int *array);
	int WriteDefaultToArray(int *array, int divideExtra);
	int GetArraySize(int divideExtra);
   void FitReferenceIntoArray(MVFrame *_pRefFrame, int *array);
   void EstimateGlobalMVDoubled(VECTOR *globalMVDoubled); // Fizick
	inline int GetnBlkX() { return nBlkX; }
	inline int GetnBlkY() { return nBlkY; }

    /* // only used by MVRecalculate
	void RecalculateMVs(MVClip & mvClip, MVFrame *_pSrcFrame, MVFrame *_pRefFrame, SearchType st,
                  int stp, int _lambda, int _lSAD, int _pennew,
				  int flags, int *out, short * outfilebuf, int _fieldShiftCur, int thSAD, DCTClass * _DCT,
				  int _divideExtra, int smooth, bool meander);
                  */
};

#endif
