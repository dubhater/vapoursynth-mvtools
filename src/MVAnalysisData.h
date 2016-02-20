// Define the BlockData class

// I borrowed a lot of code from XviD's sources here, so I thank all the developpers
// of this wonderful codec

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

#ifndef MVANALYSISDATA_H
#define MVANALYSISDATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

#include <VapourSynth.h>


#define MOTION_MAGIC_KEY 0x564D //'MV' is IMHO better 31415926 :)

typedef struct VECTOR {
    int x;
    int y;
    int sad;
} VECTOR;


#define N_PER_BLOCK 3


/*! \brief Search type : defines the algorithm used for minimizing the SAD */
typedef enum SearchType {
    SearchOnetime               = (1 << 0),
    SearchNstep                 = (1 << 1),
    SearchLogarithmic           = (1 << 2),
    SearchExhaustive            = (1 << 3),
    SearchHex2                  = (1 << 4),
    SearchUnevenMultiHexagon    = (1 << 5),
    SearchHorizontal            = (1 << 6),
    SearchVertical              = (1 << 7)
} SearchType;


#define MOTION_USE_ISSE             0x00000001
#define MOTION_IS_BACKWARD          0x00000002
#define MOTION_SMALLEST_PLANE       0x00000004
#define MOTION_USE_CHROMA_MOTION    0x00000008
//force MVAnalyse to use a different function for SAD / SADCHROMA (debug)
#define MOTION_USE_SSD              0x00000010
#define MOTION_USE_SATD             0x00000020


#define MV_DEFAULT_SCD1 400 // increased in v1.4.1
#define MV_DEFAULT_SCD2 130

//#define MV_BUFFER_FRAMES 10

static const VECTOR zeroMV = { 0, 0, -1 };


#define MVANALYSIS_DATA_VERSION 5

typedef struct MVAnalysisData {
    /*! \brief Unique identifier, not very useful */
    int nMagicKey; // placed to head in v.1.2.6

    int nVersion; // MVAnalysisData and outfile format version - added in v1.2.6

    /*! \brief size of a block, in pixel */
    int nBlkSizeX; // horizontal block size

    int nBlkSizeY; // vertical block size - v1.7

    /*! \brief pixel refinement of the motion estimation */
    int nPel;

    /*! \brief number of level for the hierarchal search */
    int nLvCount;

    /*! \brief difference between the index of the reference and the index of the current frame */
    int nDeltaFrame;

    /*! \brief direction of the search ( forward / backward ) */
    int isBackward;

    int nCPUFlags;

    /*! \brief diverse flags to set up the search */
    int nMotionFlags;

    /*! \brief Width of the frame */
    int nWidth;

    /*! \brief Height of the frame */
    int nHeight;

    int nOverlapX; // overlap block size - v1.1

    int nOverlapY; // vertical overlap - v1.7

    int nBlkX; // number of blocks along X

    int nBlkY; // number of blocks along Y

    int bitsPerSample;

    int yRatioUV; // ratio of luma plane height to chroma plane height

    int xRatioUV; // ratio of luma plane width to chroma plane width

    int nHPadding; // Horizontal padding - v1.8.1

    int nVPadding; // Vertical padding - v1.8.1
} MVAnalysisData;


void scaleThSCD(int *thscd1, int *thscd2, const MVAnalysisData *ad, const char *filter_name, char *error, size_t error_size);

void adataFromVectorClip(struct MVAnalysisData *ad, VSNodeRef *clip, const char *filter_name, const char *vector_name, const VSAPI *vsapi, char *error, size_t error_size);

void adataCheckSimilarity(const MVAnalysisData *ad1, const MVAnalysisData *ad2, const char *filter_name1, const char *filter_name2, const char *vector_name, char *error, size_t error_size);


//#define MOTION_DELTA_FRAME_BUFFER 5


#ifdef __cplusplus
} // extern "C"
#endif

#endif // MVANALYSISDATA_H
