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

#ifndef __MV_INTERFACES_H__
#define __MV_INTERFACES_H__

#include <stdint.h>
#include <stdio.h>

#include <stdexcept>
#include <string>

#include <VapourSynth.h>


#define MOTION_MAGIC_KEY 0x564D //'MV' is IMHO better 31415926 :)

struct VECTOR {
    int x;
    int y;
    int sad;
};

inline void CopyVector(VECTOR *destVector, const VECTOR *srcVector) {
    destVector->x = srcVector->x;
    destVector->y = srcVector->y;
    destVector->sad = srcVector->sad;
}

#define N_PER_BLOCK 3


/*! \brief Search type : defines the algorithm used for minimizing the SAD */
enum SearchType {
    ONETIME = 1,
    NSTEP = 2,
    LOGARITHMIC = 4,
    EXHAUSTIVE = 8,
    HEX2SEARCH = 16, // v.2
    UMHSEARCH = 32,  // v.2
    HSEARCH = 64,    // v.2.5.11
    VSEARCH = 128    // v.2.5.11
};

#define MAX(a, b) (((a) < (b)) ? (b) : (a))
#define MIN(a, b) (((a) > (b)) ? (b) : (a))


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

class MVAnalysisData {
public:
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
    bool isBackward;

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


public:
    inline void SetCPUFlags(int _nCPUFlags) {
        nCPUFlags |= _nCPUFlags;
    }
    inline int GetCPUFlags() const {
        return nCPUFlags;
    }
    inline void SetMotionFlags(int _nMotionFlags) {
        nMotionFlags |= _nMotionFlags;
    }
    inline int GetMotionFlags() const {
        return nMotionFlags;
    }
    inline int GetBlkSizeX() const {
        return nBlkSizeX;
    }
    inline int GetPel() const {
        return nPel;
    }
    inline int GetLevelCount() const {
        return nLvCount;
    }
    inline bool IsBackward() const {
        return isBackward;
    }
    inline int GetMagicKey() const {
        return nMagicKey;
    }
    inline int GetDeltaFrame() const {
        return nDeltaFrame;
    }
    inline int GetWidth() const {
        return nWidth;
    }
    inline int GetHeight() const {
        return nHeight;
    }
    inline bool IsChromaMotion() const {
        return !!(nMotionFlags & MOTION_USE_CHROMA_MOTION);
    }
    inline int GetOverlapX() const {
        return nOverlapX;
    }
    inline int GetBlkX() const {
        return nBlkX;
    }
    inline int GetBlkY() const {
        return nBlkY;
    }
    inline int GetBitsPerSample() const {
        return bitsPerSample;
    }
    inline int GetYRatioUV() const {
        return yRatioUV;
    }
    inline int GetXRatioUV() const {
        return xRatioUV;
    }
    inline int GetBlkSizeY() const {
        return nBlkSizeY;
    }
    inline int GetOverlapY() const {
        return nOverlapY;
    }
    inline int GetHPadding() const {
        return nHPadding;
    }
    inline int GetVPadding() const {
        return nVPadding;
    }
};


class MVException : public std::runtime_error {
public:
    MVException(const char *descr)
        : std::runtime_error(descr) {
    }
    MVException(const std::string &descr)
        : std::runtime_error(descr) {
    }
};


//#define MOTION_DELTA_FRAME_BUFFER 5

#endif
