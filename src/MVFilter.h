#ifndef MVTOOLS_MVFILTER_H
#define MVTOOLS_MVFILTER_H

#include "MVClip.h"

class MVFilter {
    public:
        /*! \brief Number of blocks horizontaly, at the first level */
        int nBlkX;

        /*! \brief Number of blocks verticaly, at the first level */
        int nBlkY;

        /*! \brief Number of blocks at the first level */
        int nBlkCount;

        /*! \brief Number of blocks at the first level */
        int nBlkSizeX;

        int nBlkSizeY;

        /*! \brief Horizontal padding */
        int nHPadding;

        /*! \brief Vertical padding */
        int nVPadding;

        /*! \brief Width of the frame */
        int nWidth;

        /*! \brief Height of the frame */
        int nHeight;

        /*! \brief MVFrames idx */
        int nIdx;

        /*! \brief pixel refinement of the motion estimation */
        int nPel;

        int nOverlapX;
        int nOverlapY;

        int bitsPerSample;
        int yRatioUV;
        int xRatioUV;

        /*! \brief Filter's name */
        const char * name; //v1.8 replaced std::string (why it was used?)

        MVFilter(VSNodeRef *vector, const char *filterName, const VSAPI *vsapi);

        void CheckSimilarity(const MVClipDicks *vector, const char *vectorName);
};

#endif // MVTOOLS_MVFILTER_H

