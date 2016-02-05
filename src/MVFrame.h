#ifndef MVTOOLS_MVFRAME_H
#define MVTOOLS_MVFRAME_H

#include <cstdint>
#include <cstdio>

enum MVPlaneSet {
    YPLANE = 1,
    UPLANE = 2,
    VPLANE = 4,
    YUPLANES = 3,
    YVPLANES = 5,
    UVPLANES = 6,
    YUVPLANES = 7
};

class MVPlane {
    uint8_t **pPlane;
    int nWidth;
    int nHeight;
    int nExtendedWidth;
    int nExtendedHeight;
    int nPitch;
    int nHPadding;
    int nVPadding;
    int nOffsetPadding;
    int nHPaddingPel;
    int nVPaddingPel;
    int bitsPerSample;
    int bytesPerSample;

    int nPel;

    bool isse;

    bool isPadded;
    bool isRefined;
    bool isFilled;

    //CRITICAL_SECTION cs;

    template <typename PixelType>
    void RefineExtPel2(const uint8_t *pSrc2x, int nSrc2xPitch, bool isExtPadded);

    template <typename PixelType>
    void RefineExtPel4(const uint8_t *pSrc2x, int nSrc2xPitch, bool isExtPadded);

public:
    MVPlane(int _nWidth, int _nHeight, int _nPel, int _nHPad, int _nVPad, bool _isse, int _bitsPerSample);
    ~MVPlane();

    void Update(uint8_t *pSrc, int _nPitch);
    void ChangePlane(const uint8_t *pNewPlane, int nNewPitch);
    void Pad();
    void Refine(int interType);
    void RefineExt(const uint8_t *pSrc2x, int nSrc2xPitch, bool isExtPadded); //2.0.08
    void ReduceTo(MVPlane *pReducedPlane, int rfilter);
    void WritePlane(FILE *pFile);

    inline const uint8_t *GetAbsolutePointer(int nX, int nY) const {
        if (nPel == 1)
            return pPlane[0] + nX * bytesPerSample + nY * nPitch;
        else if (nPel == 2) {
            int idx = (nX & 1) | ((nY & 1) << 1);

            nX >>= 1;
            nY >>= 1;

            return pPlane[idx] + nX * bytesPerSample + nY * nPitch;
        } else { // nPel = 4
            int idx = (nX & 3) | ((nY & 3) << 2);

            nX >>= 2;
            nY >>= 2;

            return pPlane[idx] + nX * bytesPerSample + nY * nPitch;
        }
    }

    inline const uint8_t *GetAbsolutePointerPel1(int nX, int nY) const {
        return pPlane[0] + nX * bytesPerSample + nY * nPitch;
    }

    inline const uint8_t *GetAbsolutePointerPel2(int nX, int nY) const {
        int idx = (nX & 1) | ((nY & 1) << 1);

        nX >>= 1;
        nY >>= 1;

        return pPlane[idx] + nX * bytesPerSample + nY * nPitch;
    }

    inline const uint8_t *GetAbsolutePointerPel4(int nX, int nY) const {
        int idx = (nX & 3) | ((nY & 3) << 2);

        nX >>= 2;
        nY >>= 2;

        return pPlane[idx] + nX * bytesPerSample + nY * nPitch;
    }

    inline const uint8_t *GetPointer(int nX, int nY) const {
        return GetAbsolutePointer(nX + nHPaddingPel, nY + nVPaddingPel);
    }

    inline const uint8_t *GetPointerPel1(int nX, int nY) const {
        return GetAbsolutePointerPel1(nX + nHPaddingPel, nY + nVPaddingPel);
    }

    inline const uint8_t *GetPointerPel2(int nX, int nY) const {
        return GetAbsolutePointerPel2(nX + nHPaddingPel, nY + nVPaddingPel);
    }

    inline const uint8_t *GetPointerPel4(int nX, int nY) const {
        return GetAbsolutePointerPel4(nX + nHPaddingPel, nY + nVPaddingPel);
    }

    inline const uint8_t *GetAbsolutePelPointer(int nX, int nY) const {
        return pPlane[0] + nX * bytesPerSample + nY * nPitch;
    }

    inline int GetPitch() const {
        return nPitch;
    }
    inline int GetWidth() const {
        return nWidth;
    }
    inline int GetHeight() const {
        return nHeight;
    }
    inline int GetExtendedWidth() const {
        return nExtendedWidth;
    }
    inline int GetExtendedHeight() const {
        return nExtendedHeight;
    }
    inline int GetHPadding() const {
        return nHPadding;
    }
    inline int GetVPadding() const {
        return nVPadding;
    }
    inline void ResetState() {
        isRefined = isFilled = isPadded = false;
    }
};

class MVFrame {

    MVPlane *pYPlane;
    MVPlane *pUPlane;
    MVPlane *pVPlane;

    int nMode;
    bool isse;
    int xRatioUV;
    int yRatioUV;
    int bitsPerSample;

public:
    MVFrame(int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int _nMode, bool _isse, int _xRatioUV, int _yRatioUV, int _bitsPerSample);
    ~MVFrame();

    void Update(int _nMode, uint8_t *pSrcY, int pitchY, uint8_t *pSrcU, int pitchU, uint8_t *pSrcV, int pitchV);
    void ChangePlane(const uint8_t *pNewSrc, int nNewPitch, MVPlaneSet _nMode);
    void Refine(MVPlaneSet _nMode, int interType);
    void Pad(MVPlaneSet _nMode);
    void ReduceTo(MVFrame *pFrame, MVPlaneSet _nMode, int rfilter);
    void ResetState();
    void WriteFrame(FILE *pFile);

    inline MVPlane *GetPlane(MVPlaneSet _nMode) {
        // no reason to test for nMode because returning NULL isn't expected in other parts
        // assert(nMode & _nMode & (YPLANE | UPLANE | VPLANE));

        if (_nMode & YPLANE) // ( nMode & _nMode & YPLANE )
            return pYPlane;

        if (_nMode & UPLANE) // ( nMode & _nMode & UPLANE )
            return pUPlane;

        if (_nMode & VPLANE) // ( nMode & _nMode & VPLANE )
            return pVPlane;

        return 0;
    }

    inline int GetMode() {
        return nMode;
    }
};


class MVGroupOfFrames {
    int nLevelCount;
    MVFrame **pFrames;

    int nWidth;
    int nHeight;
    int nPel;
    int nHPad;
    int nVPad;
    int xRatioUV;
    int yRatioUV;
    int bitsPerSample;

public:
    MVGroupOfFrames(int _nLevelCount, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int nMode, bool isse, int _xRatioUV, int yRatioUV, int _bitsPerSample);
    ~MVGroupOfFrames();
    void Update(int nModeYUV, uint8_t *pSrcY, int pitchY, uint8_t *pSrcU, int pitchU, uint8_t *pSrcV, int pitchV);

    MVFrame *GetFrame(int nLevel);
    void SetPlane(const uint8_t *pNewSrc, int nNewPitch, MVPlaneSet nMode);
    void Refine(MVPlaneSet nMode, int interType);
    void Pad(MVPlaneSet nMode);
    void Reduce(MVPlaneSet nMode, int rfilter);
    void ResetState();
};

#endif // MVTOOLS_MVFRAME_H
