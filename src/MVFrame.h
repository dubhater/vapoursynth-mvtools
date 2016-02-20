#ifndef MVTOOLS_MVFRAME_H
#define MVTOOLS_MVFRAME_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

typedef enum MVPlaneSet {
    YPLANE = (1 << 0),
    UPLANE = (1 << 1),
    VPLANE = (1 << 2),
    YUPLANES = YPLANE | UPLANE,
    YVPLANES = YPLANE | VPLANE,
    UVPLANES = UPLANE | VPLANE,
    YUVPLANES = YPLANE | UPLANE | VPLANE
} MVPlaneSet;


typedef enum SharpParam {
    SharpBilinear = 0,
    SharpBicubic = 1,
    SharpWiener = 2
} SharpParam;


typedef enum RfilterParam {
    RfilterSimple = 0,
    RfilterTriangle = 1,
    RfilterBilinear = 2,
    RfilterQuadratic = 3,
    RfilterCubic = 4
} RfilterParam;


int PlaneHeightLuma(int src_height, int level, int yRatioUV, int vpad);

int PlaneWidthLuma(int src_width, int level, int xRatioUV, int hpad);

unsigned int PlaneSuperOffset(int chroma, int src_height, int level, int pel, int vpad, int plane_pitch, int yRatioUV);


typedef struct MVPlane {
    uint8_t **pPlane;
    int nWidth;
    int nHeight;
    int nPaddedWidth;
    int nPaddedHeight;
    int nPitch;
    int nHPadding;
    int nVPadding;
    int nOffsetPadding;
    int nHPaddingPel;
    int nVPaddingPel;
    int bitsPerSample;
    int bytesPerSample;

    int nPel;

    int isse;

    int isPadded;
    int isRefined;
    int isFilled;
} MVPlane;

void mvpInit(MVPlane *mvp, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int isse, int bitsPerSample);

void mvpDeinit(MVPlane *mvp);

void mvpResetState(MVPlane *mvp);

void mvpUpdate(MVPlane *mvp, uint8_t *pSrc, int _nPitch);

void mvpFillPlane(MVPlane *mvp, const uint8_t *pNewPlane, int nNewPitch);

void mvpPad(MVPlane *mvp);

void mvpRefine(MVPlane *mvp, int sharp);

void mvpRefineExtPel2_uint8_t(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded);
void mvpRefineExtPel2_uint16_t(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded);

void mvpRefineExtPel4_uint8_t(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded);
void mvpRefineExtPel4_uint16_t(MVPlane *mvp, const uint8_t *pSrc2x8, int nSrc2xPitch, int isExtPadded);

void mvpRefineExt(MVPlane *mvp, const uint8_t *pSrc2x, int nSrc2xPitch, int isExtPadded);

void mvpReduceTo(MVPlane *mvp, MVPlane *pReducedPlane, int rfilter);

const uint8_t *mvpGetAbsolutePointer(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetAbsolutePointerPel1(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetAbsolutePointerPel2(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetAbsolutePointerPel4(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetPointer(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetPointerPel1(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetPointerPel2(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetPointerPel4(const MVPlane *mvp, int nX, int nY);

const uint8_t *mvpGetAbsolutePelPointer(const MVPlane *mvp, int nX, int nY);


typedef struct MVFrame {
    MVPlane *planes[3];

    int nMode;
    int isse;
    int xRatioUV;
    int yRatioUV;
    int bitsPerSample;
} MVFrame;


void mvfInit(MVFrame *mvf, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int nMode, int isse, int xRatioUV, int yRatioUV, int bitsPerSample);

void mvfDeinit(MVFrame *mvf);

void mvfUpdate(MVFrame *mvf, uint8_t **pSrc, int *pitch);

void mvfFillPlane(MVFrame *mvf, const uint8_t *pNewPlane, int nNewPitch, int plane);

void mvfRefine(MVFrame *mvf, MVPlaneSet nMode, int sharp);

void mvfPad(MVFrame *mvf, MVPlaneSet nMode);

void mvfResetState(MVFrame *mvf);

void mvfReduceTo(MVFrame *mvf, MVFrame *pFrame, MVPlaneSet nMode, int rfilter);


typedef struct MVGroupOfFrames {
    int nLevelCount;
    MVFrame **frames;

    int nWidth[3];
    int nHeight[3];
    int nPel;
    int nHPad[3];
    int nVPad[3];
    int xRatioUV;
    int yRatioUV;
    int bitsPerSample;
} MVGroupOfFrames;


void mvgofInit(MVGroupOfFrames *mvgof, int nLevelCount, int nWidth, int nHeight, int nPel, int nHPad, int nVPad, int nMode, int isse, int xRatioUV, int yRatioUV, int bitsPerSample);

void mvgofDeinit(MVGroupOfFrames *mvgof);

void mvgofUpdate(MVGroupOfFrames *mvgof, uint8_t **pSrc, int *pitch);

MVFrame *mvgofGetFrame(MVGroupOfFrames *mvgof, int nLevel);

void mvgofSetPlane(MVGroupOfFrames *mvgof, const uint8_t *pNewSrc, int nNewPitch, int plane);

void mvgofRefine(MVGroupOfFrames *mvgof, MVPlaneSet nMode, int sharp);

void mvgofPad(MVGroupOfFrames *mvgof, MVPlaneSet nMode);

void mvgofReduce(MVGroupOfFrames *mvgof, MVPlaneSet nMode, int rfilter);

void mvgofResetState(MVGroupOfFrames *mvgof);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // MVTOOLS_MVFRAME_H
