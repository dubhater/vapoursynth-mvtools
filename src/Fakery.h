#ifndef MVTOOLS_FAKERY_H
#define MVTOOLS_FAKERY_H

#ifdef __cplusplus
extern "C" {
#endif


#include "MVAnalysisData.h"


typedef struct FakeBlockData {
    int x;
    int y;
    VECTOR vector;
} FakeBlockData;


typedef struct FakePlaneOfBlocks {
    int nWidth_Bi;
    int nHeight_Bi;
    int nBlkX;
    int nBlkY;
    int nBlkSizeX;
    int nBlkSizeY;
    int nBlkCount;
    int nPel;
    int nLogPel;
    int nScale;
    int nLogScale;
    int nOverlapX;
    int nOverlapY;

    FakeBlockData *blocks;
} FakePlaneOfBlocks;


typedef struct FakeGroupOfPlanes {
    int nLvCount;
    int validity;
    int nWidth_B;
    int nHeight_B;

    FakePlaneOfBlocks **planes;
} FakeGroupOfPlanes;


// FakeBlockData

void fbdUpdate(FakeBlockData *fbd, const int *array);


// FakePlaneOfBlocks

void fpobInit(FakePlaneOfBlocks *fpob, int sizeX, int sizeY, int lv, int pel, int nOverlapX, int nOverlapY, int nBlkX, int nBlkY);

void fpobDeinit(FakePlaneOfBlocks *fpob);

void fpobUpdate(FakePlaneOfBlocks *fpob, const int *array);

int fpobIsSceneChange(const FakePlaneOfBlocks *fpob, int nTh1, int nTh2);

const FakeBlockData *fpobGetBlock(const FakePlaneOfBlocks *fpob, int i);


// FakeGroupOfPlanes

void fgopInit(FakeGroupOfPlanes *fgop, const MVAnalysisData *ad);

void fgopDeinit(FakeGroupOfPlanes *fgop);

void fgopUpdate(FakeGroupOfPlanes *fgop, const int *array);

int fgopIsSceneChange(const FakeGroupOfPlanes *fgop, int nThSCD1, int nThSCD2);

int fgopIsValid(const FakeGroupOfPlanes *fgop);

const FakePlaneOfBlocks *fgopGetPlane(const FakeGroupOfPlanes *fgop, int i);

const FakeBlockData *fgopGetBlock(const FakeGroupOfPlanes *fgop, int nLevel, int nBlk);

int fgopIsUsable(const FakeGroupOfPlanes *fgop, int thscd1, int thscd2);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // MVTOOLS_FAKERY_H
