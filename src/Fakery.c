
#include <stdlib.h>

#include "CommonFunctions.h"
#include "Fakery.h"


// FakeBlockData

void fbdUpdate(FakeBlockData *fbd, const int *array) {
    fbd->vector.x = array[0];
    fbd->vector.y = array[1];
    fbd->vector.sad = array[2];
}


// FakePlaneOfBlocks

void fpobInit(FakePlaneOfBlocks *fpob, int sizeX, int sizeY, int lv, int pel, int nOverlapX, int nOverlapY, int nBlkX, int nBlkY) {
    fpob->nBlkSizeX = sizeX;
    fpob->nBlkSizeY = sizeY;
    fpob->nOverlapX = nOverlapX;
    fpob->nOverlapY = nOverlapY;
    fpob->nBlkX = nBlkX;
    fpob->nBlkY = nBlkY;
    fpob->nWidth_Bi = fpob->nOverlapX + fpob->nBlkX * (fpob->nBlkSizeX - fpob->nOverlapX);  //w;
    fpob->nHeight_Bi = fpob->nOverlapY + fpob->nBlkY * (fpob->nBlkSizeY - fpob->nOverlapY); //h;
    fpob->nBlkCount = fpob->nBlkX * fpob->nBlkY;
    fpob->nPel = pel;

    fpob->nLogPel = ilog2(fpob->nPel);
    fpob->nLogScale = lv;
    fpob->nScale = iexp2(fpob->nLogScale);

    fpob->blocks = (FakeBlockData *)malloc(fpob->nBlkCount * sizeof(FakeBlockData));

    for (int j = 0, blkIdx = 0; j < fpob->nBlkY; j++) {
        for (int i = 0; i < fpob->nBlkX; i++, blkIdx++) {
            fpob->blocks[blkIdx].x = i * (fpob->nBlkSizeX - fpob->nOverlapX);
            fpob->blocks[blkIdx].y = j * (fpob->nBlkSizeY - fpob->nOverlapY);
        }
    }
}


void fpobDeinit(FakePlaneOfBlocks *fpob) {
    free(fpob->blocks);
}


void fpobUpdate(FakePlaneOfBlocks *fpob, const int *array) {
    array += 0;
    for (int i = 0; i < fpob->nBlkCount; i++) {
        fbdUpdate(&fpob->blocks[i], array);
        array += N_PER_BLOCK;
    }
}


int fpobIsSceneChange(const FakePlaneOfBlocks *fpob, int nTh1, int nTh2) {
    int sum = 0;
    for (int i = 0; i < fpob->nBlkCount; i++)
        sum += (fpob->blocks[i].vector.sad > nTh1) ? 1 : 0;

    return (sum > nTh2);
}


const FakeBlockData *fpobGetBlock(const FakePlaneOfBlocks *fpob, int i) {
    return &fpob->blocks[i];
}


// FakeGroupOfPlanes

void fgopInit(FakeGroupOfPlanes *fgop, const MVAnalysisData *ad) {
    fgop->nLvCount = ad->nLvCount;
    int nBlkX1 = ad->nBlkX;
    int nBlkY1 = ad->nBlkY;
    fgop->nWidth_B = (ad->nBlkSizeX - ad->nOverlapX) * nBlkX1 + ad->nOverlapX;
    fgop->nHeight_B = (ad->nBlkSizeY - ad->nOverlapY) * nBlkY1 + ad->nOverlapY;

    fgop->planes = (FakePlaneOfBlocks **)malloc(ad->nLvCount * sizeof(FakePlaneOfBlocks *));

    fgop->planes[0] = (FakePlaneOfBlocks *)malloc(sizeof(FakePlaneOfBlocks));
    fpobInit(fgop->planes[0], ad->nBlkSizeX, ad->nBlkSizeY, 0, ad->nPel, ad->nOverlapX, ad->nOverlapY, nBlkX1, nBlkY1);

    for (int i = 1; i < ad->nLvCount; i++) {
        nBlkX1 = ((fgop->nWidth_B >> i) - ad->nOverlapX) / (ad->nBlkSizeX - ad->nOverlapX);
        nBlkY1 = ((fgop->nHeight_B >> i) - ad->nOverlapY) / (ad->nBlkSizeY - ad->nOverlapY);

        fgop->planes[i] = (FakePlaneOfBlocks *)malloc(sizeof(FakePlaneOfBlocks));
        fpobInit(fgop->planes[i], ad->nBlkSizeX, ad->nBlkSizeY, i, 1, ad->nOverlapX, ad->nOverlapY, nBlkX1, nBlkY1); // fixed bug with nOverlapX in v1.10.2
    }
}


void fgopDeinit(FakeGroupOfPlanes *fgop) {
    if (fgop->planes) {
        for (int i = 0; i < fgop->nLvCount; i++) {
            fpobDeinit(fgop->planes[i]);
            free(fgop->planes[i]);
        }

        free(fgop->planes);
        fgop->planes = 0; //v1.2.1
    }
}


static inline int fgopGetValidity(const int *array) {
    return (array[1] == 1);
}


void fgopUpdate(FakeGroupOfPlanes *fgop, const int *array) {
    const int *pA = array;
    fgop->validity = fgopGetValidity(array);

    pA += 2;
    for (int i = fgop->nLvCount - 1; i >= 0; i--)
        pA += pA[0];

    pA++;

    pA = array;
    pA += 2;
    for (int i = fgop->nLvCount - 1; i >= 0; i--) {
        fpobUpdate(fgop->planes[i], pA + 1);
        pA += pA[0];
    }
}


int fgopIsSceneChange(const FakeGroupOfPlanes *fgop, int nThSCD1, int nThSCD2) {
    return fpobIsSceneChange(fgop->planes[0], nThSCD1, nThSCD2);
}


int fgopIsValid(const FakeGroupOfPlanes *fgop) {
    return fgop->validity;
}


const FakePlaneOfBlocks *fgopGetPlane(const FakeGroupOfPlanes *fgop, int i) {
    return fgop->planes[i];
}


const FakeBlockData *fgopGetBlock(const FakeGroupOfPlanes *fgop, int nLevel, int nBlk) {
    return fpobGetBlock(fgopGetPlane(fgop, nLevel), nBlk);
}


int fgopIsUsable(const FakeGroupOfPlanes *fgop, int thscd1, int thscd2) {
    return !fgopIsSceneChange(fgop, thscd1, thscd2) && fgopIsValid(fgop);
}
