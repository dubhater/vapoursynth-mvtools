
#include <stdlib.h>
#include <string.h>

#include "CommonFunctions.h"
#include "Fakery.h"


// FakeBlockData

void fbdUpdate(FakeBlockData *fbd, const VECTOR *array) {
    fbd->vector = *array;
}


// FakePlaneOfBlocks

void fpobInit(FakePlaneOfBlocks *fpob, int sizeX, int sizeY, int pel, int nOverlapX, int nOverlapY, int nBlkX, int nBlkY) {
    fpob->nBlkSizeX = sizeX;
    fpob->nBlkSizeY = sizeY;
    fpob->nOverlapX = nOverlapX;
    fpob->nOverlapY = nOverlapY;
    fpob->nBlkX = nBlkX;
    fpob->nBlkY = nBlkY;
    fpob->nBlkCount = fpob->nBlkX * fpob->nBlkY;
    fpob->nPel = pel;

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


void fpobUpdate(FakePlaneOfBlocks *fpob, const uint8_t *array) {
    const VECTOR *blocks = (const VECTOR *)array;

    for (int i = 0; i < fpob->nBlkCount; i++)
        fbdUpdate(&fpob->blocks[i], &blocks[i]);
}


int fpobIsSceneChange(const FakePlaneOfBlocks *fpob, int64_t nTh1, int nTh2) {
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
    int nWidth_B = (ad->nBlkSizeX - ad->nOverlapX) * nBlkX1 + ad->nOverlapX;
    int nHeight_B = (ad->nBlkSizeY - ad->nOverlapY) * nBlkY1 + ad->nOverlapY;

    fgop->planes = (FakePlaneOfBlocks **)malloc(ad->nLvCount * sizeof(FakePlaneOfBlocks *));

    fgop->planes[0] = (FakePlaneOfBlocks *)malloc(sizeof(FakePlaneOfBlocks));
    fpobInit(fgop->planes[0], ad->nBlkSizeX, ad->nBlkSizeY, ad->nPel, ad->nOverlapX, ad->nOverlapY, nBlkX1, nBlkY1);

    for (int i = 1; i < ad->nLvCount; i++) {
        nBlkX1 = ((nWidth_B >> i) - ad->nOverlapX) / (ad->nBlkSizeX - ad->nOverlapX);
        nBlkY1 = ((nHeight_B >> i) - ad->nOverlapY) / (ad->nBlkSizeY - ad->nOverlapY);

        fgop->planes[i] = (FakePlaneOfBlocks *)malloc(sizeof(FakePlaneOfBlocks));
        fpobInit(fgop->planes[i], ad->nBlkSizeX, ad->nBlkSizeY, 1, ad->nOverlapX, ad->nOverlapY, nBlkX1, nBlkY1); // fixed bug with nOverlapX in v1.10.2
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


static inline int fgopGetValidity(const uint8_t *array) {
    MVArraySizeType validity;
    memcpy(&validity, array + sizeof(MVArraySizeType), sizeof(validity));
    return (validity == 1);
}


void fgopUpdate(FakeGroupOfPlanes *fgop, const uint8_t *array) {
    fgop->validity = fgopGetValidity(array);

    const uint8_t *pA = array + 2 * sizeof(MVArraySizeType);
    for (int i = fgop->nLvCount - 1; i >= 0; i--) {
        fpobUpdate(fgop->planes[i], pA + sizeof(MVArraySizeType));

        MVArraySizeType size;
        memcpy(&size, pA, sizeof(size));
        pA += size;
    }
}


int fgopIsSceneChange(const FakeGroupOfPlanes *fgop, int64_t nThSCD1, int nThSCD2) {
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


int fgopIsUsable(const FakeGroupOfPlanes *fgop, int64_t thscd1, int thscd2) {
    return !fgopIsSceneChange(fgop, thscd1, thscd2) && fgopIsValid(fgop);
}
