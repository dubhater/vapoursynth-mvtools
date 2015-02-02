#ifndef MVTOOLS_FAKEPLANEOFBLOCKS_H
#define MVTOOLS_FAKEPLANEOFBLOCKS_H

#include "FakeBlockData.h"

class FakePlaneOfBlocks {

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

    public :

    FakePlaneOfBlocks(int sizex,  int sizey, int lv, int pel, int overlapx, int overlapy, int nBlkX, int nBlkY);
    ~FakePlaneOfBlocks();

    void Update(const int *array);
    bool IsSceneChange(int nTh1, int nTh2) const;

    inline bool IsInFrame(int i) const
    {
        return (( i >= 0 ) && ( i < nBlkCount ));
    }

    inline const FakeBlockData& operator[](const int i) const {
        return (blocks[i]);
    }

    inline int GetBlockCount() const { return nBlkCount; }
    inline int GetReducedWidth() const { return nBlkX; }
    inline int GetReducedHeight() const { return nBlkY; }
    inline int GetWidth() const { return nWidth_Bi; }
    inline int GetHeight() const { return nHeight_Bi; }
    inline int GetScaleLevel() const { return nLogScale; }
    inline int GetEffectiveScale() const { return nScale; }
    inline int GetBlockSizeX() const { return nBlkSizeX; }
    inline int GetBlockSizeY() const { return nBlkSizeY; }
    inline int GetPel() const { return nPel; }
    inline const FakeBlockData& GetBlock(int i) const { return (blocks[i]); }
    inline int GetOverlapX() const { return nOverlapX; }
    inline int GetOverlapY() const { return nOverlapY; }
};

#endif // MVTOOLS_FAKEPLANEOFBLOCKS_H

