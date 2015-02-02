#ifndef MVTOOLS_FAKEGROUPOFPLANES_H
#define MVTOOLS_FAKEGROUPOFPLANES_H

#include "FakePlaneOfBlocks.h"

class FakeGroupOfPlanes {
    int nLvCount_;
    bool validity;
    int nWidth_B;
    int nHeight_B;
    int yRatioUV_B;
    FakePlaneOfBlocks **planes;
    inline static bool GetValidity(const int *array) { return (array[1] == 1); }
    //CRITICAL_SECTION cs;

    public :
    FakeGroupOfPlanes();
    ~FakeGroupOfPlanes();

    void Create(int _nBlkSizeX, int _nBlkSizeY, int _nLevelCount, int _nPel, int _nOverlapX, int _nOverlapY, int _yRatioUV, int _nBlkX, int _nBlkY);

    void Update(const int *array);
    bool IsSceneChange(int nThSCD1, int nThSCD2) const;

    inline const FakePlaneOfBlocks& operator[](const int i) const {
        return *(planes[i]);
    }


    inline bool IsValid() const { return validity; }
    inline int GetPitch() const { return nWidth_B; }
    inline int GetPitchUV() const { return nWidth_B / 2; } // FIXME: lol

    inline const FakePlaneOfBlocks& GetPlane(int i) const { return *(planes[i]); }
};

#endif // MVTOOLS_FAKEGROUPOFPLANES_H

