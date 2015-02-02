#ifndef MVTOOLS_FAKEBLOCKDATA_H
#define MVTOOLS_FAKEBLOCKDATA_H

#include "MVInterface.h"

class FakeBlockData {
    int x;
    int y;
    VECTOR vector;

    public :
    FakeBlockData();
    FakeBlockData(int _x, int _y);
    ~FakeBlockData();

    void Init(int _x, int _y);
    void Update(const int *array);

    inline int GetX() const { return x; }
    inline int GetY() const { return y; }
    inline VECTOR GetMV() const { return vector; }
    inline int GetSAD() const { return vector.sad; }
};

#endif // MVTOOLS_FAKEBLOCKDATA_H

