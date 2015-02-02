#ifndef MVTOOLS_MVCLIP_H
#define MVTOOLS_MVCLIP_H

#include "FakeGroupOfPlanes.h"

class MVClipDicks : public MVAnalysisData {
    /*! \brief Number of blocks at the first level */
    int nBlkCount;

    /*! \brief First Scene Change Detection threshold ( compared against SAD value of the block ) */
    int nSCD1;

    /*! \brief Second Scene Change Detection threshold ( compared against the number of block over the first threshold */
    int nSCD2;

    const VSAPI *vsapi;

    public:
    MVClipDicks(VSNodeRef *vectors, int nSCD1, int nSCD2, const VSAPI *_vsapi);
    ~MVClipDicks();

    inline int GetBlkCount() const { return nBlkCount; }
    inline int GetThSCD1() const { return nSCD1; }
    inline int GetThSCD2() const { return nSCD2; }
};


class MVClipBalls : public FakeGroupOfPlanes {
    MVClipDicks *dicks;
    const VSAPI *vsapi;
    public:
    MVClipBalls(MVClipDicks *_dicks, const VSAPI *_vsapi);
    ~MVClipBalls();

    void Update(const VSFrameRef *fn); // v1.4.13
    inline const FakeBlockData& GetBlock(int nLevel, int nBlk) const { return GetPlane(nLevel)[nBlk]; }
    bool IsUsable() const;
    bool IsSceneChange(int nSCD1, int nSCD2) const { return FakeGroupOfPlanes::IsSceneChange(nSCD1, nSCD2); }
};

#endif // MVTOOLS_MVCLIP_H

