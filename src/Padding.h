#ifndef __PADDING_H__
#define __PADDING_H__

#if 0
#include "windows.h"
#include "avisynth.h"
#include <stdio.h>
#include "yuy2planes.h"

class Padding : public GenericVideoFilter {
    private:
        int horizontalPadding;
        int verticalPadding;
        bool planar;

        int width;
        int height;
        YUY2Planes *DstPlanes;
        YUY2Planes *SrcPlanes;

    public:
        Padding(PClip _child, int hPad, int vPad, bool _planar, IScriptEnvironment* env);
        ~Padding();
        PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
        static void PadReferenceFrame(unsigned char *frame, int pitch, int hPad, int vPad, int width, int height);
};
#endif

void PadReferenceFrame(unsigned char *frame, int pitch, int hPad, int vPad, int width, int height);

#endif
