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

template <typename PixelType>
void PadReferenceFrame(unsigned char *frame, int pitch, int hPad, int vPad, int width, int height);


template <typename PixelType>
void PadCorner(PixelType *p, PixelType v, int hPad, int vPad, int refPitch)
{
    for (int i = 0; i < vPad; i++) {
        if (sizeof(PixelType) == 1)
            memset(p, v, hPad); // faster than loop
        else
            for (int j = 0; j < hPad; j++)
                p[j] = v;

        p += refPitch;
    }
}


template <typename PixelType>
void PadReferenceFrame(unsigned char *refFrame8, int refPitch, int hPad, int vPad, int width, int height)
{
    refPitch /= sizeof(PixelType);
    PixelType *refFrame = (PixelType *)refFrame8;
    PixelType value;
    PixelType *pfoff = refFrame + vPad * refPitch + hPad;
    PixelType *p;

    // Up-Left
    PadCorner<PixelType>(refFrame, pfoff[0], hPad, vPad, refPitch);
    // Up-Right
    PadCorner<PixelType>(refFrame + hPad + width, pfoff[width - 1], hPad, vPad, refPitch);
    // Down-Left
    PadCorner<PixelType>(refFrame + (vPad + height) * refPitch,
            pfoff[(height - 1) * refPitch], hPad, vPad, refPitch);
    // Down-Right
    PadCorner<PixelType>(refFrame + hPad + width + (vPad + height) * refPitch,
            pfoff[(height - 1) * refPitch + width - 1], hPad, vPad, refPitch);

    // Up
    for ( int i = 0; i < width; i++ )
    {
        value = pfoff[i];
        p = refFrame + hPad + i;
        for ( int j = 0; j < vPad; j++ )
        {
            p[0] = value;
            p += refPitch;
        }
    }

    // Left
    for ( int i = 0; i < height; i++ )
    {
        value = pfoff[i*refPitch];
        p = refFrame + (vPad + i) * refPitch;
        for ( int j = 0; j < hPad; j++ )
            p[j] = value;
    }

    // Right
    for ( int i = 0; i < height; i++ )
    {
        value = pfoff[i * refPitch + width - 1];
        p = refFrame + (vPad + i) * refPitch + width + hPad;
        for ( int j = 0; j < hPad; j++ )
            p[j] = value;
    }

    // Down
    for ( int i = 0; i < width; i++ )
    {
        value = pfoff[i + (height - 1) * refPitch];
        p = refFrame + hPad + i + (height + vPad) * refPitch;
        for ( int j = 0; j < vPad; j++ )
        {
            p[0] = value;
            p += refPitch;
        }
    }
}
#endif
