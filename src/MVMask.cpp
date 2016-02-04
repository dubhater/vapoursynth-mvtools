// Create an overlay mask with the motion vectors
// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick - YUY2, occlusion
// See legal notice in Copying.txt for more information
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .

#include <math.h>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "MaskFun.h"
#include "MVFilter.h"
#include "SimpleResize.h"



typedef struct {
    VSNodeRef *node;
    VSVideoInfo vi;

    VSNodeRef *vectors;
    float ml;
    float fGamma;
    int kind;
    int nSceneChangeValue;
    int thscd1;
    int thscd2;

    float fMaskNormFactor;
    float fMaskNormFactor2;
    float fHalfGamma;

    int nWidthUV;
    int nHeightUV;
    int nWidthB;
    int nHeightB;
    int nWidthBUV;
    int nHeightBUV;

    MVFilter *bleh;

    MVClipDicks *mvClip;

    SimpleResize *upsizer;
    SimpleResize *upsizerUV;
} MVMaskData;


static void VS_CC mvmaskInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVMaskData *d = (MVMaskData *) * instanceData;
    vsapi->setVideoInfo(&d->vi, 1, node);
}


static inline uint8_t mvmaskLength(VECTOR v, uint8_t pel, float fMaskNormFactor2, float fHalfGamma)
{
    double norme = double(v.x * v.x + v.y * v.y) / ( pel * pel);

    double l = 255 * pow(norme*fMaskNormFactor2, fHalfGamma); //Fizick - simply rewritten

    return (uint8_t)((l > 255) ? 255 : l);
}

static inline uint8_t mvmaskSAD(unsigned int s, float fMaskNormFactor, float fGamma, int nBlkSizeX, int nBlkSizeY)
{
    double l = 255 * pow((s*4*fMaskNormFactor)/(nBlkSizeX*nBlkSizeY), fGamma); // Fizick - now linear for gm=1
    return (uint8_t)((l > 255) ? 255 : l);
}


static const VSFrameRef *VS_CC mvmaskGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVMaskData *d = (MVMaskData *) * instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrameRef *dst = vsapi->newVideoFrame(d->vi.format, d->vi.width, d->vi.height, src, core);

        const uint8_t *pSrc[3];
        uint8_t *pDst[3];
        int nDstPitches[3];
        int nSrcPitches[3];

        pSrc[0] = vsapi->getReadPtr(src, 0);
        nSrcPitches[0] = vsapi->getStride(src, 0);

        for (int i = 0; i < 3; i++) {
            pDst[i] = vsapi->getWritePtr(dst, i);
            nDstPitches[i] = vsapi->getStride(dst, i);
        }

        const VSFrameRef *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        MVClipBalls balls(d->mvClip, vsapi);
        balls.Update(mvn);
        vsapi->freeFrame(mvn);

        const int kind = d->kind;
        const int nWidth = d->bleh->nWidth;
        const int nHeight = d->bleh->nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int nSceneChangeValue = d->nSceneChangeValue;

        if ( balls.IsUsable() )
        {
            const int nBlkX = d->bleh->nBlkX;
            const int nBlkY = d->bleh->nBlkY;
            const int nBlkCount = d->bleh->nBlkCount;
            const float fMaskNormFactor = d->fMaskNormFactor;
            const float fMaskNormFactor2 = d->fMaskNormFactor2;
            const float fGamma = d->fGamma;
            const float fHalfGamma = d->fHalfGamma;
            const int nPel = d->bleh->nPel;
            const int nBlkSizeX = d->bleh->nBlkSizeX;
            const int nBlkSizeY = d->bleh->nBlkSizeY;
            const int nOverlapX = d->bleh->nOverlapX;
            const int nOverlapY = d->bleh->nOverlapY;
            const int nWidthB = d->nWidthB;
            const int nHeightB = d->nHeightB;
            const int nWidthBUV = d->nWidthBUV;
            const int nHeightBUV = d->nHeightBUV;
            SimpleResize *upsizer = d->upsizer;
            SimpleResize *upsizerUV = d->upsizerUV;

            uint8_t *smallMask = new uint8_t[nBlkX * nBlkY];
            uint8_t *smallMaskV = new uint8_t[nBlkX * nBlkY];

            if (kind == 0) // vector length mask
            {
                for ( int j = 0; j < nBlkCount; j++)
                    smallMask[j] = mvmaskLength(balls.GetBlock(0, j).GetMV(), d->mvClip->GetPel(), fMaskNormFactor2, fHalfGamma);
            }
            else if (kind == 1) // SAD mask
            {
                for ( int j = 0; j < nBlkCount; j++)
                    smallMask[j] = mvmaskSAD(balls.GetBlock(0, j).GetSAD(), fMaskNormFactor, fGamma, nBlkSizeX, nBlkSizeY);
            }
            else if (kind == 2) // occlusion mask
            {
                MakeVectorOcclusionMaskTime(&balls, nBlkX, nBlkY, fMaskNormFactor, fGamma, nPel, smallMask, nBlkX, 256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY) ;
            }
            else if (kind == 3) // vector x mask
            {
                for ( int j = 0; j < nBlkCount; j++)
                    smallMask[j] = balls.GetBlock(0, j).GetMV().x + 128; // shited by 128 for signed support
            }
            else if (kind == 4) // vector y mask
            {
                for ( int j = 0; j < nBlkCount; j++)
                    smallMask[j] = balls.GetBlock(0, j).GetMV().y + 128; // shited by 128 for signed support
            }
            else if (kind == 5) // vector x mask in U, y mask in V
            {
                for ( int j = 0; j < nBlkCount; j++) {
                    VECTOR v = balls.GetBlock(0, j).GetMV();
                    smallMask[j] = v.x + 128; // shited by 128 for signed support
                    smallMaskV[j] = v.y + 128; // shited by 128 for signed support
                }
            }

            if (kind == 5) { // do not change luma for kind=5
                memcpy(pDst[0], pSrc[0], nSrcPitches[0] * nHeight);
            }
            else {
                upsizer->Resize(pDst[0], nDstPitches[0], smallMask, nBlkX);
                if (nWidth > nWidthB)
                    for (int h = 0; h < nHeight; h++)
                        for (int w = nWidthB; w < nWidth; w++)
                            *(pDst[0] + h * nDstPitches[0] + w) = *(pDst[0] + h * nDstPitches[0] + nWidthB - 1);
                if (nHeight > nHeightB)
                    vs_bitblt(pDst[0] + nHeightB * nDstPitches[0], nDstPitches[0], pDst[0] + (nHeightB - 1) * nDstPitches[0], nDstPitches[0], nWidth, nHeight - nHeightB);
            }

            // chroma
            upsizerUV->Resize(pDst[1], nDstPitches[1], smallMask, nBlkX);

            if (kind == 5)
                upsizerUV->Resize(pDst[2], nDstPitches[2], smallMaskV, nBlkX);
            else
                memcpy(pDst[2], pDst[1], nHeightUV * nDstPitches[1]);

            if (nWidthUV > nWidthBUV)
                for (int h = 0; h < nHeightUV; h++)
                    for (int w = nWidthBUV; w < nWidthUV; w++)
                    {
                        *(pDst[1] + h * nDstPitches[1] + w) = *(pDst[1] + h * nDstPitches[1] + nWidthBUV - 1);
                        *(pDst[2] + h * nDstPitches[2] + w) = *(pDst[2] + h * nDstPitches[2] + nWidthBUV - 1);
                    }
            if (nHeightUV > nHeightBUV)
            {
                vs_bitblt(pDst[1] + nHeightBUV * nDstPitches[1], nDstPitches[1], pDst[1] + (nHeightBUV - 1) * nDstPitches[1], nDstPitches[1], nWidthUV, nHeightUV - nHeightBUV);
                vs_bitblt(pDst[2] + nHeightBUV * nDstPitches[2], nDstPitches[2], pDst[2] + (nHeightBUV - 1) * nDstPitches[2], nDstPitches[2], nWidthUV, nHeightUV - nHeightBUV);
            }

            delete[] smallMask;
            delete[] smallMaskV;
        } else { // not usable
            if (kind == 5)
                memcpy(pDst[0], pSrc[0], nSrcPitches[0] * nHeight);
            else
                memset(pDst[0], nSceneChangeValue, nHeight * nDstPitches[0]);

            memset(pDst[1], nSceneChangeValue, nHeightUV * nDstPitches[1]);
            memset(pDst[2], nSceneChangeValue, nHeightUV * nDstPitches[2]);
        }

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}


static void VS_CC mvmaskFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVMaskData *d = (MVMaskData *)instanceData;

    vsapi->freeNode(d->node);
    vsapi->freeNode(d->vectors);
    delete d->mvClip;
    delete d->bleh;
    delete d->upsizer;
    delete d->upsizerUV;
    free(d);
}


static void VS_CC mvmaskCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVMaskData d;
    MVMaskData *data;

    int err;

    d.ml = (float)vsapi->propGetFloat(in, "ml", 0, &err);
    if (err)
        d.ml = 100.0f;

    d.fGamma = (float)vsapi->propGetFloat(in, "gamma", 0, &err);
    if (err)
        d.fGamma = 1.0f;

    d.kind = int64ToIntS(vsapi->propGetInt(in, "kind", 0, &err));

    d.nSceneChangeValue = int64ToIntS(vsapi->propGetInt(in, "ysc", 0, &err));

    d.thscd1 = int64ToIntS(vsapi->propGetInt(in, "thscd1", 0, &err));
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;


    if (d.fGamma < 0.0f) {
        vsapi->setError(out, "Mask: gamma must not be negative.");
        return;
    }

    if (d.kind < 0 || d.kind > 5) {
        vsapi->setError(out, "Mask: kind must 0, 1, 2, 3, 4, or 5.");
        return;
    }

    if (d.nSceneChangeValue < 0 || d.nSceneChangeValue > 255) {
        vsapi->setError(out, "Mask: ysc must be between 0 and 255 (inclusive).");
        return;
    }


    d.vectors = vsapi->propGetNode(in, "vectors", 0, NULL);

    try {
        d.mvClip = new MVClipDicks(d.vectors, d.thscd1, d.thscd2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.vectors);
        return;
    }

    try {
        d.bleh = new MVFilter(d.vectors, "Mask", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, e.what());
        vsapi->freeNode(d.vectors);
        delete d.mvClip;
        return;
    }

    d.fMaskNormFactor = 1.0f / d.ml; // Fizick
    d.fMaskNormFactor2 = d.fMaskNormFactor * d.fMaskNormFactor;

    d.fHalfGamma = d.fGamma * 0.5f;

    d.nWidthB = d.bleh->nBlkX * (d.bleh->nBlkSizeX - d.bleh->nOverlapX) + d.bleh->nOverlapX;
    d.nHeightB = d.bleh->nBlkY * (d.bleh->nBlkSizeY - d.bleh->nOverlapY) + d.bleh->nOverlapY;

    d.nHeightUV = d.bleh->nHeight / d.bleh->yRatioUV;
    d.nWidthUV = d.bleh->nWidth / d.bleh->xRatioUV;
    d.nHeightBUV = d.nHeightB / d.bleh->yRatioUV;
    d.nWidthBUV = d.nWidthB / d.bleh->xRatioUV;


    d.node = vsapi->propGetNode(in, "clip", 0, NULL);
    d.vi = *vsapi->getVideoInfo(d.node);

    if (!isConstantFormat(&d.vi) || d.vi.format->bitsPerSample > 8 || d.vi.format->subSamplingW > 1 || d.vi.format->subSamplingH > 1 || (d.vi.format->colorFamily != cmYUV && d.vi.format->colorFamily != cmGray)) {
        vsapi->setError(out, "Mask: input clip must be GRAY8, YUV420P8, YUV422P8, YUV440P8, or YUV444P8, with constant dimensions.");
        vsapi->freeNode(d.node);
        vsapi->freeNode(d.vectors);
        delete d.mvClip;
        delete d.bleh;
        return;
    }

    if (d.vi.format->colorFamily == cmGray)
        d.vi.format = vsapi->getFormatPreset(pfYUV444P8, core);

    d.upsizer = new SimpleResize(d.nWidthB, d.nHeightB, d.bleh->nBlkX, d.bleh->nBlkY);
    d.upsizerUV = new SimpleResize(d.nWidthBUV, d.nHeightBUV, d.bleh->nBlkX, d.bleh->nBlkY);


    data = (MVMaskData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "Mask", mvmaskInit, mvmaskGetFrame, mvmaskFree, fmParallel, 0, data, core);
}


extern "C" void mvmaskRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("Mask",
            "clip:clip;"
            "vectors:clip;"
            "ml:float:opt;"
            "gamma:float:opt;"
            "kind:int:opt;"
            "ysc:int:opt;"
            "thscd1:int:opt;"
            "thscd2:int:opt;"
            , mvmaskCreate, 0, plugin);
}
