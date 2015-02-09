// Make a motion compensate temporal denoiser
// Copyright(c)2006 A.G.Balakhnin aka Fizick
// See legal notice in Copying.txt for more information

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


#include <algorithm>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <VapourSynth.h>
#include <VSHelper.h>

#include "MaskFun.h"
#include "MVFilter.h"
#include "SimpleResize.h"


typedef struct {
    VSNodeRef *node;
    const VSVideoInfo *vi;

    VSNodeRef *finest;
    VSNodeRef *super;
    VSNodeRef *mvbw;
    VSNodeRef *mvfw;

    float blur;
    int prec;
    int thscd1;
    int thscd2;
    bool isse;

    MVClipDicks *mvClipB;
    MVClipDicks *mvClipF;

    MVFilter *bleh;

    int nSuperHPad;

    int nWidthUV;
    int nHeightUV;
    int nVPaddingUV;
    int nHPaddingUV;
    int VPitchY;
    int VPitchUV;

    int blur256;

    SimpleResize *upsizer;
    SimpleResize *upsizerUV;
} MVFlowBlurData;


static void VS_CC mvflowblurInit(VSMap *in, VSMap *out, void **instanceData, VSNode *node, VSCore *core, const VSAPI *vsapi) {
    MVFlowBlurData *d = (MVFlowBlurData *) * instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}


template <typename PixelType>
static void RealFlowBlur(uint8_t * pdst8, int dst_pitch, const uint8_t *pref8, int ref_pitch,
        uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF,
        int VPitch, int width, int height, int blur256, int prec, int nPel)
{
    const PixelType *pref = (const PixelType *)pref8;
    PixelType *pdst = (PixelType *)pdst8;

    ref_pitch /= sizeof(PixelType);
    dst_pitch /= sizeof(PixelType);

    // very slow, but precise motion blur
    if (nPel==1)
    {
        for (int h=0; h<height; h++)
        {
            for (int w=0; w<width; w++)
            {
                int bluredsum = pref[w];
                int vxF0 = ((VXFullF[w]-128)*blur256);
                int vyF0 = ((VYFullF[w]-128)*blur256);
                int mF = (std::max(abs(vxF0), abs(vyF0))/prec)>>8;
                if (mF>0)
                {
                    vxF0 /= mF;
                    vyF0 /= mF;
                    int vxF = vxF0;
                    int vyF = vyF0;
                    for (int i=0; i<mF; i++)
                    {
                        int dstF = pref[(vyF>>8)*ref_pitch + (vxF>>8) + w];
                        bluredsum += dstF;
                        vxF += vxF0;
                        vyF += vyF0;
                    }
                }
                int vxB0 = ((VXFullB[w]-128)*blur256);
                int vyB0 = ((VYFullB[w]-128)*blur256);
                int mB = (std::max(abs(vxB0), abs(vyB0))/prec)>>8;
                if (mB>0)
                {
                    vxB0 /= mB;
                    vyB0 /= mB;
                    int vxB = vxB0;
                    int vyB = vyB0;
                    for (int i=0; i<mB; i++)
                    {
                        int dstB = pref[(vyB>>8)*ref_pitch + (vxB>>8) + w];
                        bluredsum += dstB;
                        vxB += vxB0;
                        vyB += vyB0;
                    }
                }
                pdst[w] = bluredsum/(mF+mB+1);
            }
            pdst += dst_pitch;
            pref += ref_pitch;
            VXFullB += VPitch;
            VYFullB += VPitch;
            VXFullF += VPitch;
            VYFullF += VPitch;
        }
    }
    else if (nPel==2)
    {
        for (int h=0; h<height; h++)
        {
            for (int w=0; w<width; w++)
            {
                int bluredsum = pref[w<<1];
                int vxF0 = ((VXFullF[w]-128)*blur256);
                int vyF0 = ((VYFullF[w]-128)*blur256);
                int mF = (std::max(abs(vxF0), abs(vyF0))/prec)>>8;
                if (mF>0)
                {
                    vxF0 /= mF;
                    vyF0 /= mF;
                    int vxF = vxF0;
                    int vyF = vyF0;
                    for (int i=0; i<mF; i++)
                    {
                        int dstF = pref[(vyF>>8)*ref_pitch + (vxF>>8) + (w<<1)];
                        bluredsum += dstF;
                        vxF += vxF0;
                        vyF += vyF0;
                    }
                }
                int vxB0 = ((VXFullB[w]-128)*blur256);
                int vyB0 = ((VYFullB[w]-128)*blur256);
                int mB = (std::max(abs(vxB0), abs(vyB0))/prec)>>8;
                if (mB>0)
                {
                    vxB0 /= mB;
                    vyB0 /= mB;
                    int vxB = vxB0;
                    int vyB = vyB0;
                    for (int i=0; i<mB; i++)
                    {
                        int dstB = pref[(vyB>>8)*ref_pitch + (vxB>>8) + (w<<1)];
                        bluredsum += dstB;
                        vxB += vxB0;
                        vyB += vyB0;
                    }
                }
                pdst[w] = bluredsum/(mF+mB+1);
            }
            pdst += dst_pitch;
            pref += (ref_pitch<<1);
            VXFullB += VPitch;
            VYFullB += VPitch;
            VXFullF += VPitch;
            VYFullF += VPitch;
        }
    }
    else if (nPel==4)
    {
        for (int h=0; h<height; h++)
        {
            for (int w=0; w<width; w++)
            {
                int bluredsum = pref[w<<2];
                int vxF0 = ((VXFullF[w]-128)*blur256);
                int vyF0 = ((VYFullF[w]-128)*blur256);
                int mF = (std::max(abs(vxF0), abs(vyF0))/prec)>>8;
                if (mF>0)
                {
                    vxF0 /= mF;
                    vyF0 /= mF;
                    int vxF = vxF0;
                    int vyF = vyF0;
                    for (int i=0; i<mF; i++)
                    {
                        int dstF = pref[(vyF>>8)*ref_pitch + (vxF>>8) + (w<<2)];
                        bluredsum += dstF;
                        vxF += vxF0;
                        vyF += vyF0;
                    }
                }
                int vxB0 = ((VXFullB[w]-128)*blur256);
                int vyB0 = ((VYFullB[w]-128)*blur256);
                int mB = (std::max(abs(vxB0), abs(vyB0))/prec)>>8;
                if (mB>0)
                {
                    vxB0 /= mB;
                    vyB0 /= mB;
                    int vxB = vxB0;
                    int vyB = vyB0;
                    for (int i=0; i<mB; i++)
                    {
                        int dstB = pref[(vyB>>8)*ref_pitch + (vxB>>8) + (w<<2)];
                        bluredsum += dstB;
                        vxB += vxB0;
                        vyB += vyB0;
                    }
                }
                pdst[w] = bluredsum/(mF+mB+1);
            }
            pdst += dst_pitch;
            pref += (ref_pitch<<2);
            VXFullB += VPitch;
            VYFullB += VPitch;
            VXFullF += VPitch;
            VYFullF += VPitch;
        }
    }
}


static void FlowBlur(uint8_t * pdst, int dst_pitch, const uint8_t *pref, int ref_pitch,
        uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF,
        int VPitch, int width, int height, int blur256, int prec, int nPel, int bitsPerSample) {
    if (bitsPerSample == 8)
        RealFlowBlur<uint8_t>(pdst, dst_pitch, pref, ref_pitch, VXFullB, VXFullF, VYFullB, VYFullF, VPitch, width, height, blur256, prec, nPel);
    else
        RealFlowBlur<uint16_t>(pdst, dst_pitch, pref, ref_pitch, VXFullB, VXFullF, VYFullB, VYFullF, VPitch, width, height, blur256, prec, nPel);
}


static const VSFrameRef *VS_CC mvflowblurGetFrame(int n, int activationReason, void **instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVFlowBlurData *d = (MVFlowBlurData *) * instanceData;

    if (activationReason == arInitial) {
        int off = d->mvClipB->GetDeltaFrame(); // integer offset of reference frame

        if (n - off >= 0 && (n + off < d->vi->numFrames || !d->vi->numFrames)) {
            vsapi->requestFrameFilter(n - off, d->mvbw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvfw, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->finest, frameCtx);
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        uint8_t *pDst[3];
        const uint8_t *pRef[3];
        int nDstPitches[3];
        int nRefPitches[3];

        MVClipBalls ballsF(d->mvClipF, vsapi);
        MVClipBalls ballsB(d->mvClipB, vsapi);

        bool isUsableB = false;
        bool isUsableF = false;

        int off = d->mvClipB->GetDeltaFrame(); // integer offset of reference frame

        if (n - off >= 0 && (n + off < d->vi->numFrames || !d->vi->numFrames)) {
            const VSFrameRef *mvF = vsapi->getFrameFilter(n + off, d->mvfw, frameCtx);
            ballsF.Update(mvF);
            vsapi->freeFrame(mvF);
            isUsableF = ballsF.IsUsable();

            const VSFrameRef *mvB = vsapi->getFrameFilter(n - off, d->mvbw, frameCtx);
            ballsB.Update(mvB);
            vsapi->freeFrame(mvB);
            isUsableB = ballsB.IsUsable();
        }


        if ( isUsableB && isUsableF )
        {
            const VSFrameRef *ref = vsapi->getFrameFilter(n, d->finest, frameCtx); //  ref for  compensation
            VSFrameRef *dst = vsapi->newVideoFrame(d->vi->format, d->vi->width, d->vi->height, ref, core);

            for (int i = 0; i < d->vi->format->numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
            }

            const int nWidth = d->bleh->nWidth;
            const int nHeight = d->bleh->nHeight;
            const int nWidthUV = d->nWidthUV;
            const int nHeightUV = d->nHeightUV;
            const int xRatioUV = d->bleh->xRatioUV;
            const int yRatioUV = d->bleh->yRatioUV;
            const int nBlkX = d->bleh->nBlkX;
            const int nBlkY = d->bleh->nBlkY;
            const int nVPadding = d->bleh->nVPadding;
            const int nHPadding = d->bleh->nHPadding;
            const int nVPaddingUV = d->nVPaddingUV;
            const int nHPaddingUV = d->nHPaddingUV;
            const int nPel = d->bleh->nPel;
            const int blur256 = d->blur256;
            const int prec = d->prec;
            const int VPitchY = d->VPitchY;
            const int VPitchUV = d->VPitchUV;

            int bitsPerSample = d->vi->format->bitsPerSample;
            int bytesPerSample = d->vi->format->bytesPerSample;

            int nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * bytesPerSample * nPel;
            int nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * bytesPerSample * nPel;


            uint8_t *VXFullYB = new uint8_t [nHeight * VPitchY];
            uint8_t *VYFullYB = new uint8_t [nHeight * VPitchY];
            uint8_t *VXFullYF = new uint8_t [nHeight * VPitchY];
            uint8_t *VYFullYF = new uint8_t [nHeight * VPitchY];
            uint8_t *VXSmallYB = new uint8_t [nBlkX * nBlkY];
            uint8_t *VYSmallYB = new uint8_t [nBlkX * nBlkY];
            uint8_t *VXSmallYF = new uint8_t [nBlkX * nBlkY];
            uint8_t *VYSmallYF = new uint8_t [nBlkX * nBlkY];

            // make  vector vx and vy small masks
            // 1. ATTENTION: vectors are assumed SHORT (|vx|, |vy| < 127) !
            // 2. they will be zeroed if not
            // 3. added 128 to all values
            MakeVectorSmallMasks(&ballsB, nBlkX, nBlkY, VXSmallYB, nBlkX, VYSmallYB, nBlkX);
            MakeVectorSmallMasks(&ballsF, nBlkX, nBlkY, VXSmallYF, nBlkX, VYSmallYF, nBlkX);

            // analyse vectors field to detect occlusion

            // upsize (bilinear interpolate) vector masks to fullframe size


            d->upsizer->Resize(VXFullYB, VPitchY, VXSmallYB, nBlkX);
            d->upsizer->Resize(VYFullYB, VPitchY, VYSmallYB, nBlkX);
            d->upsizer->Resize(VXFullYF, VPitchY, VXSmallYF, nBlkX);
            d->upsizer->Resize(VYFullYF, VPitchY, VYSmallYF, nBlkX);

            FlowBlur(pDst[0], nDstPitches[0], pRef[0] + nOffsetY, nRefPitches[0],
                    VXFullYB, VXFullYF, VYFullYB, VYFullYF, VPitchY,
                    nWidth, nHeight, blur256, prec, nPel, bitsPerSample);

            if (d->vi->format->colorFamily != cmGray) {
                uint8_t *VXFullUVB = new uint8_t [nHeightUV * VPitchUV];
                uint8_t *VYFullUVB = new uint8_t [nHeightUV * VPitchUV];

                uint8_t *VXFullUVF = new uint8_t [nHeightUV * VPitchUV];
                uint8_t *VYFullUVF = new uint8_t [nHeightUV * VPitchUV];

                uint8_t *VXSmallUVB = new uint8_t [nBlkX * nBlkY];
                uint8_t *VYSmallUVB = new uint8_t [nBlkX * nBlkY];

                uint8_t *VXSmallUVF = new uint8_t [nBlkX * nBlkY];
                uint8_t *VYSmallUVF = new uint8_t [nBlkX * nBlkY];

                uint8_t *MaskFullUVB = new uint8_t [nHeightUV * VPitchUV];
                uint8_t *MaskFullUVF = new uint8_t [nHeightUV * VPitchUV];

                VectorSmallMaskYToHalfUV(VXSmallYB, nBlkX, nBlkY, VXSmallUVB, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYB, nBlkX, nBlkY, VYSmallUVB, yRatioUV);

                VectorSmallMaskYToHalfUV(VXSmallYF, nBlkX, nBlkY, VXSmallUVF, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYF, nBlkX, nBlkY, VYSmallUVF, yRatioUV);

                d->upsizerUV->Resize(VXFullUVB, VPitchUV, VXSmallUVB, nBlkX);
                d->upsizerUV->Resize(VYFullUVB, VPitchUV, VYSmallUVB, nBlkX);

                d->upsizerUV->Resize(VXFullUVF, VPitchUV, VXSmallUVF, nBlkX);
                d->upsizerUV->Resize(VYFullUVF, VPitchUV, VYSmallUVF, nBlkX);


                FlowBlur(pDst[1], nDstPitches[1], pRef[1] + nOffsetUV, nRefPitches[1],
                        VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, VPitchUV,
                        nWidthUV, nHeightUV, blur256, prec, nPel, bitsPerSample);
                FlowBlur(pDst[2], nDstPitches[2], pRef[2] + nOffsetUV, nRefPitches[2],
                        VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF, VPitchUV,
                        nWidthUV, nHeightUV, blur256, prec, nPel, bitsPerSample);

                delete [] VXFullUVB;
                delete [] VYFullUVB;
                delete [] VXSmallUVB;
                delete [] VYSmallUVB;
                delete [] VXFullUVF;
                delete [] VYFullUVF;
                delete [] VXSmallUVF;
                delete [] VYSmallUVF;
                delete [] MaskFullUVB;
                delete [] MaskFullUVF;
            }

            delete [] VXFullYB;
            delete [] VYFullYB;
            delete [] VXSmallYB;
            delete [] VYSmallYB;
            delete [] VXFullYF;
            delete [] VYFullYF;
            delete [] VXSmallYF;
            delete [] VYSmallYF;

            vsapi->freeFrame(ref);

            return dst;
        }
        else // not usable
        {
            return vsapi->getFrameFilter(n, d->node, frameCtx);
        }
    }

    return 0;
}


static void VS_CC mvflowblurFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVFlowBlurData *d = (MVFlowBlurData *)instanceData;

    delete d->mvClipB;
    delete d->mvClipF;

    delete d->bleh;

    delete d->upsizer;
    if (d->vi->format->colorFamily != cmGray)
        delete d->upsizerUV;

    vsapi->freeNode(d->finest);
    vsapi->freeNode(d->super);
    vsapi->freeNode(d->mvfw);
    vsapi->freeNode(d->mvbw);
    vsapi->freeNode(d->node);
    free(d);
}


static void VS_CC mvflowblurCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    MVFlowBlurData d;
    MVFlowBlurData *data;

    int err;

    d.blur = (float)vsapi->propGetFloat(in, "blur", 0, &err);
    if (err)
        d.blur = 50.0f;

    d.prec = int64ToIntS(vsapi->propGetInt(in, "prec", 0, &err));
    if (err)
        d.prec = 1;

    d.thscd1 = int64ToIntS(vsapi->propGetInt(in, "thscd1", 0, &err));
    if (err)
        d.thscd1 = MV_DEFAULT_SCD1;

    d.thscd2 = int64ToIntS(vsapi->propGetInt(in, "thscd2", 0, &err));
    if (err)
        d.thscd2 = MV_DEFAULT_SCD2;

    d.isse = !!vsapi->propGetInt(in, "isse", 0, &err);
    if (err)
        d.isse = 1;


    if (d.blur < 0.0f || d.blur > 200.0f) {
        vsapi->setError(out, "FlowBlur: blur must be between 0 and 200 % (inclusive).");
        return;
    }

    if (d.prec < 1) {
        vsapi->setError(out, "FlowBlur: prec must be at least 1.");
        return;
    }

    d.blur256 = (int)(d.blur * 256.0f / 200.0f);


    d.super = vsapi->propGetNode(in, "super", 0, NULL);

    char errorMsg[1024];
    const VSFrameRef *evil = vsapi->getFrame(0, d.super, errorMsg, 1024);
    if (!evil) {
        vsapi->setError(out, std::string("FlowBlur: failed to retrieve first frame from super clip. Error message: ").append(errorMsg).c_str());
        vsapi->freeNode(d.super);
        return;
    }
    const VSMap *props = vsapi->getFramePropsRO(evil);
    int evil_err[2];
    int nHeightS = int64ToIntS(vsapi->propGetInt(props, "Super_height", 0, &evil_err[0]));
    d.nSuperHPad = int64ToIntS(vsapi->propGetInt(props, "Super_hpad", 0, &evil_err[1]));
    vsapi->freeFrame(evil);

    for (int i = 0; i < 2; i++)
        if (evil_err[i]) {
            vsapi->setError(out, "FlowBlur: required properties not found in first frame of super clip. Maybe clip didn't come from mv.Super? Was the first frame trimmed away?");
            vsapi->freeNode(d.super);
            return;
        }


    d.mvbw = vsapi->propGetNode(in, "mvbw", 0, NULL);
    d.mvfw = vsapi->propGetNode(in, "mvfw", 0, NULL);

    // XXX Fuck all this trying.
    try {
        d.mvClipB = new MVClipDicks(d.mvbw, d.thscd1, d.thscd2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("FlowBlur: ").append(e.what()).c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.mvfw);
        return;
    }

    try {
        d.mvClipF = new MVClipDicks(d.mvfw, d.thscd1, d.thscd2, vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("FlowBlur: ").append(e.what()).c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        return;
    }

    // XXX Alternatively, use both clips' delta as offsets in GetFrame.
    if (d.mvClipF->GetDeltaFrame() != d.mvClipB->GetDeltaFrame()) {
        vsapi->setError(out, "FlowBlur: mvbw and mvfw must be generated with the same delta.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    // Make sure the motion vector clips are correct.
    if (!d.mvClipB->IsBackward() || d.mvClipF->IsBackward()) {
        if (!d.mvClipB->IsBackward())
            vsapi->setError(out, "FlowBlur: mvbw must be generated with isb=True.");
        else
            vsapi->setError(out, "FlowBlur: mvfw must be generated with isb=False.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    try {
        d.bleh = new MVFilter(d.mvfw, "FlowBlur", vsapi);
    } catch (MVException &e) {
        vsapi->setError(out, std::string("FlowBlur: ").append(e.what()).c_str());
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    try {
        // So it checks the similarity of mvfw and mvfw? ?????
        // Copied straight from 2.5.11.3...
        d.bleh->CheckSimilarity(d.mvClipF, "mvfw");
        d.bleh->CheckSimilarity(d.mvClipB, "mvbw");
    } catch (MVException &e) {
        vsapi->setError(out, std::string("FlowBlur: ").append(e.what()).c_str());
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        return;
    }

    if (d.bleh->nPel == 1)
        d.finest = vsapi->cloneNodeRef(d.super); // v2.0.9.1
    else
    {
        VSPlugin *mvtoolsPlugin = vsapi->getPluginById("com.nodame.mvtools", core);
        VSPlugin *stdPlugin = vsapi->getPluginById("com.vapoursynth.std", core);

        VSMap *args = vsapi->createMap();
        vsapi->propSetNode(args, "super", d.super, paReplace);
        vsapi->propSetInt(args, "isse", d.isse, paReplace);
        VSMap *ret = vsapi->invoke(mvtoolsPlugin, "Finest", args);
        if (vsapi->getError(ret)) {
            vsapi->setError(out, std::string("FlowBlur: ").append(vsapi->getError(ret)).c_str());

            delete d.bleh;
            delete d.mvClipB;
            delete d.mvClipF;
            vsapi->freeNode(d.super);
            vsapi->freeNode(d.mvfw);
            vsapi->freeNode(d.mvbw);
            vsapi->freeMap(args);
            vsapi->freeMap(ret);
            return;
        }
        d.finest = vsapi->propGetNode(ret, "clip", 0, NULL);
        vsapi->freeMap(ret);

        vsapi->clearMap(args);
        vsapi->propSetNode(args, "clip", d.finest, paReplace);
        vsapi->freeNode(d.finest);
        ret = vsapi->invoke(stdPlugin, "Cache", args);
        vsapi->freeMap(args);
        if (vsapi->getError(ret)) {
            // prefix the error messages
            vsapi->setError(out, std::string("FlowBlur: ").append(vsapi->getError(ret)).c_str());

            delete d.bleh;
            delete d.mvClipB;
            delete d.mvClipF;
            vsapi->freeNode(d.super);
            vsapi->freeNode(d.mvfw);
            vsapi->freeNode(d.mvbw);
            vsapi->freeMap(ret);
            return;
        }
        d.finest = vsapi->propGetNode(ret, "clip", 0, NULL);
        vsapi->freeMap(ret);
    }

    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    const VSVideoInfo *supervi = vsapi->getVideoInfo(d.super);
    int nSuperWidth = supervi->width;

    if (d.bleh->nHeight != nHeightS || d.bleh->nWidth != nSuperWidth - d.nSuperHPad * 2) {
        vsapi->setError(out, "FlowBlur: wrong source or super clip frame size.");
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    if (!isConstantFormat(d.vi) || d.vi->format->bitsPerSample > 16 || d.vi->format->sampleType != stInteger || d.vi->format->subSamplingW > 1 || d.vi->format->subSamplingH > 1 || (d.vi->format->colorFamily != cmYUV && d.vi->format->colorFamily != cmGray)) {
        vsapi->setError(out, "FlowBlur: input clip must be GRAY, 420, 422, V440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d.super);
        vsapi->freeNode(d.finest);
        vsapi->freeNode(d.mvfw);
        vsapi->freeNode(d.mvbw);
        vsapi->freeNode(d.node);
        delete d.bleh;
        delete d.mvClipB;
        delete d.mvClipF;
        return;
    }

    if (d.vi->format->bitsPerSample > 8)
        d.isse = 0;


    d.nHeightUV = d.bleh->nHeight / d.bleh->yRatioUV;
    d.nWidthUV = d.bleh->nWidth / d.bleh->xRatioUV;
    d.nHPaddingUV = d.bleh->nHPadding / d.bleh->xRatioUV;
    //d.nVPaddingUV = d.bleh->nHPadding / d.bleh->yRatioUV; // original looks wrong
    d.nVPaddingUV = d.bleh->nVPadding / d.bleh->yRatioUV;

    d.VPitchY = d.bleh->nWidth;
    d.VPitchUV= d.nWidthUV;

    d.upsizer = new SimpleResize(d.bleh->nWidth, d.bleh->nHeight, d.bleh->nBlkX, d.bleh->nBlkY);
    if (d.vi->format->colorFamily != cmGray)
        d.upsizerUV = new SimpleResize(d.nWidthUV, d.nHeightUV, d.bleh->nBlkX, d.bleh->nBlkY);


    data = (MVFlowBlurData *)malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "FlowBlur", mvflowblurInit, mvflowblurGetFrame, mvflowblurFree, fmParallel, 0, data, core);
}


void mvflowblurRegister(VSRegisterFunction registerFunc, VSPlugin *plugin) {
    registerFunc("FlowBlur",
            "clip:clip;"
            "super:clip;"
            "mvbw:clip;"
            "mvfw:clip;"
            "blur:float:opt;"
            "prec:int:opt;"
            "thscd1:int:opt;"
            "thscd2:int:opt;"
            "isse:int:opt;"
            , mvflowblurCreate, 0, plugin);
}
