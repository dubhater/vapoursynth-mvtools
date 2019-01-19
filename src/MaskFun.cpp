// Create an overlay mask with the motion vectors
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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "CommonFunctions.h"
#include "CPU.h"
#include "MaskFun.h"


extern uint32_t g_cpuinfo;


#if defined(MVTOOLS_X86)
void selectFlowInterFunctions_AVX2(FlowInterSimpleFunction *simple, FlowInterFunction *regular, FlowInterExtraFunction *extra, int bitsPerSample);
#endif


#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) > (b) ? (b) : (a))


void CheckAndPadSmallY(int16_t *VXSmallY, int16_t *VYSmallY, int nBlkXP, int nBlkYP, int nBlkX, int nBlkY) {
    if (nBlkXP > nBlkX) { // fill right
        for (int j = 0; j < nBlkY; j++) {
            int16_t VXright = min(VXSmallY[j * nBlkXP + nBlkX - 1], (int16_t)0); // not positive
            int16_t VYright = VYSmallY[j * nBlkXP + nBlkX - 1];
            // clone: multiple 2.7.30-
            for (int dx = nBlkX; dx < nBlkXP; dx++) {
                VXSmallY[j * nBlkXP + dx] = VXright;
                VYSmallY[j * nBlkXP + dx] = VYright;
            }
        }
    }
    if (nBlkYP > nBlkY) { // fill bottom
        for (int i = 0; i < nBlkXP; i++) {
            int16_t VXbottom = VXSmallY[nBlkXP * (nBlkY - 1) + i];
            int16_t VYbottom = min(VYSmallY[nBlkXP * (nBlkY - 1) + i], (int16_t)0);
            for (int dy = nBlkY; dy < nBlkYP; dy++) {
                VXSmallY[nBlkXP * dy + i] = VXbottom;
                VYSmallY[nBlkXP * dy + i] = VYbottom;
            }
        }
    }
}


void CheckAndPadMaskSmall(uint8_t *MaskSmall, int nBlkXP, int nBlkYP, int nBlkX, int nBlkY) {
    if (nBlkXP > nBlkX) { // fill right
        for (int j = 0; j < nBlkY; j++) {
            uint8_t right = MaskSmall[j * nBlkXP + nBlkX - 1];
            // clone: multiple 2.7.30-
            for (int dx = nBlkX; dx < nBlkXP; dx++) {
                MaskSmall[j * nBlkXP + dx] = right;
            }
        }
    }
    if (nBlkYP > nBlkY) { // fill bottom
        for (int i = 0; i < nBlkXP; i++) {
            uint8_t bottom = MaskSmall[nBlkXP * (nBlkY - 1) + i];
            // clone: multiple 2.7.30-
            for (int dy = nBlkY; dy < nBlkYP; dy++) {
                MaskSmall[nBlkXP * dy + i] = bottom;
            }
        }
    }
}


static inline void ByteOccMask(uint8_t *occMask, int occlusion, double occnorm, double fGamma) {
    if (fGamma == 1.0)
        *occMask = max(*occMask, min((int)(255 * occlusion * occnorm), 255));
    else
        *occMask = max(*occMask, min((int)(255 * pow(occlusion * occnorm, fGamma)), 255));
}

void MakeVectorOcclusionMaskTime(const FakeGroupOfPlanes *fgop, int isBackward, int nBlkX, int nBlkY, double dMaskNormDivider, double fGamma, int nPel, uint8_t *occMask, int occMaskPitch, int time256, int nBlkStepX, int nBlkStepY) { // analyse vectors field to detect occlusion
    memset(occMask, 0, occMaskPitch * nBlkY);
    int time4096X = time256 * 16 / (nBlkStepX * nPel);
    int time4096Y = time256 * 16 / (nBlkStepY * nPel);
    double occnormX = 80.0 / (dMaskNormDivider * nBlkStepX * nPel);
    double occnormY = 80.0 / (dMaskNormDivider * nBlkStepY * nPel);
    int occlusion;

    for (int by = 0; by < nBlkY; by++) {
        for (int bx = 0; bx < nBlkX; bx++) {
            int i = bx + by * nBlkX; // current block
            const FakeBlockData *block = fgopGetBlock(fgop, 0, i);
            int vx = block->vector.x;
            int vy = block->vector.y;
            if (bx < nBlkX - 1) { // right neighbor
                int i1 = i + 1;
                const FakeBlockData *block1 = fgopGetBlock(fgop, 0, i1);
                int vx1 = block1->vector.x;
                if (vx1 < vx) {
                    occlusion = vx - vx1;
                    int minb = isBackward ? max(0, bx + 1 - occlusion * time4096X / 4096) : bx;
                    int maxb = isBackward ? bx + 1 : min(bx + 1 - occlusion * time4096X / 4096, nBlkX - 1);
                    for (int bxi = minb; bxi <= maxb; bxi++)
                        ByteOccMask(&occMask[bxi + by * occMaskPitch], occlusion, occnormX, fGamma);
                }
            }
            if (by < nBlkY - 1) { // bottom neighbor
                int i1 = i + nBlkX;
                const FakeBlockData *block1 = fgopGetBlock(fgop, 0, i1);
                int vy1 = block1->vector.y;
                if (vy1 < vy) {
                    occlusion = vy - vy1;
                    int minb = isBackward ? max(0, by + 1 - occlusion * time4096Y / 4096) : by;
                    int maxb = isBackward ? by + 1 : min(by + 1 - occlusion * time4096Y / 4096, nBlkY - 1);
                    for (int byi = minb; byi <= maxb; byi++)
                        ByteOccMask(&occMask[bx + byi * occMaskPitch], occlusion, occnormY, fGamma);
                }
            }
        }
    }
}


static unsigned char ByteNorm(int64_t sad, double dSADNormFactor, double fGamma) {
    //	    double dSADNormFactor = 4 / (dMaskNormFactor*blkSizeX*blkSizeY);
    double l = 255 * pow(sad * dSADNormFactor, fGamma); // Fizick - now linear for gm=1
    return (unsigned char)((l > 255) ? 255 : l);
}


void MakeSADMaskTime(const FakeGroupOfPlanes *fgop, int nBlkX, int nBlkY, double dSADNormFactor, double fGamma, int nPel, uint8_t *Mask, int MaskPitch, int time256, int nBlkStepX, int nBlkStepY, int bitsPerSample) {
    // Make approximate SAD mask at intermediate time
    //    double dSADNormFactor = 4 / (dMaskNormDivider*nBlkSizeX*nBlkSizeY);
    memset(Mask, 0, nBlkY * MaskPitch);
    int time4096X = (256 - time256) * 16 / (nBlkStepX * nPel); // blkstep here is really blksize-overlap
    int time4096Y = (256 - time256) * 16 / (nBlkStepY * nPel);

    for (int by = 0; by < nBlkY; by++) {
        for (int bx = 0; bx < nBlkX; bx++) {
            int i = bx + by * nBlkX; // current block
            const FakeBlockData *block = fgopGetBlock(fgop, 0, i);
            int vx = block->vector.x;
            int vy = block->vector.y;
            int bxi = bx - vx * time4096X / 4096; // limits?
            int byi = by - vy * time4096Y / 4096;
            if (bxi < 0 || bxi >= nBlkX || byi < 0 || byi >= nBlkY) {
                bxi = bx;
                byi = by;
            }
            int i1 = bxi + byi * nBlkX;
            int64_t sad = fgopGetBlock(fgop, 0, i1)->vector.sad >> (bitsPerSample - 8);
            Mask[bx + by * MaskPitch] = ByteNorm(sad, dSADNormFactor, fGamma);
        }
    }
}


void MakeVectorSmallMasks(const FakeGroupOfPlanes *fgop, int nBlkX, int nBlkY, int16_t *VXSmallY, int pitchVXSmallY, int16_t *VYSmallY, int pitchVYSmallY) {
    // make  vector vx and vy small masks
    for (int by = 0; by < nBlkY; by++) {
        for (int bx = 0; bx < nBlkX; bx++) {
            int i = bx + by * nBlkX;
            const FakeBlockData *block = fgopGetBlock(fgop, 0, i);
            int vx = block->vector.x;
            int vy = block->vector.y;
            VXSmallY[bx + by * pitchVXSmallY] = vx; // luma
            VYSmallY[bx + by * pitchVYSmallY] = vy; // luma
        }
    }
}

void VectorSmallMaskYToHalfUV(int16_t *VSmallY, int nBlkX, int nBlkY, int16_t *VSmallUV, int ratioUV) {
    if (ratioUV == 2) {
        // YV12 colorformat
        for (int by = 0; by < nBlkY; by++) {
            for (int bx = 0; bx < nBlkX; bx++) {
                VSmallUV[bx] = VSmallY[bx] >> 1; // chroma
            }
            VSmallY += nBlkX;
            VSmallUV += nBlkX;
        }
    } else { // ratioUV==1
        // Height YUY2 colorformat
        for (int by = 0; by < nBlkY; by++) {
            for (int bx = 0; bx < nBlkX; bx++) {
                VSmallUV[bx] = VSmallY[bx]; // chroma
            }
            VSmallY += nBlkX;
            VSmallUV += nBlkX;
        }
    }
}


// copy refined planes to big one plane
template <typename PixelType>
static void RealMerge4PlanesToBig(uint8_t *pel2Plane_u8, int pel2Pitch, const uint8_t *pPlane0_u8, const uint8_t *pPlane1_u8,
                           const uint8_t *pPlane2_u8, const uint8_t *pPlane3_u8, int width, int height, int pitch) {
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            PixelType *pel2Plane = (PixelType *)pel2Plane_u8;
            const PixelType *pPlane0 = (const PixelType *)pPlane0_u8;
            const PixelType *pPlane1 = (const PixelType *)pPlane1_u8;

            pel2Plane[w << 1] = pPlane0[w];
            pel2Plane[(w << 1) + 1] = pPlane1[w];
        }
        pel2Plane_u8 += pel2Pitch;
        for (int w = 0; w < width; w++) {
            PixelType *pel2Plane = (PixelType *)pel2Plane_u8;
            const PixelType *pPlane2 = (const PixelType *)pPlane2_u8;
            const PixelType *pPlane3 = (const PixelType *)pPlane3_u8;

            pel2Plane[w << 1] = pPlane2[w];
            pel2Plane[(w << 1) + 1] = pPlane3[w];
        }
        pel2Plane_u8 += pel2Pitch;
        pPlane0_u8 += pitch;
        pPlane1_u8 += pitch;
        pPlane2_u8 += pitch;
        pPlane3_u8 += pitch;
    }
}


void Merge4PlanesToBig(uint8_t *pel2Plane, int pel2Pitch, const uint8_t *pPlane0, const uint8_t *pPlane1, const uint8_t *pPlane2, const uint8_t *pPlane3, int width, int height, int pitch, int bitsPerSample) {
    if (bitsPerSample == 8)
        RealMerge4PlanesToBig<uint8_t>(pel2Plane, pel2Pitch, pPlane0, pPlane1, pPlane2, pPlane3, width, height, pitch);
    else
        RealMerge4PlanesToBig<uint16_t>(pel2Plane, pel2Pitch, pPlane0, pPlane1, pPlane2, pPlane3, width, height, pitch);
}


// copy refined planes to big one plane
template <typename PixelType>
static void RealMerge16PlanesToBig(uint8_t *pel4Plane_u8, int pel4Pitch,
                            const uint8_t *pPlane0_u8, const uint8_t *pPlane1_u8, const uint8_t *pPlane2_u8, const uint8_t *pPlane3_u8,
                            const uint8_t *pPlane4_u8, const uint8_t *pPlane5_u8, const uint8_t *pPlane6_u8, const uint8_t *pPlane7_u8,
                            const uint8_t *pPlane8_u8, const uint8_t *pPlane9_u8, const uint8_t *pPlane10_u8, const uint8_t *pPlane11_u8,
                            const uint8_t *pPlane12_u8, const uint8_t *pPlane13_u8, const uint8_t *pPlane14_u8, const uint8_t *pPlane15_u8,
                            int width, int height, int pitch) {
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            PixelType *pel4Plane = (PixelType *)pel4Plane_u8;
            const PixelType *pPlane0 = (const PixelType *)pPlane0_u8;
            const PixelType *pPlane1 = (const PixelType *)pPlane1_u8;
            const PixelType *pPlane2 = (const PixelType *)pPlane2_u8;
            const PixelType *pPlane3 = (const PixelType *)pPlane3_u8;

            pel4Plane[w << 2] = pPlane0[w];
            pel4Plane[(w << 2) + 1] = pPlane1[w];
            pel4Plane[(w << 2) + 2] = pPlane2[w];
            pel4Plane[(w << 2) + 3] = pPlane3[w];
        }
        pel4Plane_u8 += pel4Pitch;
        for (int w = 0; w < width; w++) {
            PixelType *pel4Plane = (PixelType *)pel4Plane_u8;
            const PixelType *pPlane4 = (const PixelType *)pPlane4_u8;
            const PixelType *pPlane5 = (const PixelType *)pPlane5_u8;
            const PixelType *pPlane6 = (const PixelType *)pPlane6_u8;
            const PixelType *pPlane7 = (const PixelType *)pPlane7_u8;

            pel4Plane[w << 2] = pPlane4[w];
            pel4Plane[(w << 2) + 1] = pPlane5[w];
            pel4Plane[(w << 2) + 2] = pPlane6[w];
            pel4Plane[(w << 2) + 3] = pPlane7[w];
        }
        pel4Plane_u8 += pel4Pitch;
        for (int w = 0; w < width; w++) {
            PixelType *pel4Plane = (PixelType *)pel4Plane_u8;
            const PixelType *pPlane8 = (const PixelType *)pPlane8_u8;
            const PixelType *pPlane9 = (const PixelType *)pPlane9_u8;
            const PixelType *pPlane10 = (const PixelType *)pPlane10_u8;
            const PixelType *pPlane11 = (const PixelType *)pPlane11_u8;

            pel4Plane[w << 2] = pPlane8[w];
            pel4Plane[(w << 2) + 1] = pPlane9[w];
            pel4Plane[(w << 2) + 2] = pPlane10[w];
            pel4Plane[(w << 2) + 3] = pPlane11[w];
        }
        pel4Plane_u8 += pel4Pitch;
        for (int w = 0; w < width; w++) {
            PixelType *pel4Plane = (PixelType *)pel4Plane_u8;
            const PixelType *pPlane12 = (const PixelType *)pPlane12_u8;
            const PixelType *pPlane13 = (const PixelType *)pPlane13_u8;
            const PixelType *pPlane14 = (const PixelType *)pPlane14_u8;
            const PixelType *pPlane15 = (const PixelType *)pPlane15_u8;

            pel4Plane[w << 2] = pPlane12[w];
            pel4Plane[(w << 2) + 1] = pPlane13[w];
            pel4Plane[(w << 2) + 2] = pPlane14[w];
            pel4Plane[(w << 2) + 3] = pPlane15[w];
        }
        pel4Plane_u8 += pel4Pitch;
        pPlane0_u8 += pitch;
        pPlane1_u8 += pitch;
        pPlane2_u8 += pitch;
        pPlane3_u8 += pitch;
        pPlane4_u8 += pitch;
        pPlane5_u8 += pitch;
        pPlane6_u8 += pitch;
        pPlane7_u8 += pitch;
        pPlane8_u8 += pitch;
        pPlane9_u8 += pitch;
        pPlane10_u8 += pitch;
        pPlane11_u8 += pitch;
        pPlane12_u8 += pitch;
        pPlane13_u8 += pitch;
        pPlane14_u8 += pitch;
        pPlane15_u8 += pitch;
    }
}


void Merge16PlanesToBig(uint8_t *pel4Plane, int pel4Pitch,
                        const uint8_t *pPlane0, const uint8_t *pPlane1, const uint8_t *pPlane2, const uint8_t *pPlane3,
                        const uint8_t *pPlane4, const uint8_t *pPlane5, const uint8_t *pPlane6, const uint8_t *pPlane7,
                        const uint8_t *pPlane8, const uint8_t *pPlane9, const uint8_t *pPlane10, const uint8_t *pPlane11,
                        const uint8_t *pPlane12, const uint8_t *pPlane13, const uint8_t *pPlane14, const uint8_t *pPlane15,
                        int width, int height, int pitch, int bitsPerSample) {
    if (bitsPerSample == 8)
        RealMerge16PlanesToBig<uint8_t>(pel4Plane, pel4Pitch, pPlane0, pPlane1, pPlane2, pPlane3, pPlane4, pPlane5, pPlane6, pPlane7, pPlane8, pPlane9, pPlane10, pPlane11, pPlane12, pPlane13, pPlane14, pPlane15, width, height, pitch);
    else
        RealMerge16PlanesToBig<uint16_t>(pel4Plane, pel4Pitch, pPlane0, pPlane1, pPlane2, pPlane3, pPlane4, pPlane5, pPlane6, pPlane7, pPlane8, pPlane9, pPlane10, pPlane11, pPlane12, pPlane13, pPlane14, pPlane15, width, height, pitch);
}


//-----------------------------------------------------------
uint8_t SADToMask(unsigned int sad, unsigned int sadnorm1024) {
    // sadnorm1024 = 255 * (4*1024)/(mlSAD*nBlkSize*nBlkSize*chromablockfactor)
    unsigned int l = sadnorm1024 * sad / 1024;
    return (uint8_t)((l > 255) ? 255 : l);
}


// time-weihted blend src with ref frames (used for interpolation for poor motion estimation)
template <typename PixelType>
static void RealBlend(uint8_t *pdst, const uint8_t *psrc, const uint8_t *pref, int height, int width, int dst_pitch, int src_pitch, int ref_pitch, int time256) {
    int h, w;
    for (h = 0; h < height; h++) {
        for (w = 0; w < width; w++) {
            const PixelType *psrc_ = (const PixelType *)psrc;
            const PixelType *pref_ = (const PixelType *)pref;
            PixelType *pdst_ = (PixelType *)pdst;

            pdst_[w] = (psrc_[w] * (256 - time256) + pref_[w] * time256) >> 8;
        }
        pdst += dst_pitch;
        psrc += src_pitch;
        pref += ref_pitch;
    }
}


void Blend(uint8_t *pdst, const uint8_t *psrc, const uint8_t *pref, int height, int width, int dst_pitch, int src_pitch, int ref_pitch, int time256, int bitsPerSample) {
    if (bitsPerSample == 8)
        RealBlend<uint8_t>(pdst, psrc, pref, height, width, dst_pitch, src_pitch, ref_pitch, time256);
    else
        RealBlend<uint16_t>(pdst, psrc, pref, height, width, dst_pitch, src_pitch, ref_pitch, time256);
}


template <typename PixelType>
static void FlowInter(
        uint8_t *pdst8, int dst_pitch,
        const uint8_t *prefB8, const uint8_t *prefF8, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel) {

    const PixelType *prefB = (const PixelType *)prefB8;
    const PixelType *prefF = (const PixelType *)prefF8;
    PixelType *pdst = (PixelType *)pdst8;

    ref_pitch /= sizeof(PixelType);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            int vxF = (VXFullF[w] * time256) >> 8;
            int vyF = (VYFullF[w] * time256) >> 8;
            int64_t dstF = prefF[vyF * ref_pitch + vxF + (w << nPelLog)];
            int dstF0 = prefF[(w << nPelLog)]; /* zero */
            int vxB = (VXFullB[w] * (256 - time256)) >> 8;
            int vyB = (VYFullB[w] * (256 - time256)) >> 8;
            int64_t dstB = prefB[vyB * ref_pitch + vxB + (w << nPelLog)];
            int dstB0 = prefB[(w << nPelLog)]; /* zero */
            pdst[w] = (PixelType)((((dstF * (255 - MaskF[w]) + ((MaskF[w] * (dstB * (255 - MaskB[w]) + MaskB[w] * dstF0) + 255) >> 8) + 255) >> 8) * (256 - time256) +
                                 ((dstB * (255 - MaskB[w]) + ((MaskB[w] * (dstF * (255 - MaskF[w]) + MaskF[w] * dstB0) + 255) >> 8) + 255) >> 8) * time256) >>
                      8);
        }
        pdst += dst_pitch;
        prefB += ref_pitch << nPelLog;
        prefF += ref_pitch << nPelLog;
        VXFullB += VPitch;
        VYFullB += VPitch;
        VXFullF += VPitch;
        VYFullF += VPitch;
        MaskB += VPitch;
        MaskF += VPitch;
    }
}


template <typename PixelType>
static void FlowInterExtra(
        uint8_t *pdst8, int dst_pitch,
        const uint8_t *prefB8, const uint8_t *prefF8, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel,
        const int16_t *VXFullBB, const int16_t *VXFullFF,
        const int16_t *VYFullBB, const int16_t *VYFullFF) {

    const PixelType *prefB = (const PixelType *)prefB8;
    const PixelType *prefF = (const PixelType *)prefF8;
    PixelType *pdst = (PixelType *)pdst8;

    ref_pitch /= sizeof(PixelType);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            int vxF = (VXFullF[w] * time256) >> 8;
            int vyF = (VYFullF[w] * time256) >> 8;
            int adrF = vyF * ref_pitch + vxF + (w << nPelLog);
            int dstF = prefF[adrF];

            int vxFF = (VXFullFF[w] * time256) >> 8;
            int vyFF = (VYFullFF[w] * time256) >> 8;
            int adrFF = vyFF * ref_pitch + vxFF + (w << nPelLog);
            int dstFF = prefF[adrFF];

            int vxB = (VXFullB[w] * (256 - time256)) >> 8;
            int vyB = (VYFullB[w] * (256 - time256)) >> 8;
            int adrB = vyB * ref_pitch + vxB + (w << nPelLog);
            int dstB = prefB[adrB];

            int vxBB = (VXFullBB[w] * (256 - time256)) >> 8;
            int vyBB = (VYFullBB[w] * (256 - time256)) >> 8;
            int adrBB = vyBB * ref_pitch + vxBB + (w << nPelLog);
            int dstBB = prefB[adrBB];

            /* use median, firsly get min max of compensations */
            int minfb = min(dstB, dstF);
            int maxfb = max(dstB, dstF);

            int medianBB = max(minfb, min(dstBB, maxfb));
            int medianFF = max(minfb, min(dstFF, maxfb));

            pdst[w] = (((medianBB * MaskF[w] + dstF * (255 - MaskF[w]) + 255) >> 8) * (256 - time256) +
                       ((medianFF * MaskB[w] + dstB * (255 - MaskB[w]) + 255) >> 8) * time256) >>
                      8;
        }
        pdst += dst_pitch;
        prefB += ref_pitch << nPelLog;
        prefF += ref_pitch << nPelLog;
        VXFullB += VPitch;
        VYFullB += VPitch;
        VXFullF += VPitch;
        VYFullF += VPitch;
        MaskB += VPitch;
        MaskF += VPitch;
        VXFullBB += VPitch;
        VYFullBB += VPitch;
        VXFullFF += VPitch;
        VYFullFF += VPitch;
    }
}


template <typename PixelType>
static void FlowInterSimple(
        uint8_t *pdst8, int dst_pitch,
        const uint8_t *prefB8, const uint8_t *prefF8, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel) {

    const PixelType *prefB = (const PixelType *)prefB8;
    const PixelType *prefF = (const PixelType *)prefF8;
    PixelType *pdst = (PixelType *)pdst8;

    ref_pitch /= sizeof(PixelType);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(nPel);

    if (time256 == 128) { /* special case double fps - fastest */
        for (int h = 0; h < height; h++) {
            for (int w = 0; w < width; w++) {
                int vxF = VXFullF[w] >> 1;
                int vyF = VYFullF[w] >> 1;
                int dstF = prefF[vyF * ref_pitch + vxF + (w << nPelLog)];
                int vxB = VXFullB[w] >> 1;
                int vyB = VYFullB[w] >> 1;
                int dstB = prefB[vyB * ref_pitch + vxB + (w << nPelLog)];
                pdst[w] = (((dstF + dstB) << 8) + (dstB - dstF) * (MaskF[w] - MaskB[w])) >> 9;
            }
            pdst += dst_pitch;
            prefB += ref_pitch << nPelLog;
            prefF += ref_pitch << nPelLog;
            VXFullB += VPitch;
            VYFullB += VPitch;
            VXFullF += VPitch;
            VYFullF += VPitch;
            MaskB += VPitch;
            MaskF += VPitch;
        }
    } else { /* general case */
        for (int h = 0; h < height; h++) {
            for (int w = 0; w < width; w++) {
                int vxF = (VXFullF[w] * time256) >> 8;
                int vyF = (VYFullF[w] * time256) >> 8;
                int dstF = prefF[vyF * ref_pitch + vxF + (w << nPelLog)];
                int vxB = (VXFullB[w] * (256 - time256)) >> 8;
                int vyB = (VYFullB[w] * (256 - time256)) >> 8;
                int dstB = prefB[vyB * ref_pitch + vxB + (w << nPelLog)];
                pdst[w] = (((dstF * (255 - MaskF[w]) + dstB * MaskF[w] + 255) >> 8) * (256 - time256) +
                           ((dstB * (255 - MaskB[w]) + dstF * MaskB[w] + 255) >> 8) * time256) >>
                          8;
            }
            pdst += dst_pitch;
            prefB += ref_pitch << nPelLog;
            prefF += ref_pitch << nPelLog;
            VXFullB += VPitch;
            VYFullB += VPitch;
            VXFullF += VPitch;
            VYFullF += VPitch;
            MaskB += VPitch;
            MaskF += VPitch;
        }
    }
}


void selectFlowInterFunctions(FlowInterSimpleFunction *simple, FlowInterFunction *regular, FlowInterExtraFunction *extra, int bitsPerSample, int opt) {
    if (bitsPerSample == 8) {
        *simple = FlowInterSimple<uint8_t>;
        *regular = FlowInter<uint8_t>;
        *extra = FlowInterExtra<uint8_t>;
    } else {
        *simple = FlowInterSimple<uint16_t>;
        *regular = FlowInter<uint16_t>;
        *extra = FlowInterExtra<uint16_t>;
    }

#if defined(MVTOOLS_X86)
    if (opt && (g_cpuinfo & X264_CPU_AVX2))
        selectFlowInterFunctions_AVX2(simple, regular, extra, bitsPerSample);
#endif
}
