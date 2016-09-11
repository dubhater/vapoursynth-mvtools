// Create an overlay mask with the motion vectors

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

#ifndef MASKFUN_H
#define MASKFUN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "Fakery.h"
#include "MVFrame.h"

void MakeVectorOcclusionMaskTime(const FakeGroupOfPlanes *fgop, int isBackward, int nBlkX, int nBlkY, double dMaskNormDivider, double fGamma, int nPel, uint8_t *occMask, int occMaskPitch, int time256, int nBlkStepX, int nBlkStepY);

void MakeSADMaskTime(const FakeGroupOfPlanes *fgop, int nBlkX, int nBlkY, double dSADNormFactor, double fGamma, int nPel, uint8_t *Mask, int MaskPitch, int time256, int nBlkStepX, int nBlkStepY, int bitsPerSample);

void MakeVectorSmallMasks(const FakeGroupOfPlanes *fgop, int nX, int nY, int16_t *VXSmallY, int pitchVXSmallY, int16_t *VYSmallY, int pitchVYSmallY);
void VectorSmallMaskYToHalfUV(int16_t *VSmallY, int nBlkX, int nBlkY, int16_t *VSmallUV, int ratioUV);

void Merge4PlanesToBig(uint8_t *pel2Plane, int pel2Pitch, const uint8_t *pPlane0, const uint8_t *pPlane1,
                       const uint8_t *pPlane2, const uint8_t *pPlane3, int width, int height, int pitch, int bitsPerSample);

void Merge16PlanesToBig(uint8_t *pel4Plane, int pel4Pitch,
                        const uint8_t *pPlane0, const uint8_t *pPlane1, const uint8_t *pPlane2, const uint8_t *pPlane3,
                        const uint8_t *pPlane4, const uint8_t *pPlane5, const uint8_t *pPlane6, const uint8_t *pPlane7,
                        const uint8_t *pPlane8, const uint8_t *pPlane9, const uint8_t *pPlane10, const uint8_t *pPlane11,
                        const uint8_t *pPlane12, const uint8_t *pPlane13, const uint8_t *pPlane14, const uint8_t *pPlane15,
                        int width, int height, int pitch, int bitsPerSample);

uint8_t SADToMask(unsigned int sad, unsigned int sadnorm1024);

void Blend(uint8_t *pdst, const uint8_t *psrc, const uint8_t *pref, int height, int width, int dst_pitch, int src_pitch, int ref_pitch, int time256, int bitsPerSample);


typedef void (*FlowInterSimpleFunction)(
        uint8_t *pdst, int dst_pitch,
        const uint8_t *prefB, const uint8_t *prefF, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel);

typedef void (*FlowInterFunction)(
        uint8_t *pdst, int dst_pitch,
        const uint8_t *prefB, const uint8_t *prefF, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel);

typedef void (*FlowInterExtraFunction)(
        uint8_t *pdst, int dst_pitch,
        const uint8_t *prefB, const uint8_t *prefF, int ref_pitch,
        const int16_t *VXFullB, const int16_t *VXFullF,
        const int16_t *VYFullB, const int16_t *VYFullF,
        const uint8_t *MaskB, const uint8_t *MaskF, int VPitch,
        int width, int height,
        int time256, int nPel,
        const int16_t *VXFullBB, const int16_t *VXFullFF,
        const int16_t *VYFullBB, const int16_t *VYFullFF);

void selectFlowInterFunctions(FlowInterSimpleFunction *simple, FlowInterFunction *regular, FlowInterExtraFunction *extra, int bitsPerSample, int opt);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
