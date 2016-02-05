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

#ifndef __MASKFUN__
#define __MASKFUN__

#include <stdint.h>

#include "MVClip.h"
#include "MVFrame.h"

void MakeVectorOcclusionMaskTime(MVClipBalls *mvClip, int nBlkX, int nBlkY, double dMaskNormFactor, double fGamma, int nPel, uint8_t *occMask, int occMaskPitch, int time256, int blkSizeX, int blkSizeY);
void VectorMasksToOcclusionMaskTime(uint8_t *VXMask, uint8_t *VYMask, int nBlkX, int nBlkY, double dMaskNormFactor, double fGamma, int nPel, uint8_t *occMask, int occMaskPitch, int time256, int blkSizeX, int blkSizeY);

void MakeVectorOcclusionMask(MVClipBalls *mvClip, int nBlkX, int nBlkY, double dMaskNormFactor, double fGamma, int nPel, uint8_t *occMask, int occMaskPitch);

void VectorMasksToOcclusionMask(uint8_t *VX, uint8_t *VY, int nBlkX, int nBlkY, double fMaskNormFactor, double fGamma, int nPel, uint8_t *smallMask);

void MakeVectorSmallMasks(MVClipBalls *mvClip, int nX, int nY, uint8_t *VXSmallY, int pitchVXSmallY, uint8_t *VYSmallY, int pitchVYSmallY);
void VectorSmallMaskYToHalfUV(uint8_t *VSmallY, int nBlkX, int nBlkY, uint8_t *VSmallUV, int ratioUV);

void Merge4PlanesToBig(uint8_t *pel2Plane, int pel2Pitch, const uint8_t *pPlane0, const uint8_t *pPlane1,
                       const uint8_t *pPlane2, const uint8_t *pPlane3, int width, int height, int pitch, int bitsPerSample);

void Merge16PlanesToBig(uint8_t *pel4Plane, int pel4Pitch,
                        const uint8_t *pPlane0, const uint8_t *pPlane1, const uint8_t *pPlane2, const uint8_t *pPlane3,
                        const uint8_t *pPlane4, const uint8_t *pPlane5, const uint8_t *pPlane6, const uint8_t *pPlane7,
                        const uint8_t *pPlane8, const uint8_t *pPlane9, const uint8_t *pPlane10, const uint8_t *pPlane11,
                        const uint8_t *pPlane12, const uint8_t *pPlane13, const uint8_t *pPlane14, const uint8_t *pPlane15,
                        int width, int height, int pitch, int bitsPerSample);

uint8_t SADToMask(unsigned int sad, unsigned int sadnorm1024);

void Blend(uint8_t *pdst, const uint8_t *psrc, const uint8_t *pref, int height, int width, int dst_pitch, int src_pitch, int ref_pitch, int time256, bool isse, int bitsPerSample);


// lookup table size 256
void Create_LUTV(int time256, int *LUTVB, int *LUTVF);

void FlowInter(uint8_t *pdst, int dst_pitch, const uint8_t *prefB, const uint8_t *prefF, int ref_pitch,
               uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF, uint8_t *MaskB, uint8_t *MaskF,
               int VPitch, int width, int height, int time256, int nPel, const int *LUTVB, const int *LUTVF, int bitsPerSample);

void FlowInterSimple(uint8_t *pdst, int dst_pitch, const uint8_t *prefB, const uint8_t *prefF, int ref_pitch,
                     uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF, uint8_t *MaskB, uint8_t *MaskF,
                     int VPitch, int width, int height, int time256, int nPel, int *LUTVB, int *LUTVF, int bitsPerSample);

void FlowInterExtra(uint8_t *pdst, int dst_pitch, const uint8_t *prefB, const uint8_t *prefF, int ref_pitch,
                    uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF, uint8_t *MaskB, uint8_t *MaskF,
                    int VPitch, int width, int height, int time256, int nPel, const int *LUTVB, const int *LUTVF,
                    uint8_t *VXFullBB, uint8_t *VXFullFF, uint8_t *VYFullBB, uint8_t *VYFullFF, int bitsPerSample);

void FlowInterPel(uint8_t *pdst, int dst_pitch, MVPlane *prefB, MVPlane *prefF, int ref_pitch,
                  uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF, uint8_t *MaskB, uint8_t *MaskF,
                  int VPitch, int width, int height, int time256, int nPel, int *LUTVB, int *LUTVF);

void FlowInterSimplePel(uint8_t *pdst, int dst_pitch, MVPlane *prefB, MVPlane *prefF, int ref_pitch,
                        uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF, uint8_t *MaskB, uint8_t *MaskF,
                        int VPitch, int width, int height, int time256, int nPel, int *LUTVB, int *LUTVF);

void FlowInterExtraPel(uint8_t *pdst, int dst_pitch, MVPlane *prefB, MVPlane *prefF, int ref_pitch,
                       uint8_t *VXFullB, uint8_t *VXFullF, uint8_t *VYFullB, uint8_t *VYFullF, uint8_t *MaskB, uint8_t *MaskF,
                       int VPitch, int width, int height, int time256, int nPel, int *LUTVB, int *LUTVF,
                       uint8_t *VXFullBB, uint8_t *VXFullFF, uint8_t *VYFullBB, uint8_t *VYFullFF);

#endif
