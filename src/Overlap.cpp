// Overlap copy (really addition)
// Copyright(c)2006 A.G.Balakhnin aka Fizick

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

#include "Overlap.h"


//--------------------------------------------------------------------
// Overlap Windows class
OverlapWindows::OverlapWindows(int _nx, int _ny, int _ox, int _oy)
{
    nx = _nx;
    ny = _ny;
    ox = _ox;
    oy = _oy;
    size = nx*ny;

    //  windows
    fWin1UVx = new float[nx];
    fWin1UVxfirst = new float[nx];
    fWin1UVxlast = new float[nx];
    for (int i=0; i<ox; i++)
    {
        fWin1UVx[i] = cos(M_PI*(i-ox+0.5f)/(ox*2));
        fWin1UVx[i] = fWin1UVx[i]*fWin1UVx[i];// left window (rised cosine)
        fWin1UVxfirst[i] = 1; // very first window
        fWin1UVxlast[i] = fWin1UVx[i]; // very last
    }
    for (int i=ox; i<nx-ox; i++)
    {
        fWin1UVx[i] = 1;
        fWin1UVxfirst[i] = 1; // very first window
        fWin1UVxlast[i] = 1; // very last
    }
    for (int i=nx-ox; i<nx; i++)
    {
        fWin1UVx[i] = cos(M_PI*(i-nx+ox+0.5f)/(ox*2));
        fWin1UVx[i] = fWin1UVx[i]*fWin1UVx[i];// right window (falled cosine)
        fWin1UVxfirst[i] = fWin1UVx[i]; // very first window
        fWin1UVxlast[i] = 1; // very last
    }

    fWin1UVy = new float[ny];
    fWin1UVyfirst = new float[ny];
    fWin1UVylast = new float[ny];
    for (int i=0; i<oy; i++)
    {
        fWin1UVy[i] = cos(M_PI*(i-oy+0.5f)/(oy*2));
        fWin1UVy[i] = fWin1UVy[i]*fWin1UVy[i];// left window (rised cosine)
        fWin1UVyfirst[i] = 1; // very first window
        fWin1UVylast[i] = fWin1UVy[i]; // very last
    }
    for (int i=oy; i<ny-oy; i++)
    {
        fWin1UVy[i] = 1;
        fWin1UVyfirst[i] = 1; // very first window
        fWin1UVylast[i] = 1; // very last
    }
    for (int i=ny-oy; i<ny; i++)
    {
        fWin1UVy[i] = cos(M_PI*(i-ny+oy+0.5f)/(oy*2));
        fWin1UVy[i] = fWin1UVy[i]*fWin1UVy[i];// right window (falled cosine)
        fWin1UVyfirst[i] = fWin1UVy[i]; // very first window
        fWin1UVylast[i] = 1; // very last
    }


    Overlap9Windows = new short[size*9];

    short *winOverUVTL = Overlap9Windows;
    short *winOverUVTM = Overlap9Windows + size;
    short *winOverUVTR = Overlap9Windows + size*2;
    short *winOverUVML = Overlap9Windows + size*3;
    short *winOverUVMM = Overlap9Windows + size*4;
    short *winOverUVMR = Overlap9Windows + size*5;
    short *winOverUVBL = Overlap9Windows + size*6;
    short *winOverUVBM = Overlap9Windows + size*7;
    short *winOverUVBR = Overlap9Windows + size*8;

    for (int j=0; j<ny; j++)
    {
        for (int i=0; i<nx; i++)
        {
            winOverUVTL[i] = (int)(fWin1UVyfirst[j]*fWin1UVxfirst[i]*2048 + 0.5f);
            winOverUVTM[i] = (int)(fWin1UVyfirst[j]*fWin1UVx[i]*2048 + 0.5f);
            winOverUVTR[i] = (int)(fWin1UVyfirst[j]*fWin1UVxlast[i]*2048 + 0.5f);
            winOverUVML[i] = (int)(fWin1UVy[j]*fWin1UVxfirst[i]*2048 + 0.5f);
            winOverUVMM[i] = (int)(fWin1UVy[j]*fWin1UVx[i]*2048 + 0.5f);
            winOverUVMR[i] = (int)(fWin1UVy[j]*fWin1UVxlast[i]*2048 + 0.5f);
            winOverUVBL[i] = (int)(fWin1UVylast[j]*fWin1UVxfirst[i]*2048 + 0.5f);
            winOverUVBM[i] = (int)(fWin1UVylast[j]*fWin1UVx[i]*2048 + 0.5f);
            winOverUVBR[i] = (int)(fWin1UVylast[j]*fWin1UVxlast[i]*2048 + 0.5f);
        }
        winOverUVTL += nx;
        winOverUVTM += nx;
        winOverUVTR += nx;
        winOverUVML += nx;
        winOverUVMM += nx;
        winOverUVMR += nx;
        winOverUVBL += nx;
        winOverUVBM += nx;
        winOverUVBR += nx;
    }
}

OverlapWindows::~OverlapWindows()
{
    delete [] Overlap9Windows;
    delete [] fWin1UVx;
    delete [] fWin1UVxfirst;
    delete [] fWin1UVxlast;
    delete [] fWin1UVy;
    delete [] fWin1UVyfirst;
    delete [] fWin1UVylast;
}

