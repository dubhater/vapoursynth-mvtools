// DCT abstract class
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

#ifndef __MV_DCT__
#define __MV_DCT__


class DCTClass {


public:
	int sizex;
	int sizey;
	int dctmode;

//	DCTClass(int _sizex, int _sizey, int _dctshift0extra);
//	~DCTClass();
	virtual void DCTBytes2D(const unsigned char *srcp0, int _src_pitch, unsigned char *dctp, int _dct_pitch) = 0;

};

#endif