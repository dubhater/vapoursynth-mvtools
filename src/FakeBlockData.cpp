// Author: Manao
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

#include "FakeBlockData.h"

void FakeBlockData::Init(int _x, int _y)
{
    x=_x;
    y=_y;
}

FakeBlockData::FakeBlockData(int _x, int _y)
{
    x = _x;
    y = _y;
}

FakeBlockData::FakeBlockData()
{
    x = 0;
    y = 0;
}

FakeBlockData::~FakeBlockData()
{

}

void FakeBlockData::Update(const int *array)
{
    vector.x = array[0];
    vector.y = array[1];
    vector.sad = array[2];
}

