#include "DCT.h"

DCTClass::DCTClass(int _sizex, int _sizey, int _dctmode, int _bitsPerSample)
    : sizex(_sizex)
    , sizey(_sizey)
    , dctmode(_dctmode)
    , bitsPerSample(_bitsPerSample) {
}

DCTClass::~DCTClass() {
}
