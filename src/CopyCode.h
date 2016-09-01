#ifndef COPYCODE_H
#define COPYCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef void (*COPYFunction)(uint8_t *pDst, intptr_t nDstPitch,
                             const uint8_t *pSrc, intptr_t nSrcPitch);


COPYFunction selectCopyFunction(unsigned width, unsigned height, unsigned bits);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // COPYCODE_H
