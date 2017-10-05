
#ifndef __VALIDIMAGE_H
#define __VALIDIMAGE_H

#include "lpclib.h"

/* Check if the currently loaded firmware image is valid.
 * A valid image has the correct signature in the row following the image.
 * A row corresponds to 16 bytes.
 */
bool check4ValidImage (uint32_t startAddress, uint32_t signatureAddress);

/* Mark the loaded image as valid by writing a correct signature. */
void signImage (uint32_t startAddress, uint32_t signatureAddress);

#endif
