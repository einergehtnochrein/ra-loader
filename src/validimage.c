
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "validimage.h"

/* Check if the currently loaded firmware image is valid.
 * A valid image has the correct signature in the last row of the image.
 * A row corresponds to 16 bytes.
 */
bool check4ValidImage (uint32_t startAddress, uint32_t signatureAddress)
{
    CRC_Handle crc;

    CRC_open(
        CRC_makeMode(CRC_POLY_CRC32,
                        CRC_DATAORDER_REVERSE,
                        CRC_SUMORDER_REVERSE,
                        CRC_DATAPOLARITY_NORMAL,
                        CRC_SUMPOLARITY_INVERSE),
        &crc);

    CRC_seed(crc, 0xFFFFFFFF);
    CRC_write(crc, (void *)startAddress, signatureAddress - startAddress, NULL, NULL);
    uint32_t sig = CRC_read(crc);
    CRC_close(&crc);

    return sig == ((uint32_t *)signatureAddress)[0];
}


/* Mark the loaded image as valid by writing a correct signature. */
void signImage (uint32_t startAddress, uint32_t signatureAddress)
{
    uint32_t *page = (uint32_t *)malloc(256);
    if (page != NULL) {
        memset(page, 0xFF, 256);

        CRC_Handle crc;

        CRC_open(
            CRC_makeMode(CRC_POLY_CRC32,
                        CRC_DATAORDER_REVERSE,
                        CRC_SUMORDER_REVERSE,
                        CRC_DATAPOLARITY_NORMAL,
                        CRC_SUMPOLARITY_INVERSE),
            &crc);

        CRC_seed(crc, 0xFFFFFFFF);
        CRC_write(crc, (void *)startAddress, signatureAddress - startAddress, NULL, NULL);
        page[(signatureAddress % 256) / 4] = CRC_read(crc);
        CRC_close(&crc);

        uint32_t sectorNumber;
        uint32_t pageNumber;
        LPCLIB_Result iapResult;

        IAP_address2SectorNumber(signatureAddress, &sectorNumber, NULL);
        IAP_address2PageNumber(signatureAddress, &pageNumber, NULL);

        iapResult = IAP_prepareSectorsForWriteOperation(
                sectorNumber,
                sectorNumber,
                NULL);
        if (iapResult == LPCLIB_SUCCESS) {
            iapResult = IAP_copyRamToFlash(
                    256 * (signatureAddress / 256),
                    (uint32_t)page,
                    256,
                    NULL);
        }

        free(page);
    }
}
