
#ifndef __USB_SERIAL_H
#define __USB_SERIAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define CONSUMER_REPORT_SIZE                1

typedef struct _USBSerial_Context *USBSerial_Handle;

ErrorCode_t USBSerial_init(
        USBD_HANDLE_T hUsb,
        const USB_INTERFACE_DESCRIPTOR *pCifIntfDesc,
        const USB_INTERFACE_DESCRIPTOR *pDifIntfDesc,
        uint32_t *mem_base,
        uint32_t *mem_size);

int USBSerial_read (void *message, int maxLen);
void USBSerial_write (const void *message, int len);
void USBSerial_sendNotification(uint8_t type, uint16_t data);


#ifdef __cplusplus
}
#endif

#endif /* __USB_SERIAL_H */
