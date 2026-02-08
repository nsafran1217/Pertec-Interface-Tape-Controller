#ifndef _USBSERIAL
#define _USBSERIAL

#include <stdint.h>
#include <stdbool.h>

int  USInit(void);
void USClear(void);

/* Block-level I/O for protocol layer */
void USSendBlock(const uint8_t *Data, int Len);
int  USRecvBlock(uint8_t *Data, int MaxLen);       /* blocking */
int  USRecvAvail(uint8_t *Data, int MaxLen);       /* non-blocking */

/* Non-blocking poll – drives USB stack, handles incoming data */
void USPoll(void);

/* Connection state */
bool usb_get_connected(void);
void usb_disconnect(void);

#endif