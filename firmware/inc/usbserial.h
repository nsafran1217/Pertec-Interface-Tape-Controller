#ifndef _USBSERIAL_H
#define _USBSERIAL_H

#include <stdint.h>
#include <stdbool.h>

int  USInit(void);
void USClear(void);
int  USPutchar(char);
int  USWritechar(char);
int  USGetchar(void);
int  USCharReady(void);
void USPuts(char *What);
int  USWriteBlock(uint8_t *What, int Count);
int  USReadBlock(uint8_t *Buf, int Count);

bool usb_get_connected(void);
void usb_disconnect(void);

#endif