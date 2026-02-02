/*
 * usb.h - USB CDC Interface with file-like streaming
 * 
 * Provides replacement functions for FatFS f_read/f_write/f_close
 * that stream data over USB to the host application.
 */

#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <stdbool.h>

/* Result codes (compatible with FatFS FRESULT) */
typedef enum {
    USB_OK = 0,
    USB_ERR_IO,
    USB_ERR_TIMEOUT,
    USB_ERR_ABORT,
} USB_Result;

/* Transfer direction */
typedef enum {
    USB_DIR_NONE = 0,
    USB_DIR_READ,       /* Host -> MCU (for WriteImage) */
    USB_DIR_WRITE,      /* MCU -> Host (for CreateImage) */
} USB_Direction;

/* Command codes from host */
#define HOST_CMD_NOP          0x00
#define HOST_CMD_STATUS       0x01
#define HOST_CMD_REWIND       0x02
#define HOST_CMD_UNLOAD       0x03
#define HOST_CMD_SKIP         0x04
#define HOST_CMD_SPACE        0x05
#define HOST_CMD_READ_BLOCK   0x06
#define HOST_CMD_SET_DENSITY  0x07
#define HOST_CMD_CREATE_IMAGE 0x10   /* Tape -> Host (read tape, send TAP) */
#define HOST_CMD_WRITE_IMAGE  0x11   /* Host -> Tape (receive TAP, write tape) */
#define HOST_CMD_ABORT        0xFF

/* Response codes to host */
#define RESP_OK               0x00
#define RESP_ERR              0x01
#define RESP_BUSY             0x02
#define RESP_DATA             0x10   /* Data follows */
#define RESP_DONE             0x20   /* Transfer complete */
#define RESP_NEED_DATA        0x30   /* Ready for more data */

/* Initialize USB subsystem */
void USB_Init(void);

/* Poll USB - call from main loop */
void USB_Poll(void);

/* Check if USB is connected/configured */
bool USB_IsConnected(void);

/* Check for and process host commands - returns command code or 0 */
uint8_t USB_CheckCommand(uint8_t *params, uint16_t *param_len);

/* Send a simple response to host */
void USB_SendResponse(uint8_t code);

/* Send response with data */
void USB_SendResponseData(uint8_t code, const void *data, uint16_t len);

/* Send status/error message to host (replaces Uprintf for status) */
void USB_SendMessage(const char *msg);

/*
 * File-like streaming interface
 * These replace FatFS f_read/f_write/f_close
 */

/* Start a transfer session */
USB_Result USB_OpenTransfer(USB_Direction dir);

/* Write data to host (replaces f_write for CreateImage) */
USB_Result USB_Write(const void *data, uint32_t len, uint32_t *written);

/* Read data from host (replaces f_read for WriteImage) */
USB_Result USB_Read(void *data, uint32_t len, uint32_t *read);

/* End transfer session */
USB_Result USB_CloseTransfer(void);

/* Check if host requested abort (replaces CheckForEscape) */
bool USB_CheckAbort(void);

#endif /* USB_H */