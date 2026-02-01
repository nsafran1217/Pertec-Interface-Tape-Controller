/*
 * usb.h - USB CDC Interface for Tape Utility
 */

#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol sync bytes */
#define CMD_SYNC_BYTE   0xAA
#define RESP_SYNC_BYTE  0x55

/* Maximum payload size */
#define USB_MAX_PAYLOAD 4096

/* Command codes */
enum usb_cmd {
    CMD_NOP             = 0x00,
    CMD_GET_INFO        = 0x01,
    CMD_GET_STATUS      = 0x02,
    CMD_RESET           = 0x03,
    
    /* Configuration */
    CMD_SET_CONFIG      = 0x10,
    CMD_GET_CONFIG      = 0x11,
    CMD_SET_DENSITY     = 0x12,
    CMD_SET_ADDRESS     = 0x13,
    
    /* Tape motion */
    CMD_REWIND          = 0x20,
    CMD_UNLOAD          = 0x21,
    CMD_SKIP_BLOCK      = 0x22,
    CMD_SKIP_FILE       = 0x23,
    
    /* Data transfer */
    CMD_READ_BLOCK      = 0x30,
    CMD_WRITE_BLOCK     = 0x31,
    CMD_WRITE_FILEMARK  = 0x32,
    
    /* Streaming mode for image transfer */
    CMD_READ_START      = 0x40,
    CMD_READ_NEXT       = 0x41,
    CMD_READ_STOP       = 0x42,
    CMD_WRITE_START     = 0x43,
    CMD_WRITE_NEXT      = 0x44,
    CMD_WRITE_STOP      = 0x45,

    CMD_DEBUG_INFO      = 0xF0
};

/* Response codes */
enum usb_resp {
    RESP_OK             = 0x00,
    RESP_ERR_UNKNOWN    = 0x01,
    RESP_ERR_CMD        = 0x02,
    RESP_ERR_PARAM      = 0x03,
    RESP_ERR_OFFLINE    = 0x04,
    RESP_ERR_TIMEOUT    = 0x05,
    RESP_ERR_PROTECTED  = 0x06,
    RESP_ERR_CHECKSUM   = 0x07,
    
    /* Tape-specific responses */
    RESP_TAPEMARK       = 0x10,
    RESP_EOT            = 0x11,
    RESP_LOADPOINT      = 0x12,
    RESP_HARDERR        = 0x13,
    RESP_CORRERR        = 0x14,
    RESP_BLANK          = 0x15,
    RESP_LENGTH         = 0x16,
    
    RESP_DATA           = 0x80,
};

/* Command packet header */
typedef struct __attribute__((packed)) {
    uint8_t  sync;
    uint8_t  cmd;
    uint16_t seq;
    uint16_t len;
    uint16_t crc;
} usb_cmd_hdr_t;

/* Response packet header */
typedef struct __attribute__((packed)) {
    uint8_t  sync;
    uint8_t  status;
    uint16_t seq;
    uint16_t len;
    uint16_t crc;
} usb_resp_hdr_t;

/* Device info response */
typedef struct __attribute__((packed)) {
    uint8_t  protocol_ver;
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  fw_patch;
    uint32_t capabilities;
    uint16_t max_payload;
    uint16_t buffer_size;
} usb_device_info_t;

/* Tape status response - mirrors your PERTEC status bits */
typedef struct __attribute__((packed)) {
    uint16_t raw_status;      /* Raw PERTEC status word */
    uint8_t  online;
    uint8_t  ready;
    uint8_t  loadpoint;
    uint8_t  eot;
    uint8_t  protected;
    uint8_t  rewinding;
    uint8_t  filemark;
    uint8_t  error;
    uint32_t position;
} usb_tape_status_t;

/* Configuration structure */
typedef struct __attribute__((packed)) {
    uint8_t  retries;
    uint8_t  stop_tapemarks;
    uint8_t  stop_on_error;
    uint8_t  density;         /* 0=auto, 1=1600, 2=6250 */
    uint16_t tape_address;
    uint16_t timeout_ms;
} usb_config_t;

/* Read block response header */
typedef struct __attribute__((packed)) {
    uint32_t length;          /* Actual bytes read */
    uint8_t  flags;           /* Error flags: bit0=corrected, bit1=hard error */
} usb_read_hdr_t;

/* Initialize USB CDC */
void USB_Init(void);

/* Poll USB - call from main loop */
void USB_Poll(void);

/* Check if USB is connected */
bool USB_IsConnected(void);

/* Process incoming commands - returns true if command was processed */
bool USB_ProcessCommands(void);

/* CRC-16-CCITT */
uint16_t USB_CRC16(const uint8_t *data, uint16_t len);

#endif /* USB_H */