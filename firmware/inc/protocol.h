/*
 * protocol.h - Command protocol for STM32 Tape Interface
 * Shared between host and device
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* Protocol version */
#define PROTOCOL_VERSION    0x01

/* Maximum data payload size per transfer */
#define MAX_PAYLOAD_SIZE    4096
#define CMD_HEADER_SIZE     8

/* Command codes */
enum cmd_code {
    CMD_NOP             = 0x00,  /* No operation / ping */
    CMD_GET_INFO        = 0x01,  /* Get device info */
    CMD_GET_STATUS      = 0x02,  /* Get current status */
    CMD_RESET           = 0x03,  /* Reset device state */
    
    /* Configuration */
    CMD_SET_CONFIG      = 0x10,  /* Set configuration */
    CMD_GET_CONFIG      = 0x11,  /* Get configuration */
    
    /* Tape operations */
    CMD_READ_START      = 0x20,  /* Start reading from tape */
    CMD_READ_DATA       = 0x21,  /* Get read data chunk */
    CMD_READ_STOP       = 0x22,  /* Stop reading */
    
    CMD_WRITE_START     = 0x30,  /* Start writing to tape */
    CMD_WRITE_DATA      = 0x31,  /* Send data chunk to write */
    CMD_WRITE_STOP      = 0x32,  /* Stop writing / finalize */
    
    /* Direct GPIO/timing control */
    CMD_SET_GPIO        = 0x40,  /* Set GPIO state */
    CMD_GET_GPIO        = 0x41,  /* Get GPIO state */
    CMD_PULSE           = 0x42,  /* Generate timed pulse */
    
    /* Debug */
    CMD_DEBUG_INFO      = 0xF0,  /* Enable/disable debug logging */
    CMD_RAW_ACCESS      = 0xF1,  /* Raw register access */
};

/* Response/status codes */
enum resp_code {
    RESP_OK             = 0x00,  /* Success */
    RESP_ERR_UNKNOWN    = 0x01,  /* Unknown error */
    RESP_ERR_CMD        = 0x02,  /* Unknown command */
    RESP_ERR_PARAM      = 0x03,  /* Invalid parameter */
    RESP_ERR_STATE      = 0x04,  /* Invalid state for operation */
    RESP_ERR_TIMEOUT    = 0x05,  /* Operation timed out */
    RESP_ERR_OVERFLOW   = 0x06,  /* Buffer overflow */
    RESP_ERR_UNDERFLOW  = 0x07,  /* Buffer underflow */
    RESP_ERR_BUSY       = 0x08,  /* Device busy */
    RESP_ERR_CHECKSUM   = 0x09,  /* Checksum mismatch */
    
    RESP_DATA           = 0x80,  /* Response contains data */
    RESP_MORE           = 0x81,  /* More data available */
    RESP_ASYNC          = 0x82,  /* Async event notification */
};

/* Device states */
enum device_state {
    STATE_IDLE          = 0x00,
    STATE_READING       = 0x01,
    STATE_WRITING       = 0x02,
    STATE_ERROR         = 0xFF,
};

/* Command packet structure (host -> device) */
typedef struct __attribute__((packed)) {
    uint8_t  sync;          /* Sync byte: 0xAA */
    uint8_t  cmd;           /* Command code */
    uint16_t seq;           /* Sequence number */
    uint16_t len;           /* Payload length */
    uint16_t crc;           /* CRC-16 of header + payload */
    uint8_t  payload[];     /* Variable payload */
} cmd_packet_t;

/* Response packet structure (device -> host) */
typedef struct __attribute__((packed)) {
    uint8_t  sync;          /* Sync byte: 0x55 */
    uint8_t  status;        /* Response/status code */
    uint16_t seq;           /* Matching sequence number */
    uint16_t len;           /* Payload length */
    uint16_t crc;           /* CRC-16 of header + payload */
    uint8_t  payload[];     /* Variable payload */
} resp_packet_t;

#define CMD_SYNC_BYTE   0xAA
#define RESP_SYNC_BYTE  0x55

/* Device info structure (returned by CMD_GET_INFO) */
typedef struct __attribute__((packed)) {
    uint8_t  protocol_ver;
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  fw_patch;
    uint32_t hw_id;
    uint32_t capabilities;
    uint16_t max_payload;
    uint16_t buffer_size;
} device_info_t;

/* Configuration structure */
typedef struct __attribute__((packed)) {
    uint32_t read_speed_hz;     /* Read clock/sample rate */
    uint32_t write_speed_hz;    /* Write clock rate */
    uint8_t  read_polarity;     /* Read signal polarity */
    uint8_t  write_polarity;    /* Write signal polarity */
    uint16_t timeout_ms;        /* Operation timeout */
} config_t;

/* Status structure (returned by CMD_GET_STATUS) */
typedef struct __attribute__((packed)) {
    uint8_t  state;
    uint8_t  gpio_state;
    uint16_t buffer_used;
    uint16_t buffer_free;
    uint32_t bytes_transferred;
    uint32_t errors;
} status_t;

/* CRC-16-CCITT calculation */
static inline uint16_t crc16_update(uint16_t crc, uint8_t data) {
    crc = (crc >> 8) | (crc << 8);
    crc ^= data;
    crc ^= (crc & 0xFF) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0xFF) << 5;
    return crc;
}

static inline uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) crc = crc16_update(crc, *data++);
    return crc;
}

#endif /* PROTOCOL_H */