/***********************************************************************
 * FILE: protocol.h
 * Binary packet protocol shared between MCU and host.
 * Packet format: [TYPE:1][LEN:4 LE][PAYLOAD:LEN bytes]
 ***********************************************************************/
#ifndef _PROTOCOL_H
#define _PROTOCOL_H

/* --- Commands (Host -> MCU) --- */
#define CMD_PING        0x01
#define CMD_STATUS      0x02
#define CMD_INIT        0x03
#define CMD_REWIND      0x04
#define CMD_UNLOAD      0x05
#define CMD_READ_FWD    0x06  /* read single block; payload: uint8 flags */
#define CMD_SKIP        0x07  /* payload: int16 count (signed) */
#define CMD_SPACE       0x08  /* payload: int16 count (signed) */
#define CMD_SET_ADDR    0x09  /* payload: uint8 addr */
#define CMD_SET_STOP    0x0A  /* payload: uint8 mode, uint8 stop_on_error */
#define CMD_SET_RETRIES 0x0B  /* payload: uint8 count */
#define CMD_DEBUG       0x0C  /* payload: uint16 command */
#define CMD_SET_1600    0x0D
#define CMD_SET_6250    0x0E
#define CMD_CREATE_IMG  0x0F  /* payload: uint8 flags */
#define CMD_WRITE_IMG   0x10  /* payload: uint8 flags */
#define CMD_ABORT       0x11
#define CMD_FILE_DATA   0x12  /* payload: raw data */
#define CMD_FILE_EOF    0x13

/* --- Responses (MCU -> Host) --- */
#define RSP_OK          0x80
#define RSP_ERROR       0x81  /* payload: uint16 code + ascii msg */
#define RSP_STATUS      0x82  /* payload: uint16 raw, uint32 position */
#define RSP_BLOCK_DATA  0x83  /* payload: uint16 tape_status, data[] */
#define RSP_MSG         0x84  /* payload: ascii string */
#define RSP_IMG_DATA    0x85  /* payload: raw TAP-format bytes */
#define RSP_IMG_DONE    0x86  /* payload: uint32 blocks, uint32 files, uint32 bytes, uint8 aborted */
#define RSP_PONG        0x88
#define RSP_READY       0x89  /* MCU ready for next data chunk */

/* --- Error codes (in RSP_ERROR payload) --- */
#define ERR_OFFLINE     0x0001
#define ERR_PROTECTED   0x0002
#define ERR_NOT_BOT     0x0003
#define ERR_INVALID     0x0004
#define ERR_ABORTED     0x0005
#define ERR_HARDERR     0x0006
#define ERR_CORRUPT     0x0007

/* --- Flag bits --- */
#define FLAG_NO_REWIND  0x01
#define FLAG_EBCDIC     0x02

#define PKT_HDR_SIZE    5

#endif /* _PROTOCOL_H */
