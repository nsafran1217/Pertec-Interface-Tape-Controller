#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/*  Packet format: [TYPE:1][LEN_LO:1][LEN_HI:1][PAYLOAD:LEN]
 *  Maximum payload = 65535 bytes.
 */

#define PKT_HEADER_SIZE  3

/* --- MCU → Host -------------------------------------------------- */

#define PKT_MSG         0x01   /* Text for display (UTF-8, no null)  */
#define PKT_READY       0x02   /* Controller ready for a command     */
#define PKT_DONE        0x03   /* Command finished                   */
#define PKT_FOPEN       0x04   /* File open: [MODE:1][name...]       */
#define PKT_FWRITE      0x05   /* File write: [data...]              */
#define PKT_FREAD       0x06   /* File read request: [count:4 LE]    */
#define PKT_FCLOSE      0x07   /* File close (no payload)            */
#define PKT_INPUT_REQ   0x08   /* Request line input: [prompt...]    */

/* --- Host → MCU -------------------------------------------------- */

#define PKT_CMD         0x81   /* Command string                     */
#define PKT_FRESULT     0x82   /* File-op result: [code:1] 0=OK      */
#define PKT_FDATA       0x83   /* File read data (0-len = EOF)       */
#define PKT_INPUT       0x84   /* User text input                    */
#define PKT_ESCAPE      0x85   /* Abort current operation            */

/* File-open modes (sent in PKT_FOPEN payload byte 0) */

#define FMODE_READ           0x01
#define FMODE_WRITE_CREATE   0x02

#endif /* PROTOCOL_H */