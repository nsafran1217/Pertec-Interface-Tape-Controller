/*
 * tapeutil.h - Tape Utilities for USB Host Mode
 */

#ifndef TAPEUTIL_H
#define TAPEUTIL_H

#include <stdint.h>
#include <stdbool.h>

/* Decoded tape status structure */
typedef struct {
    uint16_t raw_status;    /* Raw PERTEC status word */
    uint8_t  online;
    uint8_t  ready;
    uint8_t  loadpoint;
    uint8_t  eot;
    uint8_t  protected;
    uint8_t  rewinding;
    uint8_t  filemark;
    uint8_t  hard_error;
    uint8_t  soft_error;
    uint8_t  data_busy;
    uint8_t  high_speed;
    uint8_t  nrzi_mode;
    uint32_t position;
} tape_status_t;

/* Read block error flags */
#define READ_FLAG_CORRECTED  0x01
#define READ_FLAG_HARDERROR  0x02
#define READ_FLAG_LENGTH     0x04

/* Initialization */
void TapeUtil_Init(void);

/* Status and counters */
uint16_t TapeUtil_GetStatus(tape_status_t *st);
uint32_t TapeUtil_GetPosition(void);
uint32_t TapeUtil_GetBytesTransferred(void);
uint32_t TapeUtil_GetErrorCount(void);
void     TapeUtil_ResetCounters(void);

/* Tape motion */
uint16_t TapeUtil_Rewind(void);
uint16_t TapeUtil_Unload(void);
uint16_t TapeUtil_SkipBlock(int16_t count);
uint16_t TapeUtil_SkipFile(int16_t count);

/* Data transfer */
uint16_t TapeUtil_ReadBlock(uint8_t *buffer, uint16_t max_len, 
                            int *bytes_read, uint8_t *flags);
uint16_t TapeUtil_WriteBlock(const uint8_t *buffer, uint16_t len);
uint16_t TapeUtil_WriteFilemark(void);

/* Configuration */
bool TapeUtil_SetDensity(uint8_t density);
void TapeUtil_SetAddress(uint16_t addr);

/* Utility */
bool TapeUtil_CheckEOV(const uint8_t *buffer, uint16_t len);
void TapeUtil_IssueCommand(uint16_t cmd);

#endif /* TAPEUTIL_H */