/*
 * tapeutil.c - Tape Utilities for USB Host Mode
 * 
 * All file I/O is handled by the host application.
 * This module provides tape operations called via USB commands.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "license.h"
#include "globals.h"
#include "tapeutil.h"
#include "tapedriver.h"
#include "pertbits.h"

/* Module state */
static uint32_t TapePosition = 0;
static uint32_t BytesTransferred = 0;
static uint32_t ErrorCount = 0;

/*
 * TapeUtil_Init - Initialize tape utility module
 */
void TapeUtil_Init(void)
{
    TapePosition = 0;
    BytesTransferred = 0;
    ErrorCount = 0;
    TapeInit();
}

/*
 * TapeUtil_GetPosition - Get current tape position (block count)
 */
uint32_t TapeUtil_GetPosition(void)
{
    if (TapeStatus() & PS1_ILDP)
        TapePosition = 0;
    return TapePosition;
}

/*
 * TapeUtil_GetBytesTransferred - Get total bytes transferred
 */
uint32_t TapeUtil_GetBytesTransferred(void)
{
    return BytesTransferred;
}

/*
 * TapeUtil_GetErrorCount - Get error count
 */
uint32_t TapeUtil_GetErrorCount(void)
{
    return ErrorCount;
}

/*
 * TapeUtil_ResetCounters - Reset transfer statistics
 */
void TapeUtil_ResetCounters(void)
{
    TapePosition = 0;
    BytesTransferred = 0;
    ErrorCount = 0;
}

/*
 * TapeUtil_GetStatus - Get detailed tape status
 * 
 * Returns raw PERTEC status word and fills in decoded status structure
 */
uint16_t TapeUtil_GetStatus(tape_status_t *st)
{
    uint16_t raw = TapeStatus();
    
    if (st)
    {
        st->raw_status = raw;
        st->online     = (raw & PS1_IONL) ? 1 : 0;
        st->ready      = (raw & PS1_IRDY) ? 1 : 0;
        st->loadpoint  = (raw & PS1_ILDP) ? 1 : 0;
        st->eot        = (raw & PS1_EOT)  ? 1 : 0;
        st->protected  = (raw & PS1_IFPT) ? 1 : 0;
        st->rewinding  = (raw & PS1_IRWD) ? 1 : 0;
        st->filemark   = (raw & PS0_IFMK) ? 1 : 0;
        st->hard_error = (raw & PS0_IHER) ? 1 : 0;
        st->soft_error = (raw & PS0_ICER) ? 1 : 0;
        st->data_busy  = (raw & PS0_IDBY) ? 1 : 0;
        st->high_speed = (raw & PS0_ISPEED) ? 1 : 0;
        st->nrzi_mode  = (raw & PS1_INRZ) ? 1 : 0;
        st->position   = TapePosition;
        
        if (st->loadpoint)
            TapePosition = 0;
    }
    
    return raw;
}

/*
 * TapeUtil_Rewind - Rewind tape to BOT
 * 
 * Returns: TSTAT_* status code
 */
uint16_t TapeUtil_Rewind(void)
{
    uint16_t stat;
    
    if (!IsTapeOnline())
        return TSTAT_OFFLINE;
    
    TapePosition = 0;
    stat = TapeRewind();
    return stat;
}

/*
 * TapeUtil_Unload - Unload tape (rewind and go offline)
 * 
 * Returns: TSTAT_* status code
 */
uint16_t TapeUtil_Unload(void)
{
    TapePosition = 0;
    return TapeUnload();
}

/*
 * TapeUtil_ReadBlock - Read one block from tape
 * 
 * Parameters:
 *   buffer    - destination buffer
 *   max_len   - maximum bytes to read
 *   bytes_read - actual bytes read (output)
 *   flags     - error flags (output): bit0=corrected, bit1=hard, bit2=length
 * 
 * Returns: TSTAT_* status code
 */
uint16_t TapeUtil_ReadBlock(uint8_t *buffer, uint16_t max_len, 
                            int *bytes_read, uint8_t *flags)
{
    uint16_t stat;
    int count = 0;
    
    if (flags) *flags = 0;
    if (bytes_read) *bytes_read = 0;
    
    if (!IsTapeOnline())
        return TSTAT_OFFLINE;
    
    stat = TapeRead(buffer, max_len, &count);
    
    if (bytes_read)
        *bytes_read = count;
    
    /* Set error flags */
    if (flags)
    {
        if (stat & TSTAT_CORRERR) *flags |= 0x01;
        if (stat & TSTAT_HARDERR) *flags |= 0x02;
        if (stat & TSTAT_LENGTH)  *flags |= 0x04;
    }
    
    /* Update statistics */
    if (!(stat & TSTAT_TAPEMARK))
        TapePosition++;
    
    if (count > 0)
        BytesTransferred += count;
    
    if (stat & (TSTAT_HARDERR | TSTAT_CORRERR))
        ErrorCount++;
    
    return stat;
}

/*
 * TapeUtil_WriteBlock - Write one block to tape
 * 
 * Parameters:
 *   buffer - data to write
 *   len    - bytes to write (0 = write tapemark)
 * 
 * Returns: TSTAT_* status code
 */
uint16_t TapeUtil_WriteBlock(const uint8_t *buffer, uint16_t len)
{
    uint16_t stat;
    
    if (!IsTapeOnline())
        return TSTAT_OFFLINE;
    
    if (IsTapeProtected())
        return TSTAT_PROTECT;
    
    stat = TapeWrite((uint8_t *)buffer, len);
    
    TapePosition++;
    if (len > 0)
        BytesTransferred += len;
    
    if (stat & TSTAT_HARDERR)
        ErrorCount++;
    
    return stat;
}

/*
 * TapeUtil_WriteFilemark - Write a tape mark
 * 
 * Returns: TSTAT_* status code
 */
uint16_t TapeUtil_WriteFilemark(void)
{
    return TapeUtil_WriteBlock(NULL, 0);
}

/*
 * TapeUtil_SkipBlock - Skip one or more blocks
 * 
 * Parameters:
 *   count - blocks to skip (negative = reverse)
 * 
 * Returns: TSTAT_* status code of last operation
 */
uint16_t TapeUtil_SkipBlock(int16_t count)
{
    uint16_t stat = TSTAT_NOERR;
    int dir = 1;
    
    if (!IsTapeOnline())
        return TSTAT_OFFLINE;
    
    if (count == 0)
        return TSTAT_NOERR;
    
    if (count < 0)
    {
        count = -count;
        dir = -1;
    }
    
    while (count--)
    {
        stat = SkipBlock(dir);
        
        /* Check for load point */
        if (TapeStatus() & PS1_ILDP)
        {
            TapePosition = 0;
            break;
        }
        
        if (stat != TSTAT_NOERR)
            break;
        
        if (dir < 0)
        {
            if (TapePosition > 0)
                TapePosition--;
        }
        else
        {
            TapePosition++;
        }
    }
    
    return stat;
}

/*
 * TapeUtil_SkipFile - Skip one or more files (to next tapemark)
 * 
 * Parameters:
 *   count - files to skip (negative = reverse)
 * 
 * Returns: TSTAT_* status code of last operation
 */
uint16_t TapeUtil_SkipFile(int16_t count)
{
    uint16_t stat = TSTAT_NOERR;
    int dir = 1;
    
    if (!IsTapeOnline())
        return TSTAT_OFFLINE;
    
    if (count == 0)
        return TSTAT_NOERR;
    
    if (count < 0)
    {
        count = -count;
        dir = -1;
    }
    
    while (count--)
    {
        stat = SpaceFile(dir);
        
        if (stat != TSTAT_NOERR)
            break;
    }
    
    /* Position is now relative to filemark */
    TapePosition = 0;
    
    return stat;
}

/*
 * TapeUtil_SetDensity - Set tape density
 * 
 * Parameters:
 *   density - 1 = 1600 PE, 2 = 6250 GCR
 * 
 * Returns: true if successful, false if not at BOT
 */
bool TapeUtil_SetDensity(uint8_t density)
{
    if (!(TapeStatus() & PS1_ILDP))
        return false;  /* Must be at BOT */
    
    if (density == 1)
        Set1600();
    else if (density == 2)
        Set6250();
    else
        return false;
    
    return true;
}

/*
 * TapeUtil_SetAddress - Set tape drive/formatter address
 */
void TapeUtil_SetAddress(uint16_t addr)
{
    SetTapeAddress(addr);
}

/*
 * TapeUtil_IssueCommand - Send raw command (for debugging)
 */
void TapeUtil_IssueCommand(uint16_t cmd)
{
    IssueTapeCommand(cmd);
}

/*
 * TapeUtil_CheckEOV - Check if buffer starts with EOV label
 * 
 * Parameters:
 *   buffer - data buffer to check
 *   len    - buffer length
 * 
 * Returns: true if EOV detected (ASCII or EBCDIC)
 */
bool TapeUtil_CheckEOV(const uint8_t *buffer, uint16_t len)
{
    static const uint8_t eov_ascii[3]  = { 'E', 'O', 'V' };
    static const uint8_t eov_ebcdic[3] = { 0xC5, 0xD6, 0xE5 };
    
    if (len < 3)
        return false;
    
    if (memcmp(buffer, eov_ascii, 3) == 0)
        return true;
    
    if (memcmp(buffer, eov_ebcdic, 3) == 0)
        return true;
    
    return false;
}