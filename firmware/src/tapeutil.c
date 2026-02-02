/*
 * tapeutil.c - Tape Utilities (USB Host Mode)
 * 
 * Modified from original to use USB streaming instead of FatFS.
 * Program flow remains the same - this module assembles/parses TAP files.
 * 
 * Key changes:
 *   f_write() -> USB_Write()
 *   f_read()  -> USB_Read()
 *   f_open()  -> USB_OpenTransfer()
 *   f_close() -> USB_CloseTransfer()
 *   CheckForEscape() -> USB_CheckAbort()
 *   Uprintf() -> removed (host handles display)
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
#include "tap.h"
#include "usb.h"

/* Local variables */
static uint32_t TapePosition;
static uint32_t BytesCopied;

/* Status structure sent to host */
typedef struct __attribute__((packed)) {
    uint16_t raw_status;
    uint32_t position;
    uint32_t bytes_copied;
    uint8_t  online;
    uint8_t  ready;
    uint8_t  loadpoint;
    uint8_t  eot;
    uint8_t  protected;
    uint8_t  rewinding;
} TapeStatusReport;

/*
 * GetTapeStatus - Send tape status to host
 */
void GetTapeStatus(void)
{
    uint16_t stat = TapeStatus();
    TapeStatusReport rpt;
    
    rpt.raw_status = stat;
    rpt.position = TapePosition;
    rpt.bytes_copied = BytesCopied;
    rpt.online = (stat & PS1_IONL) ? 1 : 0;
    rpt.ready = (stat & PS1_IRDY) ? 1 : 0;
    rpt.loadpoint = (stat & PS1_ILDP) ? 1 : 0;
    rpt.eot = (stat & PS1_EOT) ? 1 : 0;
    rpt.protected = (stat & PS1_IFPT) ? 1 : 0;
    rpt.rewinding = (stat & PS1_IRWD) ? 1 : 0;
    
    if (rpt.loadpoint)
        TapePosition = 0;
    
    USB_SendResponseData(RESP_OK, &rpt, sizeof(rpt));
}

/*
 * DoRewind - Rewind tape
 */
void DoRewind(void)
{
    if (!IsTapeOnline()) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    TapePosition = 0;
    TapeRewind();
    USB_SendResponse(RESP_OK);
}

/*
 * DoUnload - Unload tape
 */
void DoUnload(void)
{
    TapeUnload();
    TapePosition = 0;
    USB_SendResponse(RESP_OK);
}

/*
 * DoSkip - Skip blocks forward or backward
 * params[0-1] = signed 16-bit count
 */
void DoSkip(const uint8_t *params)
{
    int16_t count = (int16_t)(params[0] | (params[1] << 8));
    int dir = 1;
    unsigned int status;
    
    if (!IsTapeOnline()) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    if (count == 0) {
        USB_SendResponse(RESP_OK);
        return;
    }
    
    if (count < 0) {
        count = -count;
        dir = -1;
    }
    
    while (count--) {
        status = SkipBlock(dir);
        if (TapeStatus() & PS1_ILDP) {
            TapePosition = 0;
            break;
        }
        if (status != TSTAT_NOERR) break;
        
        if (dir < 0) {
            if (TapePosition) TapePosition--;
        } else {
            TapePosition++;
        }
    }
    
    USB_SendResponse(RESP_OK);
}

/*
 * DoSpace - Space files forward or backward
 * params[0-1] = signed 16-bit count
 */
void DoSpace(const uint8_t *params)
{
    int16_t count = (int16_t)(params[0] | (params[1] << 8));
    int dir = 1;
    unsigned int status;
    
    if (!IsTapeOnline()) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    if (count == 0) {
        USB_SendResponse(RESP_OK);
        return;
    }
    
    if (count < 0) {
        count = -count;
        dir = -1;
    }
    
    while (count--) {
        status = SpaceFile(dir);
        if (status != TSTAT_NOERR) break;
    }
    
    TapePosition = 0;
    USB_SendResponse(RESP_OK);
}

/*
 * DoReadBlock - Read and send a single block
 */
void DoReadBlock(void)
{
    unsigned int status;
    int bytesRead;
    
    if (!IsTapeOnline()) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    status = TapeRead(TapeBuffer, TAPE_BUFFER_SIZE, &bytesRead);
    
    /* Build response: [status:2][length:4][data...] */
    uint8_t hdr[6];
    hdr[0] = status & 0xFF;
    hdr[1] = (status >> 8) & 0xFF;
    hdr[2] = bytesRead & 0xFF;
    hdr[3] = (bytesRead >> 8) & 0xFF;
    hdr[4] = (bytesRead >> 16) & 0xFF;
    hdr[5] = (bytesRead >> 24) & 0xFF;
    
    /* Send header first */
    USB_SendResponseData(RESP_DATA, hdr, sizeof(hdr));
    
    /* Send data if any */
    if (bytesRead > 0) {
        uint32_t written;
        USB_Write(TapeBuffer, bytesRead, &written);
    }
    
    TapePosition++;
}

/*
 * DoSetDensity - Set tape density
 * params[0] = 1 for 1600, 2 for 6250
 */
void DoSetDensity(const uint8_t *params)
{
    if (!(TapeStatus() & PS1_ILDP)) {
        USB_SendResponse(RESP_ERR);  /* Must be at BOT */
        return;
    }
    
    if (params[0] == 1)
        Set1600();
    else if (params[0] == 2)
        Set6250();
    
    USB_SendResponse(RESP_OK);
}

/*
 * DoCreateImage - Read tape and stream TAP image to host
 * 
 * params[0] = stop_tapemarks (1-9, or 'V' for EOV)
 * params[1] = stop_on_error flag
 * params[2] = no_rewind flag
 */
void DoCreateImage(const uint8_t *params)
{
    bool noRewind;
    bool abort;
    int fileCount;
    int tapeMarkSeen;
    uint32_t tapeHeader;
    uint32_t wc;
    uint8_t stopTapemarks;
    bool stopAfterError;
    
    stopTapemarks = params[0];
    stopAfterError = params[1] ? true : false;
    noRewind = params[2] ? true : false;
    
    /* Check if tape is online */
    if (!IsTapeOnline()) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    /* Rewind if requested */
    if (!noRewind)
        TapeRewind();
    
    /* Open transfer - tell host we're sending TAP data */
    if (USB_OpenTransfer(USB_DIR_WRITE) != USB_OK) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    /* Initialize counters */
    TapePosition = 0;
    tapeMarkSeen = 0;
    BytesCopied = 0;
    fileCount = 0;
    abort = false;
    
    /* Main read loop */
    while (true) {
        unsigned int readStat;
        int readCount;
        
        /* Check for abort from host */
        if (USB_CheckAbort()) {
            abort = true;
            break;
        }
        
        /* Read forward */
        readStat = TapeRead(TapeBuffer, TAPE_BUFFER_SIZE, &readCount);
        tapeHeader = readCount;
        
        /* Check for stop conditions */
        if (stopAfterError && (readStat & TSTAT_HARDERR)) {
            break;
        }
        
        if ((readStat & TSTAT_BLANK) || (readStat & TSTAT_EOT)) {
            break;
        }
        
        /* Check for tapemark */
        if (readStat & TSTAT_TAPEMARK) {
            tapeMarkSeen++;
            fileCount++;
            readCount = 0;
        } else {
            tapeMarkSeen = 0;
        }
        
        /* Set error flag if needed */
        if (readStat & TSTAT_LENGTH) {
            tapeHeader |= TAP_ERROR_FLAG;
        }
        
        if (readStat & TSTAT_HARDERR) {
            tapeHeader |= TAP_ERROR_FLAG;
        }
        
        TapePosition++;
        
        /* Write TAP record header */
        USB_Write(&tapeHeader, sizeof(tapeHeader), &wc);
        
        /* Write data and trailer if not a tapemark */
        if (readCount) {
            USB_Write(TapeBuffer, readCount, &wc);
            USB_Write(&tapeHeader, sizeof(tapeHeader), &wc);
        }
        
        BytesCopied += readCount;
        
        /* Check for EOV stop condition */
        if (stopTapemarks == 'V') {
            const uint8_t stopASCII[3] = { 'E', 'O', 'V' };
            const uint8_t stopEBCDIC[3] = { 0xC5, 0xD6, 0xE5 };
            
            if (readCount >= 3) {
                if (!memcmp(TapeBuffer, stopASCII, 3) ||
                    !memcmp(TapeBuffer, stopEBCDIC, 3)) {
                    break;
                }
            }
        }
        
        /* Check for consecutive tapemarks */
        if (tapeMarkSeen == stopTapemarks) {
            fileCount -= (stopTapemarks - 1);
            break;
        }
    }
    
    /* Write EOM marker */
    tapeHeader = TAP_EOM;
    USB_Write(&tapeHeader, sizeof(tapeHeader), &wc);
    
    /* Close transfer */
    USB_CloseTransfer();
    
    /* Rewind if requested */
    if (!noRewind && !abort) {
        TapeRewind();
        TapePosition = 0;
    }
}

/*
 * DoWriteImage - Receive TAP image from host and write to tape
 * 
 * params[0] = no_rewind flag
 */
void DoWriteImage(const uint8_t *params)
{
    bool noRewind;
    bool abort;
    int fileCount;
    unsigned int status;
    
    noRewind = params[0] ? true : false;
    
    /* Check if tape is online */
    if (!IsTapeOnline()) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    /* Check write protection */
    if (IsTapeProtected()) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    /* Rewind if requested */
    if (!noRewind)
        TapeRewind();
    
    /* Open transfer - tell host to send TAP data */
    if (USB_OpenTransfer(USB_DIR_READ) != USB_OK) {
        USB_SendResponse(RESP_ERR);
        return;
    }
    
    /* Initialize */
    abort = false;
    TapePosition = 0;
    BytesCopied = 0;
    fileCount = 0;
    status = TSTAT_NOERR;
    
    /* Main write loop */
    while (true) {
        uint32_t header1;
        uint32_t header2;
        uint32_t bytesRead;
        uint32_t bcount;
        
        /* Read TAP header from host */
        if (USB_Read(&header1, sizeof(header1), &bytesRead) != USB_OK) {
            abort = true;
            break;
        }
        
        if (bytesRead == 0) break;  /* End of data from host */
        
        if (bytesRead != sizeof(header1)) {
            abort = true;
            break;
        }
        
        /* Check for abort */
        if (USB_CheckAbort()) {
            abort = true;
            break;
        }
        
        /* Check for EOM */
        if (header1 == TAP_EOM) break;
        
        TapePosition++;
        
        if (header1 != 0) {
            /* Data block */
            bool corrupt = true;
            
            bcount = header1 & TAP_LENGTH_MASK;
            
            if (bcount <= TAPE_BUFFER_SIZE) {
                /* Read data */
                if (USB_Read(TapeBuffer, bcount, &bytesRead) == USB_OK &&
                    bytesRead == bcount) {
                    /* Read trailer */
                    if (USB_Read(&header2, sizeof(header2), &bytesRead) == USB_OK &&
                        bytesRead == sizeof(header2)) {
                        if (header1 == header2) {
                            corrupt = false;
                            status = TapeWrite(TapeBuffer, bcount);
                        }
                    }
                }
            }
            
            if (corrupt) {
                abort = true;
                break;
            }
        } else {
            /* Tapemark */
            bcount = 0;
            status = TapeWrite(TapeBuffer, 0);
            fileCount++;
        }
        
        BytesCopied += bcount;
        
        if (status != TSTAT_NOERR) break;
    }
    
    /* Close transfer */
    USB_CloseTransfer();
    
    /* Rewind if requested */
    if (!noRewind && !abort) {
        TapeRewind();
        TapePosition = 0;
    }
}