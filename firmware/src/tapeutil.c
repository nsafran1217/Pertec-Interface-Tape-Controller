/*  Tape Utilities – host-controlled version.
 *  Low-level tape I/O is still in tapedriver.c.
 *  File I/O is replaced by streaming over USB via hostcomm.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "license.h"
#include "protocol.h"
#include "hostcomm.h"
#include "globals.h"
#include "tapeutil.h"
#include "tapedriver.h"
#include "pertbits.h"
#include "tap.h"
#include "usbserial.h"
#include "dbserial.h"

/* --- local state --- */

static uint32_t LastRecLen, LastRecCnt, BytesCopied, TapePosition;

/* --- forward declarations --- */
static void  AddRecordCount(uint32_t len);
static void  FlushRecordCount(void);

/* --- Status table --- */

typedef struct { uint16_t bit; char *txt; } DSTAT;

static DSTAT DrvStatTbl[] = {
    {PS0_IRP,    "Odd"},
    {PS0_IDBY,   "Data busy"},
    {PS0_ISPEED, "High speed"},
    {PS0_RDAVAIL,"Read full"},
    {PS0_WREMPTY,"Write empty"},
    {PS0_IFMK,   "Tape mark"},
    {PS0_IHER,   "Hard error"},
    {PS0_ICER,   "Soft error"},
    {PS1_INRZ,   "NRZI mode"},
    {PS1_EOT,    "EOT"},
    {PS1_IONL,   "Online"},
    {PS1_IFPT,   "Protected"},
    {PS1_IRWD,   "Rewinding"},
    {PS1_ILDP,   "Load point"},
    {PS1_IRDY,   "Ready"},
    {0, 0}
};

/* ================================================================ */

void HandleStatus(void)
{
    uint16_t s = TapeStatus();

    /* Build a text summary and send via RSP_MSG, then raw via RSP_STATUS */
    char buf[256];
    int pos = 0;
    int i;
    for (i = 0; DrvStatTbl[i].txt; i++) {
        if (s & DrvStatTbl[i].bit) {
            if (pos) buf[pos++] = ' ';
            const char *p = DrvStatTbl[i].txt;
            while (*p && pos < 250) buf[pos++] = *p++;
        }
    }
    buf[pos] = 0;
    if (pos) SendMsg(buf);

    SendStatus(s, TapePosition);
}

void HandleInit(void)
{
    TapeInit();
    SendOK(NULL, 0);
}

void HandleRewind(void)
{
    unsigned int st = TapeRewind();
    TapePosition = 0;
    if (st != TSTAT_NOERR)
        SendError((uint16_t)st, "Rewind failed");
    else
        SendOK(NULL, 0);
}

void HandleUnload(void)
{
    unsigned int st = TapeUnload();
    if (st != TSTAT_NOERR)
        SendError((uint16_t)st, "Unload failed");
    else
        SendOK(NULL, 0);
}

void HandleReadForward(uint8_t flags)
{
    if (!IsTapeOnline()) {
        SendError(ERR_OFFLINE, "Tape drive is offline");
        return;
    }

    int bytesRead;
    unsigned int st = TapeRead(TapeBuffer, TAPE_BUFFER_SIZE, &bytesRead);

    /* Build response: uint16 status + data */
    uint8_t hdr[2];
    hdr[0] = (uint8_t)(st);
    hdr[1] = (uint8_t)(st >> 8);

    /* Send header + data as a single RSP_BLOCK_DATA */
    uint32_t total = 2 + (uint32_t)bytesRead;
    uint8_t phdr[PKT_HDR_SIZE];
    phdr[0] = RSP_BLOCK_DATA;
    phdr[1] = (uint8_t)(total);
    phdr[2] = (uint8_t)(total >> 8);
    phdr[3] = (uint8_t)(total >> 16);
    phdr[4] = (uint8_t)(total >> 24);
    USWriteBlock(phdr, PKT_HDR_SIZE);
    USWriteBlock(hdr, 2);
    if (bytesRead > 0)
        USWriteBlock(TapeBuffer, bytesRead);

    TapePosition++;
}

void HandleSkip(int16_t count)
{
    if (count == 0) { SendOK(NULL, 0); return; }
    if (!IsTapeOnline()) {
        SendError(ERR_OFFLINE, "Tape drive is offline");
        return;
    }

    int dir = (count > 0) ? 1 : -1;
    int n = (count > 0) ? count : -count;
    unsigned int st = TSTAT_NOERR;

    while (n--) {
        st = SkipBlock(dir);
        if (TapeStatus() & PS1_ILDP) { TapePosition = 0; break; }
        if (st != TSTAT_NOERR) break;
        if (dir < 0) { if (TapePosition) TapePosition--; }
        else TapePosition++;
    }

    if (st != TSTAT_NOERR)
        SendError((uint16_t)st, "Skip error");
    else
        SendOK(NULL, 0);
}

void HandleSpace(int16_t count)
{
    if (count == 0) { SendOK(NULL, 0); return; }
    if (!IsTapeOnline()) {
        SendError(ERR_OFFLINE, "Tape drive is offline");
        return;
    }

    int dir = (count > 0) ? 1 : -1;
    int n = (count > 0) ? count : -count;
    unsigned int st = TSTAT_NOERR;

    while (n--) {
        st = SpaceFile(dir);
        if (st != TSTAT_NOERR) break;
    }
    TapePosition = 0;

    if (st != TSTAT_NOERR)
        SendError((uint16_t)st, "Space error");
    else
        SendOK(NULL, 0);
}

void HandleSetAddr(uint8_t addr)
{
    SetTapeAddress((uint16_t)addr);
    SendOK(NULL, 0);
}

void HandleSetRetries(uint8_t count)
{
    TapeRetries = count;
    SendOK(&count, 1);
}

void HandleSetStop(uint8_t mode, uint8_t stopOnError)
{
    StopAfterError = (stopOnError != 0);
    if (mode == 'V' || mode == 'v')
        StopTapemarks = 'V';
    else
        StopTapemarks = mode;
    if (StopTapemarks <= 0 && StopTapemarks != 'V')
        StopTapemarks = 2;
    SendOK(NULL, 0);
}

void HandleDebug(uint16_t cmd)
{
    IssueTapeCommand(cmd);
    SendOK(NULL, 0);
}

void HandleSet1600(void)
{
    if ((TapeStatus() & PS1_ILDP) == 0)
        SendError(ERR_NOT_BOT, "Tape must be at BOT");
    else {
        Set1600();
        SendOK(NULL, 0);
    }
}

void HandleSet6250(void)
{
    if ((TapeStatus() & PS1_ILDP) == 0)
        SendError(ERR_NOT_BOT, "Tape must be at BOT");
    else {
        Set6250();
        SendOK(NULL, 0);
    }
}

/* ================================================================
 * HandleCreateImage – read tape, stream TAP image to host.
 * ================================================================ */

void HandleCreateImage(uint8_t flags)
{
    bool noRewind = (flags & FLAG_NO_REWIND) != 0;
    bool abort = false;
    int fileCount = 0, tapeMarkSeen = 0;
    uint32_t tapeHeader, wc;

    if (!IsTapeOnline()) {
        SendError(ERR_OFFLINE, "Tape is offline");
        return;
    }

    if (!noRewind) TapeRewind();

    /* Signal host that streaming begins */
    SendOK(NULL, 0);

    LastRecCnt = 0;
    TapePosition = 0;
    tapeMarkSeen = 0;
    BytesCopied = 0;
    fileCount = 0;

    StreamReset();

    while (true) {
        unsigned int readStat;
        int readCount;

        if ((abort = CheckAbort()))
            break;

        readStat = TapeRead(TapeBuffer, TAPE_BUFFER_SIZE, &readCount);
        tapeHeader = (uint32_t)readCount;

        if (StopAfterError && (readStat & TSTAT_HARDERR)) {
            SendMsg("Stopping at error");
            break;
        }

        if ((readStat & TSTAT_BLANK) || (readStat & TSTAT_EOT)) {
            SendMsg("Blank/Erased tape or EOT");
            break;
        }

        if (readStat & TSTAT_TAPEMARK) {
            SendMsgF( "Filemark hit at %d", TapePosition);
            tapeMarkSeen++;
            fileCount++;
            readCount = 0;
        } else {
            tapeMarkSeen = 0;
        }

        if (readStat & TSTAT_CORRERR)
            SendMsgF("At block %d, an error was auto-corrected.", TapePosition);

        if (readStat & TSTAT_LENGTH) {
            SendMsgF("Block too long at %d; truncated and flagged.", TapePosition);
            tapeHeader |= TAP_ERROR_FLAG;
        }

        if (readStat & TSTAT_HARDERR) {
            SendMsgF("At block %d, an un-corrected error was hit.", TapePosition);
            tapeHeader |= TAP_ERROR_FLAG;
        }

        TapePosition++;

        /* Write TAP header */
        StreamWrite(&tapeHeader, sizeof(tapeHeader), &wc);

        /* If data block, write data + trailer */
        if (readCount) {
            StreamWrite(TapeBuffer, readCount, &wc);
            StreamWrite(&tapeHeader, sizeof(tapeHeader), &wc);
        }

        AddRecordCount(readCount);

        /* Check for EOV stop */
        if (StopTapemarks == 'V') {
            const uint8_t eovA[3] = {'E','O','V'};
            const uint8_t eovE[3] = {0xc5, 0xd6, 0xe5};
            if (!memcmp(TapeBuffer, eovA, 3) || !memcmp(TapeBuffer, eovE, 3)) {
                SendMsg("Hit EOV");
                break;
            }
        }

        if (tapeMarkSeen == StopTapemarks) {
            fileCount -= (StopTapemarks - 1);
            SendMsgF("%d consecutive tape marks; ending.", StopTapemarks);
            break;
        }
    }

    /* Write EOM */
    tapeHeader = TAP_EOM;
    StreamWrite(&tapeHeader, sizeof(tapeHeader), &wc);

    FlushRecordCount();

    if (!noRewind) {
        TapeRewind();
        TapePosition = 0;
    }

    SendImgDone(TapePosition, (uint32_t)fileCount, BytesCopied, abort ? 1 : 0);
}

/* ================================================================
 * HandleWriteImage – receive TAP image from host, write to tape.
 *
 * Simple pipeline:
 *   1. Read next TAP record from stream into TapeBuffer
 *   2. StreamPrefetch — requests next chunk if running low,
 *      so the host's response arrives via ISR during TapeWrite
 *   3. TapeWrite — ISR fills InQ in background
 *   4. Repeat
 * ================================================================ */

void HandleWriteImage(uint8_t flags)
{
    bool noRewind = (flags & FLAG_NO_REWIND) != 0;
    int fileCount = 0;
    unsigned int tStatus = TSTAT_NOERR;

    if (!IsTapeOnline()) {
        SendError(ERR_OFFLINE, "Tape is offline");
        return;
    }

    if (!noRewind) TapeRewind();

    if (IsTapeProtected()) {
        SendError(ERR_PROTECTED, "Tape is write protected");
        return;
    }

    /* Signal host: ready to receive data */
    SendOK(NULL, 0);

    LastRecCnt = 0;
    TapePosition = 0;
    BytesCopied = 0;
    fileCount = 0;
    tStatus = TSTAT_NOERR;

    StreamReset();

    while (true) {
        DBprintf("%d\n", TapePosition);
        uint32_t h1, h2, br;
        uint32_t bcount;
        int sr;

        /* Read TAP header (4 bytes). */
        sr = StreamRead(&h1, sizeof(h1), &br);
        if (sr != 0 || br != sizeof(h1))
            break;

        if (h1 == TAP_EOM)
            break;

        TapePosition++;

        if (h1 != 0) {
            /* Data block: read data + trailer, validate. */
            bool corrupt = true;
            bcount = h1 & TAP_LENGTH_MASK;

            if (bcount <= TAPE_BUFFER_SIZE) {
                sr = StreamRead(TapeBuffer, bcount, &br);
                if (sr == 0 && br == bcount) {
                    sr = StreamRead(&h2, sizeof(h2), &br);
                    if (sr == 0 && br == sizeof(h2) && h1 == h2)
                        corrupt = false;
                }
            }

            if (corrupt) {
                SendMsgF("Image file corrupt at block %d.", TapePosition);
                break;
            }

            /* Prefetch: if stream buffer is running low, request
               next chunk NOW so it arrives during TapeWrite. */
            StreamPrefetch();
            tStatus = TapeWrite(TapeBuffer, bcount);
        } else {
            /* Tapemark (zero-length). */
            DBprintf("Tapemark at block %d.\n", TapePosition);
            StreamPrefetch();
            tStatus = TapeWrite(TapeBuffer, 0);
            fileCount++;
        }

        AddRecordCount(h1 ? (h1 & TAP_LENGTH_MASK) : 0);

        if (tStatus & TSTAT_CORRERR)
            DBprintf("At block %d, an error was auto-corrected.", TapePosition);
        if (tStatus == TSTAT_HARDERR) {
            SendMsg("Tape write error");
            break;
        }
        
    }

    FlushRecordCount();

    if (!noRewind) {
        TapeRewind();
        TapePosition = 0;
    }

    SendImgDone(TapePosition, (uint32_t)fileCount, BytesCopied, 0);
}

/* ---- local helpers ---- */

static void AddRecordCount(uint32_t len)
{
    BytesCopied += len;
    if (LastRecCnt) {
        if (LastRecLen != len)
            FlushRecordCount();
        LastRecCnt++;
    } else {
        LastRecLen = len;
        LastRecCnt = 1;
    }
}

static void FlushRecordCount(void)
{
    /* In host-controlled mode we just reset counters.
       The host tracks record stats from the data stream. */
    LastRecCnt = 0;
    LastRecLen = 0xffff;
}