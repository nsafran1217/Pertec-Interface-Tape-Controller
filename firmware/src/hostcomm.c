
/***********************************************************************
 * FILE: hostcomm.c
 ***********************************************************************/
/*  Host communication implementation.
 *  Wraps USB serial with the binary packet protocol and provides
 *  stream read/write for image transfer operations.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "protocol.h"
#include "hostcomm.h"
#include "usbserial.h"

/* Stream state for image read/write */
static uint32_t sAvail;   /* bytes remaining in current incoming packet */
static bool     sEOF;     /* host signalled end-of-file */

void HostCommInit(void)
{
    USInit();
    USClear();
    StreamReset();
}

/* ---- low-level helpers ---- */

void PktRecvExact(uint8_t *buf, uint32_t count)
{
    while (count--)
        *buf++ = (uint8_t)USGetchar();
}

static void PktSendRaw(const uint8_t *buf, uint32_t count)
{
    if (count)
        USWriteBlock((uint8_t *)buf, (int)count);
}

/* ---- Packet send / receive ---- */

void PktSend(uint8_t type, const uint8_t *data, uint32_t len)
{
    uint8_t hdr[PKT_HDR_SIZE];
    hdr[0] = type;
    hdr[1] = (uint8_t)(len);
    hdr[2] = (uint8_t)(len >> 8);
    hdr[3] = (uint8_t)(len >> 16);
    hdr[4] = (uint8_t)(len >> 24);
    PktSendRaw(hdr, PKT_HDR_SIZE);
    if (len)
        PktSendRaw(data, len);
}

/* Receive a packet.  Returns 0 on success, -1 if payload too large. */
int PktRecv(uint8_t *type, uint8_t *data, uint32_t maxlen, uint32_t *len)
{
    uint8_t hdr[PKT_HDR_SIZE];
    PktRecvExact(hdr, PKT_HDR_SIZE);
    *type = hdr[0];
    *len = (uint32_t)hdr[1]
         | ((uint32_t)hdr[2] << 8)
         | ((uint32_t)hdr[3] << 16)
         | ((uint32_t)hdr[4] << 24);
    if (*len > maxlen)
    {
        /* drain oversized payload */
        uint32_t rem = *len;
        uint8_t tmp;
        while (rem--) PktRecvExact(&tmp, 1);
        return -1;
    }
    if (*len)
        PktRecvExact(data, *len);
    return 0;
}

/* ---- Convenience senders ---- */

void SendOK(const uint8_t *data, uint32_t len)
{
    PktSend(RSP_OK, data, len);
}

void SendError(uint16_t code, const char *msg)
{
    uint8_t buf[258];
    buf[0] = (uint8_t)(code);
    buf[1] = (uint8_t)(code >> 8);
    uint32_t mlen = strlen(msg);
    if (mlen > sizeof(buf) - 2) mlen = sizeof(buf) - 2;
    memcpy(buf + 2, msg, mlen);
    PktSend(RSP_ERROR, buf, 2 + mlen);
}

void SendMsg(const char *msg)
{
    PktSend(RSP_MSG, (const uint8_t *)msg, strlen(msg));
}

/*  SendMsgF – printf-style message sender.
 *  Supports the same format specifiers as the original Uprintf:
 *    %d   - unsigned decimal
 *    %x   - unsigned hex (lowercase)
 *    %c   - single character
 *    %s   - string
 *  Width and leading-zero modifiers work: %04x, %8d, etc.
 */

static void MsgNumout(char *buf, int *pos, int max,
                      unsigned num, int dig, int radix, int bwz)
{
    const char hex[] = "0123456789abcdef";
    char tmp[10];
    int i;

    memset(tmp, 0, sizeof(tmp));
    if (radix == 0 || radix > 16) return;

    i = sizeof(tmp) - 2;
    do {
        tmp[i] = (char)(num % radix);
        num /= radix;
        i--;
    } while (num);

    if (dig == 0)
        dig = sizeof(tmp) - 2 - i;

    for (i = sizeof(tmp) - dig - 1; i < (int)sizeof(tmp) - 1; i++) {
        if (*pos >= max - 1) break;
        if (bwz && !tmp[i] && (i != (int)sizeof(tmp) - 2))
            buf[(*pos)++] = ' ';
        else {
            bwz = 0;
            buf[(*pos)++] = hex[(int)tmp[i]];
        }
    }
}

void SendMsgF(const char *fmt, ...)
{
    char buf[256];
    int pos = 0;
    const int max = sizeof(buf);
    const char *p;
    va_list ap;
    int width, bwz, i;
    unsigned u;
    char *s;

    va_start(ap, fmt);

    for (p = fmt; *p && pos < max - 1; p++) {
        if (*p != '%') {
            buf[pos++] = *p;
            continue;
        }

        ++p;
        width = 0;
        bwz = 1;

        if (*p == '0') { p++; bwz = 0; }

        while (*p >= '0' && *p <= '9')
            width = (width * 10) + (*p++ - '0');

        switch (*p) {
        case 'd':
            u = va_arg(ap, unsigned int);
            MsgNumout(buf, &pos, max, u, width, 10, bwz);
            break;

        case 'x':
            u = va_arg(ap, unsigned int);
            MsgNumout(buf, &pos, max, u, width, 16, bwz);
            break;

        case 'c':
            i = va_arg(ap, int);
            if (pos < max - 1) buf[pos++] = (char)i;
            break;

        case 's':
            s = va_arg(ap, char *);
            if (!width) {
                while (*s && pos < max - 1)
                    buf[pos++] = *s++;
            } else {
                int slen = strlen(s);
                i = slen - width;
                if (i >= 0) {
                    s += i;  /* truncate from left */
                    while (*s && pos < max - 1)
                        buf[pos++] = *s++;
                } else {
                    while (*s && pos < max - 1)
                        buf[pos++] = *s++;
                    for (; i && pos < max - 1; i++)
                        buf[pos++] = ' ';
                }
            }
            break;

        default:
            if (pos < max - 1) buf[pos++] = *p;
            break;
        }
    }

    va_end(ap);
    buf[pos] = 0;

    PktSend(RSP_MSG, (const uint8_t *)buf, pos);
}

void SendStatus(uint16_t raw, uint32_t position)
{
    uint8_t buf[6];
    buf[0] = (uint8_t)(raw);
    buf[1] = (uint8_t)(raw >> 8);
    buf[2] = (uint8_t)(position);
    buf[3] = (uint8_t)(position >> 8);
    buf[4] = (uint8_t)(position >> 16);
    buf[5] = (uint8_t)(position >> 24);
    PktSend(RSP_STATUS, buf, 6);
}

void SendImgDone(uint32_t blocks, uint32_t files, uint32_t bytes, uint8_t aborted)
{
    uint8_t buf[13];
    buf[0]  = (uint8_t)(blocks);
    buf[1]  = (uint8_t)(blocks >> 8);
    buf[2]  = (uint8_t)(blocks >> 16);
    buf[3]  = (uint8_t)(blocks >> 24);
    buf[4]  = (uint8_t)(files);
    buf[5]  = (uint8_t)(files >> 8);
    buf[6]  = (uint8_t)(files >> 16);
    buf[7]  = (uint8_t)(files >> 24);
    buf[8]  = (uint8_t)(bytes);
    buf[9]  = (uint8_t)(bytes >> 8);
    buf[10] = (uint8_t)(bytes >> 16);
    buf[11] = (uint8_t)(bytes >> 24);
    buf[12] = aborted;
    PktSend(RSP_IMG_DONE, buf, 13);
}

/* ---- Stream I/O (virtual file replacement) ---- */

void StreamReset(void)
{
    sAvail = 0;
    sEOF = false;
}

/*  StreamWrite – send image data to host (used during CmdCreateImage).
 *  Data is sent as RSP_IMG_DATA packets.
 *  Returns 0 on success.
 */
int StreamWrite(const void *data, uint32_t count, uint32_t *written)
{
    PktSend(RSP_IMG_DATA, (const uint8_t *)data, count);
    *written = count;
    return 0;
}

/*  StreamRead – receive image data from host (used during CmdWriteImage).
 *  Uses RSP_READY handshake: MCU requests a chunk, host sends one.
 *  Large chunk size (host sends ~16KB at a time) keeps the InQ primed
 *  so the next record is ready as soon as TapeWrite finishes.
 *  The ISR fills InQ in the background during TapeWrite.
 *  Returns: 0 = ok, 1 = EOF, -1 = abort.
 */
int StreamRead(void *data, uint32_t count, uint32_t *bytesRead)
{
    uint8_t *p = (uint8_t *)data;
    uint32_t remaining = count;

    while (remaining > 0 && !sEOF)
    {
        if (sAvail > 0)
        {
            uint32_t chunk = (remaining < sAvail) ? remaining : sAvail;
            PktRecvExact(p, chunk);
            p += chunk;
            sAvail -= chunk;
            remaining -= chunk;
        }
        else
        {
            /* Request next chunk from host. */
            PktSend(RSP_READY, NULL, 0);

            /* Wait for host response. */
            uint8_t hdr[PKT_HDR_SIZE];
            PktRecvExact(hdr, PKT_HDR_SIZE);
            uint8_t type = hdr[0];
            uint32_t plen = (uint32_t)hdr[1]
                          | ((uint32_t)hdr[2] << 8)
                          | ((uint32_t)hdr[3] << 16)
                          | ((uint32_t)hdr[4] << 24);

            if (type == CMD_FILE_EOF)
            {
                sEOF = true;
                break;
            }
            if (type == CMD_ABORT)
            {
                *bytesRead = count - remaining;
                return -1;
            }
            /* CMD_FILE_DATA – payload bytes follow */
            sAvail = plen;
        }
    }
    *bytesRead = count - remaining;
    return sEOF ? 1 : 0;
}

/*  CheckAbort – non-blocking check for CMD_ABORT from host. */
bool CheckAbort(void)
{
    if (!USCharReady())
        return false;

    uint8_t type;
    uint32_t len;
    uint8_t tmp[64];
    if (PktRecv(&type, tmp, sizeof(tmp), &len) == 0)
    {
        if (type == CMD_ABORT)
            return true;
    }
    return false;
}
