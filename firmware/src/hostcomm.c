
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
 *  MCU sends RSP_READY when it needs more data.
 *  Host replies with CMD_FILE_DATA or CMD_FILE_EOF.
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
            /* request next chunk */
            PktSend(RSP_READY, NULL, 0);

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
