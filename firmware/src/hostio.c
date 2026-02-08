#include <string.h>
#include "protocol.h"
#include "hostcomm.h"
#include "hostio.h"

/* ------------------------------------------------------------------ */
/*  Write buffer – fire-and-forget to host                             */
/* ------------------------------------------------------------------ */

#define WR_BUF_SZ  8192

static uint8_t wr_buf[WR_BUF_SZ];
static int     wr_pos = 0;

static void flush_wr(void)
{
    if (wr_pos > 0)
    {
        SendPacket(PKT_FWRITE, wr_buf, (uint16_t)wr_pos);
        wr_pos = 0;
    }
}

/* ------------------------------------------------------------------ */
/*  Read buffer – 8 KB prefetch from host                              */
/* ------------------------------------------------------------------ */

#define RD_BUF_SZ  8192

static uint8_t rd_buf[RD_BUF_SZ];
static int     rd_pos = 0;
static int     rd_len = 0;
static bool    rd_eof = false;

static void refill_rd(void)
{
    if (rd_eof) return;

    uint8_t req[4];
    uint32_t want = RD_BUF_SZ;
    req[0] = (uint8_t)(want);
    req[1] = (uint8_t)(want >> 8);
    req[2] = (uint8_t)(want >> 16);
    req[3] = (uint8_t)(want >> 24);
    SendPacket(PKT_FREAD, req, 4);

    uint8_t  type;
    uint16_t len;
    RecvPacket(&type, rd_buf, &len);

    if (type != PKT_FDATA || len == 0)
    {
        rd_eof = true;
        rd_len = 0;
    }
    else
        rd_len = len;

    rd_pos = 0;
}

/* ------------------------------------------------------------------ */
/*  FatFS-compatible API                                               */
/* ------------------------------------------------------------------ */

FRESULT f_open(FIL *fp, const char *path, uint8_t mode)
{
    (void)fp;
    uint8_t  pkt[260];
    uint16_t nlen = (uint16_t)strlen(path);

    pkt[0] = (mode & FA_READ) ? FMODE_READ : FMODE_WRITE_CREATE;
    memcpy(pkt + 1, path, nlen);

    /* Reset buffers */
    wr_pos = 0;
    rd_pos = 0;
    rd_len = 0;
    rd_eof = false;

    SendPacket(PKT_FOPEN, pkt, nlen + 1);

    /* Wait for result */
    uint8_t  rtype;
    uint8_t  rbuf[4];
    uint16_t rlen;
    RecvPacket(&rtype, rbuf, &rlen);

    if (rtype != PKT_FRESULT || rbuf[0] != 0)
        return FR_NO_FILE;

    return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buf, UINT btw, UINT *bw)
{
    (void)fp;
    const uint8_t *src = (const uint8_t *)buf;
    UINT left = btw;

    while (left)
    {
        int space = WR_BUF_SZ - wr_pos;
        int chunk = ((int)left < space) ? (int)left : space;
        memcpy(wr_buf + wr_pos, src, chunk);
        wr_pos += chunk;
        src    += chunk;
        left   -= chunk;
        if (wr_pos >= WR_BUF_SZ)
            flush_wr();
    }
    *bw = btw;
    return FR_OK;
}

FRESULT f_read(FIL *fp, void *buf, UINT btr, UINT *br)
{
    (void)fp;
    uint8_t *dst = (uint8_t *)buf;
    *br = 0;

    while (btr)
    {
        if (rd_pos >= rd_len)
        {
            if (rd_eof) break;
            refill_rd();
            if (rd_eof) break;
        }
        int avail = rd_len - rd_pos;
        int chunk = ((int)btr < avail) ? (int)btr : avail;
        memcpy(dst, rd_buf + rd_pos, chunk);
        rd_pos += chunk;
        dst    += chunk;
        btr    -= chunk;
        *br    += chunk;
    }
    return FR_OK;
}

FRESULT f_close(FIL *fp)
{
    (void)fp;
    flush_wr();                 /* drain any buffered writes */

    SendPacket(PKT_FCLOSE, 0, 0);

    uint8_t  rtype, rbuf[4];
    uint16_t rlen;
    RecvPacket(&rtype, rbuf, &rlen);

    if (rtype != PKT_FRESULT || rbuf[0] != 0)
        return FR_DISK_ERR;

    return FR_OK;
}