#include <string.h>
#include "protocol.h"
#include "hostcomm.h"
#include "usbserial.h"

/* ------------------------------------------------------------------ */
/*  Low-level helpers                                                  */
/* ------------------------------------------------------------------ */

static void SendRaw(const uint8_t *d, int n)
{
    USSendBlock(d, n);
}

static void RecvRaw(uint8_t *d, int n)
{
    int got = 0;
    while (got < n)
    {
        int r = USRecvBlock(d + got, n - got);
        got += r;
    }
}

/* ------------------------------------------------------------------ */
/*  Packet send / receive                                              */
/* ------------------------------------------------------------------ */

void SendPacket(uint8_t Type, const uint8_t *Payload, uint16_t Len)
{
    uint8_t hdr[PKT_HEADER_SIZE];
    hdr[0] = Type;
    hdr[1] = (uint8_t)(Len & 0xFF);
    hdr[2] = (uint8_t)(Len >> 8);
    SendRaw(hdr, PKT_HEADER_SIZE);
    if (Len && Payload)
        SendRaw(Payload, Len);
}

int RecvPacket(uint8_t *Type, uint8_t *Payload, uint16_t *Len)
{
    uint8_t hdr[PKT_HEADER_SIZE];
    RecvRaw(hdr, PKT_HEADER_SIZE);
    *Type = hdr[0];
    *Len  = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
    if (*Len && Payload)
        RecvRaw(Payload, *Len);
    return 0;
}

/* Escape flag – set asynchronously when an ESCAPE packet is found
 * during a USB poll cycle.  Cleared by PollEscape(). */

static volatile bool escape_flag = false;

void HostComm_NoteEscape(void)   /* called from USB rx path */
{
    escape_flag = true;
}

bool PollEscape(void)
{
    /* Trigger a USB poll so we process any waiting OUT data. */
    USPoll();
    bool r = escape_flag;
    escape_flag = false;
    return r;
}