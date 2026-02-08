#include <string.h>
#include "protocol.h"
#include "hostcomm.h"
#include "usbserial.h"
#include "globals.h"            /* Milliseconds */

/* ------------------------------------------------------------------ */
/*  Low-level helpers                                                  */
/* ------------------------------------------------------------------ */

static void SendRaw(const uint8_t *d, int n)
{
    USSendBlock(d, n);
}

/* Blocking receive of exactly n bytes */
static void RecvRaw(uint8_t *d, int n)
{
    int got = 0;
    while (got < n)
    {
        int r = USRecvBlock(d + got, n - got);
        got += r;
    }
}

/* Non-blocking receive: tries to collect n bytes within a deadline.
 * Returns the number of bytes actually received. */
static int RecvRawTimeout(uint8_t *d, int n, uint32_t deadline)
{
    int got = 0;
    while (got < n)
    {
        if ((Milliseconds - deadline) < 0x80000000u)
            break;                          /* deadline passed */
        int r = USRecvAvail(d + got, n - got);
        got += r;
    }
    return got;
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

/* Non-blocking receive with timeout.  Returns true if a complete
 * packet was received, false on timeout. */

bool TryRecvPacket(uint8_t *Type, uint8_t *Payload, uint16_t *Len,
                   uint32_t TimeoutMs)
{
    uint32_t deadline = Milliseconds + TimeoutMs;
    uint8_t hdr[PKT_HEADER_SIZE];

    /* Try to get the 3-byte header within the timeout */
    int got = RecvRawTimeout(hdr, PKT_HEADER_SIZE, deadline);
    if (got < PKT_HEADER_SIZE)
        return false;                   /* timed out */

    *Type = hdr[0];
    *Len  = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);

    /* Once we have a valid header, read the payload with a generous
     * deadline — the sender is clearly alive at this point. */
    if (*Len && Payload)
        RecvRaw(Payload, *Len);

    return true;
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