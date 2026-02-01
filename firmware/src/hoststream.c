#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "hoststream.h"
#include "usbserial.h"
#include "globals.h"

// Initialize host streaming. Wait briefly for USB CDC connection.
bool HostStreamInit(void)
{
    // If USB is connected already, return true
    if (usb_get_connected())
        return true;

    // Wait up to ~5 seconds for host to connect
    uint32_t start = Milliseconds;
    while (!usb_get_connected()) {
        if ((Milliseconds - start) > 5000)
            break;
    }
    return usb_get_connected();
}

// Write data to host. Try bulk write; fallback to char writes.
void HostStreamWrite(const uint8_t *buf, int len)
{
    if (!buf || len <= 0)
        return;

    // Try bulk write on second CDC interface first
    if (usb_get_connected2() && (USWriteBlock2((uint8_t *)buf, len) >= 0))
        return;
    // Fallback to primary interface
    if (usb_get_connected() && (USWriteBlock((uint8_t *)buf, len) >= 0))
        return;

    // Fallback: send byte-by-byte
    for (int i = 0; i < len; ++i)
        USPutchar(buf[i]);
}

void HostStreamClose(void)
{
    // Nothing special to do; leaving as a placeholder if flush needed.
    return;
}
