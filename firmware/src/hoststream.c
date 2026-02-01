#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "hoststream.h"
#include "usbserial.h"
#include "globals.h"

// Initialize host streaming. Wait briefly for USB CDC connection.
bool HostStreamInit(void)
{
    // Wait for the host to open the streaming CDC (second ACM)
    if (usb_get_connected2())
        return true;

    uint32_t start = Milliseconds;
    while (!usb_get_connected2()) {
        if ((Milliseconds - start) > 5000)
            break;
    }
    return usb_get_connected2();
}

// Write data to host. Try bulk write; fallback to char writes.
void HostStreamWrite(const uint8_t *buf, int len)
{
    if (!buf || len <= 0)
        return;

    // Try bulk write on second CDC interface first
    if (usb_get_connected2() && (USWriteBlock2((uint8_t *)buf, len) >= 0))
        return;

    // Fallback: send byte-by-byte
    for (int i = 0; i < len; ++i)
        USPutchar2(buf[i]);
}

void HostStreamClose(void)
{
    // Nothing special to do; leaving as a placeholder if flush needed.
    return;
}
