/***********************************************************************
 * FILE: protocol.h
 * Binary packet protocol shared between MCU and host.
 * Packet format: [TYPE:1][LEN:4 LE][PAYLOAD:LEN bytes]
 ***********************************************************************/
#ifndef _PROTOCOL_H
#define _PROTOCOL_H

/* --- Commands (Host -> MCU) --- */
#define CMD_PING        0x01
#define CMD_STATUS      0x02
#define CMD_INIT        0x03
#define CMD_REWIND      0x04
#define CMD_UNLOAD      0x05
#define CMD_READ_FWD    0x06  /* read single block; payload: uint8 flags */
#define CMD_SKIP        0x07  /* payload: int16 count (signed) */
#define CMD_SPACE       0x08  /* payload: int16 count (signed) */
#define CMD_SET_ADDR    0x09  /* payload: uint8 addr */
#define CMD_SET_STOP    0x0A  /* payload: uint8 mode, uint8 stop_on_error */
#define CMD_SET_RETRIES 0x0B  /* payload: uint8 count */
#define CMD_DEBUG       0x0C  /* payload: uint16 command */
#define CMD_SET_1600    0x0D
#define CMD_SET_6250    0x0E
#define CMD_CREATE_IMG  0x0F  /* payload: uint8 flags */
#define CMD_WRITE_IMG   0x10  /* payload: uint8 flags */
#define CMD_ABORT       0x11
#define CMD_FILE_DATA   0x12  /* payload: raw data */
#define CMD_FILE_EOF    0x13

/* --- Responses (MCU -> Host) --- */
#define RSP_OK          0x80
#define RSP_ERROR       0x81  /* payload: uint16 code + ascii msg */
#define RSP_STATUS      0x82  /* payload: uint16 raw, uint32 position */
#define RSP_BLOCK_DATA  0x83  /* payload: uint16 tape_status, data[] */
#define RSP_MSG         0x84  /* payload: ascii string */
#define RSP_IMG_DATA    0x85  /* payload: raw TAP-format bytes */
#define RSP_IMG_DONE    0x86  /* payload: uint32 blocks, uint32 files, uint32 bytes, uint8 aborted */
#define RSP_PONG        0x88
#define RSP_READY       0x89  /* MCU ready for next data chunk */

/* --- Error codes (in RSP_ERROR payload) --- */
#define ERR_OFFLINE     0x0001
#define ERR_PROTECTED   0x0002
#define ERR_NOT_BOT     0x0003
#define ERR_INVALID     0x0004
#define ERR_ABORTED     0x0005
#define ERR_HARDERR     0x0006
#define ERR_CORRUPT     0x0007

/* --- Flag bits --- */
#define FLAG_NO_REWIND  0x01
#define FLAG_EBCDIC     0x02

#define PKT_HDR_SIZE    5

#endif /* _PROTOCOL_H */


/***********************************************************************
 * FILE: hostcomm.h
 * Host communication layer - packet send/receive and stream I/O.
 ***********************************************************************/
#ifndef _HOSTCOMM_H
#define _HOSTCOMM_H

#include <stdint.h>
#include <stdbool.h>

void HostCommInit(void);

/* --- Packet I/O --- */
void PktSend(uint8_t type, const uint8_t *data, uint32_t len);
int  PktRecv(uint8_t *type, uint8_t *data, uint32_t maxlen, uint32_t *len);
void PktRecvExact(uint8_t *buf, uint32_t count);

/* --- Convenience senders --- */
void SendOK(const uint8_t *data, uint32_t len);
void SendError(uint16_t code, const char *msg);
void SendMsg(const char *msg);
void SendMsgF(const char *fmt, ...);
void SendStatus(uint16_t raw, uint32_t position);
void SendImgDone(uint32_t blocks, uint32_t files, uint32_t bytes, uint8_t aborted);

/* --- Stream I/O (replaces FatFS f_read / f_write) --- */
void StreamReset(void);
int  StreamWrite(const void *data, uint32_t count, uint32_t *written);
int  StreamRead(void *data, uint32_t count, uint32_t *bytesRead);

/* --- Abort check (replaces CheckForEscape) --- */
bool CheckAbort(void);

#endif /* _HOSTCOMM_H */


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


/***********************************************************************
 * FILE: usbserial.h  (updated – MSC removed)
 ***********************************************************************/
#ifndef _USBSERIAL_H
#define _USBSERIAL_H

#include <stdint.h>
#include <stdbool.h>

int  USInit(void);
void USClear(void);
int  USPutchar(char);
int  USWritechar(char);
int  USGetchar(void);
int  USCharReady(void);
void USPuts(char *What);
int  USWriteBlock(uint8_t *What, int Count);

bool usb_get_connected(void);
void usb_disconnect(void);

#endif


/***********************************************************************
 * FILE: usb.c  (MSC and SD card code removed, CDC-only)
 ***********************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dwc/otg_fs.h>

#include "dbserial.h"
#include "usbserial.h"

#define USB_DATA_EP_IN   0x81
#define USB_DATA_EP_OUT  0x01
#define USB_CDC_EP_IN    0x83

#define USB_PKT  64
#define INQ_SIZE (4096 + 64)   /* input queue – large enough for protocol bursts */

static char  RxBuf[65];
static char  InQ[INQ_SIZE];
static volatile int InQIn, InQOut;

static char usb_serial_number[13];
static usbd_device *udev;
static bool usb_connected = false;

/* ---- descriptors ---- */

static const struct usb_device_descriptor dev_descr = {
    .bLength            = USB_DT_DEVICE_SIZE,
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = USB_CLASS_CDC,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x0483,
    .idProduct          = 0x5740,
    .bcdDevice          = 0x0200,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

static const struct {
    struct usb_cdc_header_descriptor          header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor             acm;
    struct usb_cdc_union_descriptor           cdc_union;
} __attribute__((packed)) cdc_func = {
    .header = {
        .bFunctionLength    = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType    = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC             = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength    = sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType    = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities     = 0,
        .bDataInterface     = 1,
    },
    .acm = {
        .bFunctionLength    = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType    = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities     = 0,
    },
    .cdc_union = {
        .bFunctionLength    = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType    = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface  = 0,
        .bSubordinateInterface0 = 1,
    }
};

static const struct usb_endpoint_descriptor comm_ep[] = {{
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_CDC_EP_IN,
    .bmAttributes     = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize   = 16,
    .bInterval        = 255,
}};

static const struct usb_endpoint_descriptor data_ep[] = {
    {
        .bLength          = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_DATA_EP_OUT,
        .bmAttributes     = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize   = USB_PKT,
        .bInterval        = 1,
    },
    {
        .bLength          = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_DATA_EP_IN,
        .bmAttributes     = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize   = USB_PKT,
        .bInterval        = 1,
    }
};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 1,
    .bInterfaceClass    = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface         = 0,
    .endpoint           = comm_ep,
    .extra              = &cdc_func,
    .extralen           = sizeof(cdc_func),
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 1,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface         = 0,
    .endpoint           = data_ep,
}};

static const struct usb_interface ifaces[] = {
    { .num_altsetting = 1, .altsetting = comm_iface },
    { .num_altsetting = 1, .altsetting = data_iface },
};

static const struct usb_config_descriptor cfg_descr = {
    .bLength             = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType     = USB_DT_CONFIGURATION,
    .wTotalLength        = 0,
    .bNumInterfaces      = 2,
    .bConfigurationValue = 1,
    .iConfiguration      = 0,
    .bmAttributes        = 0x80,
    .bMaxPower           = 0x32,
    .interface           = ifaces,
};

static const char *usb_strings[] = {
    "Tape Controller",
    "",
    usb_serial_number,
};

uint8_t usbd_control_buffer[128];

/* ---- callbacks ---- */

static enum usbd_request_return_codes cdc_ctrl(
    usbd_device *dev, struct usb_setup_data *req,
    uint8_t **buf, uint16_t *len,
    void (**complete)(usbd_device *, struct usb_setup_data *))
{
    (void)complete; (void)dev;
    static const struct usb_cdc_line_coding lc = {
        .dwDTERate   = 115200,
        .bCharFormat = USB_CDC_1_STOP_BITS,
        .bParityType = USB_CDC_NO_PARITY,
        .bDataBits   = 0x08
    };
    switch (req->bRequest) {
        case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
            return USBD_REQ_HANDLED;
        case USB_CDC_REQ_GET_LINE_CODING:
            if (*len < sizeof(lc)) return USBD_REQ_NOTSUPP;
            *buf = (uint8_t *)&lc;
            *len = sizeof(lc);
            return USBD_REQ_HANDLED;
        case USB_CDC_REQ_SET_LINE_CODING:
            if (*len < sizeof(lc)) return USBD_REQ_NOTSUPP;
            return USBD_REQ_HANDLED;
    }
    return USBD_REQ_NOTSUPP;
}

static void cdc_rx(usbd_device *dev, uint8_t ep)
{
    int len, i;
    (void)ep;
    len = usbd_ep_read_packet(dev, USB_DATA_EP_OUT, RxBuf, USB_PKT);
    for (i = 0; i < len; i++) {
        int nxt = InQIn + 1;
        if (nxt >= INQ_SIZE) nxt = 0;
        if (nxt != InQOut) {
            InQ[InQIn] = RxBuf[i];
            InQIn = nxt;
        }
    }
}

static void cdc_setcfg(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;
    if (!usb_connected) usb_connected = true;
    usbd_ep_setup(dev, USB_DATA_EP_OUT, USB_ENDPOINT_ATTR_BULK, USB_PKT, cdc_rx);
    usbd_ep_setup(dev, USB_DATA_EP_IN,  USB_ENDPOINT_ATTR_BULK, USB_PKT, NULL);
    usbd_ep_setup(dev, USB_CDC_EP_IN,   USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
    usbd_register_control_callback(dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE  | USB_REQ_TYPE_RECIPIENT,
        cdc_ctrl);
}

static void cdc_suspend(void)
{
    usb_connected = false;
}

/* ---- public API ---- */

int USInit(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_OTGFS);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

    desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));

    OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
    OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
    OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD;

    udev = usbd_init(&otgfs_usb_driver,
                     &dev_descr, &cfg_descr,
                     usb_strings, 3,
                     usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(udev, cdc_setcfg);
    usbd_register_suspend_callback(udev, cdc_suspend);
    return 0;
}

void otg_fs_isr(void)
{
    if (udev) usbd_poll(udev);
}

bool usb_get_connected(void) { return usb_connected; }
void usb_disconnect(void)    { if (udev) usbd_disconnect(udev, true); }

void USClear(void)
{
    InQIn = 0;
    InQOut = 0;
}

int USGetchar(void)
{
    if (!usb_connected) USInit();
    while (InQIn == InQOut)
        usbd_poll(udev);
    char c = InQ[InQOut++];
    if (InQOut >= INQ_SIZE) InQOut = 0;
    return (unsigned char)c;
}

int USCharReady(void)
{
    if (!usb_connected) USInit();
    usbd_poll(udev);
    return (InQIn != InQOut) ? 1 : 0;
}

int USWritechar(char c)
{
    if (!usb_connected) USInit();
    while (usbd_ep_write_packet(udev, USB_DATA_EP_IN, (uint8_t *)&c, 1) == 0)
        usbd_poll(udev);
    return c;
}

int USPutchar(char c)
{
    return USWritechar(c);
}

void USPuts(char *s)
{
    while (*s) USWritechar(*s++);
}

int USWriteBlock(uint8_t *buf, int count)
{
    if (!usb_connected) USInit();
    while (count > 0) {
        int pass = (count >= USB_PKT) ? USB_PKT : count;
        while (usbd_ep_write_packet(udev, USB_DATA_EP_IN, buf, pass) == 0)
            usbd_poll(udev);
        buf   += pass;
        count -= pass;
    }
    return 0;
}


/***********************************************************************
 * FILE: globals.h  (cleaned up – SD card references removed)
 ***********************************************************************/
#ifndef GLOBAL_H_DEFINED
#define GLOBAL_H_DEFINED 1

#include <stdint.h>
#include <stdbool.h>

#define VERSION "2.0"

#ifndef MAIN
#define SCOPE extern
#else
#define SCOPE
#endif

#define NO_OP {}

SCOPE volatile uint32_t Milliseconds;

#define TAPE_BUFFER_SIZE 65536

SCOPE uint8_t __attribute__((aligned(4)))
    TapeBuffer[TAPE_BUFFER_SIZE];

SCOPE int  TapeRetries;
SCOPE int  StopTapemarks;
SCOPE bool StopAfterError;
SCOPE uint16_t TapeAddress;
SCOPE uint32_t TapePosition;

#undef SCOPE
#endif


/***********************************************************************
 * FILE: tapeutil.h  (updated signatures)
 ***********************************************************************/
#ifndef _TAPEUTIL_INC
#define _TAPEUTIL_INC

#include <stdint.h>

void HandleStatus(void);
void HandleInit(void);
void HandleRewind(void);
void HandleUnload(void);
void HandleReadForward(uint8_t flags);
void HandleSkip(int16_t count);
void HandleSpace(int16_t count);
void HandleSetAddr(uint8_t addr);
void HandleSetRetries(uint8_t count);
void HandleSetStop(uint8_t mode, uint8_t stopOnError);
void HandleDebug(uint16_t cmd);
void HandleCreateImage(uint8_t flags);
void HandleWriteImage(uint8_t flags);
void HandleSet1600(void);
void HandleSet6250(void);

#endif


/***********************************************************************
 * FILE: tapeutil.c  (rewritten for host protocol)
 ***********************************************************************/
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

/* --- local state --- */

static uint32_t LastRecLen, LastRecCnt, BytesCopied;

/* --- forward declarations --- */
static void  AddRecordCount(uint32_t len);
static void  FlushRecordCount(void);

/* --- Status table (unchanged) --- */

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
 * ================================================================ */

void HandleWriteImage(uint8_t flags)
{
    bool noRewind = (flags & FLAG_NO_REWIND) != 0;
    bool abort = false;
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
        uint32_t h1, h2, br;
        int sr;

        sr = StreamRead(&h1, sizeof(h1), &br);
        if (sr != 0 || br != sizeof(h1))
            break;

        if (h1 == TAP_EOM)
            break;

        TapePosition++;

        if (h1 != 0) {
            /* data block */
            bool corrupt = true;
            uint32_t bcount = h1 & TAP_LENGTH_MASK;

            if (bcount < TAPE_BUFFER_SIZE) {
                sr = StreamRead(TapeBuffer, bcount, &br);
                if (sr == 0 && br == bcount) {
                    sr = StreamRead(&h2, sizeof(h2), &br);
                    if (sr == 0 && br == sizeof(h2) && h1 == h2) {
                        corrupt = false;
                        tStatus = TapeWrite(TapeBuffer, bcount);
                    }
                }
            }
            if (corrupt) {
                SendMsgF("Image file corrupt at block %d.", TapePosition);
                break;
            }
        } else {
            /* tapemark */
            tStatus = TapeWrite(TapeBuffer, 0);
            fileCount++;
        }

        AddRecordCount(h1 ? (h1 & TAP_LENGTH_MASK) : 0);

        if (tStatus != TSTAT_NOERR) {
            SendMsg("Tape write error");
            break;
        }

        if (CheckAbort()) { abort = true; break; }
    }

    FlushRecordCount();

    if (!noRewind) {
        TapeRewind();
        TapePosition = 0;
    }

    SendImgDone(TapePosition, (uint32_t)fileCount, BytesCopied, abort ? 1 : 0);
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


/***********************************************************************
 * FILE: main.c  (command dispatcher, no CLI)
 ***********************************************************************/

#define MAIN 1

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "license.h"
#include "protocol.h"
#include "hostcomm.h"
#include "usbserial.h"
#include "gpiodef.h"
#include "globals.h"
#include "miscsubs.h"
#include "tapedriver.h"
#include "tapeutil.h"
#include "dbserial.h"

/* Receive buffer for command payloads (small – max command payload) */
static uint8_t CmdBuf[64];

static void Init(void);
static void ProcessCommands(void);

int main(void)
{
    Init();
    ProcessCommands();   /* never returns */
    return 0;
}

static void Init(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    DBinit();

    Milliseconds = 0;
    InitGPIO();
    SetupSysTick();
    DelaySetup();

    HostCommInit();      /* USB CDC init + protocol layer */

    TapeInit();
    TapePosition = 0;

    DBprintf("Tape controller v" VERSION " ready\n");
}

static void ProcessCommands(void)
{
    uint8_t type;
    uint32_t len;

    while (1) {
        if (PktRecv(&type, CmdBuf, sizeof(CmdBuf), &len) != 0) {
            SendError(ERR_INVALID, "Packet too large");
            continue;
        }

        switch (type) {

        case CMD_PING:
            PktSend(RSP_PONG, NULL, 0);
            break;

        case CMD_STATUS:
            HandleStatus();
            break;

        case CMD_INIT:
            HandleInit();
            break;

        case CMD_REWIND:
            HandleRewind();
            break;

        case CMD_UNLOAD:
            HandleUnload();
            break;

        case CMD_READ_FWD:
            HandleReadForward(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_SKIP: {
            int16_t cnt = 1;
            if (len >= 2)
                cnt = (int16_t)((uint16_t)CmdBuf[0] | ((uint16_t)CmdBuf[1] << 8));
            HandleSkip(cnt);
            break;
        }

        case CMD_SPACE: {
            int16_t cnt = 1;
            if (len >= 2)
                cnt = (int16_t)((uint16_t)CmdBuf[0] | ((uint16_t)CmdBuf[1] << 8));
            HandleSpace(cnt);
            break;
        }

        case CMD_SET_ADDR:
            HandleSetAddr(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_SET_STOP:
            HandleSetStop(
                len >= 1 ? CmdBuf[0] : 2,
                len >= 2 ? CmdBuf[1] : 0);
            break;

        case CMD_SET_RETRIES:
            HandleSetRetries(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_DEBUG: {
            uint16_t cmd = 0;
            if (len >= 2)
                cmd = (uint16_t)CmdBuf[0] | ((uint16_t)CmdBuf[1] << 8);
            HandleDebug(cmd);
            break;
        }

        case CMD_SET_1600:
            HandleSet1600();
            break;

        case CMD_SET_6250:
            HandleSet6250();
            break;

        case CMD_CREATE_IMG:
            HandleCreateImage(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_WRITE_IMG:
            HandleWriteImage(len >= 1 ? CmdBuf[0] : 0);
            break;

        default:
            SendError(ERR_INVALID, "Unknown command");
            break;
        }
    }
}