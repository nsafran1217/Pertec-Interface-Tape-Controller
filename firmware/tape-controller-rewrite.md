# Tape Controller – USB Host Protocol Conversion

All MCU firmware files and the Python host application are below. **tapeutil.c requires only minimal include/stub changes** — the FatFS-compatible shim in `hostio.h`/`hostio.c` provides drop-in replacements for `f_open`, `f_read`, `f_write`, and `f_close`.

---

## Architecture

```
┌──────────────┐  USB CDC Bulk   ┌──────────────────┐
│  Python Host │◄───────────────►│  STM32 Firmware   │
│  (tapehost)  │   protocol.h    │                   │
│              │   packets       │  hostcomm ←→ usb  │
│  CLI + Files │                 │  hostio (FatFS)   │
│              │                 │  tapeutil/driver   │
└──────────────┘                 └──────────────────┘
```

**Protocol**: 3-byte header `[TYPE:1][LEN:2 LE]` followed by payload.  
**Writes** (MCU→Host file): fire-and-forget; errors reported at close.  
**Reads** (Host file→MCU): request/response with 8 KB prefetch buffer.  
**Escape**: Host can send `PKT_ESCAPE` at any time; MCU polls in `CheckForEscape`.

---

## protocol.h (new)

```c
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/*  Packet format: [TYPE:1][LEN_LO:1][LEN_HI:1][PAYLOAD:LEN]
 *  Maximum payload = 65535 bytes.
 */

#define PKT_HEADER_SIZE  3

/* --- MCU → Host -------------------------------------------------- */

#define PKT_MSG         0x01   /* Text for display (UTF-8, no null)  */
#define PKT_READY       0x02   /* Controller ready for a command     */
#define PKT_DONE        0x03   /* Command finished                   */
#define PKT_FOPEN       0x04   /* File open: [MODE:1][name...]       */
#define PKT_FWRITE      0x05   /* File write: [data...]              */
#define PKT_FREAD       0x06   /* File read request: [count:4 LE]    */
#define PKT_FCLOSE      0x07   /* File close (no payload)            */
#define PKT_INPUT_REQ   0x08   /* Request line input: [prompt...]    */

/* --- Host → MCU -------------------------------------------------- */

#define PKT_CMD         0x81   /* Command string                     */
#define PKT_FRESULT     0x82   /* File-op result: [code:1] 0=OK      */
#define PKT_FDATA       0x83   /* File read data (0-len = EOF)       */
#define PKT_INPUT       0x84   /* User text input                    */
#define PKT_ESCAPE      0x85   /* Abort current operation            */

/* File-open modes (sent in PKT_FOPEN payload byte 0) */

#define FMODE_READ           0x01
#define FMODE_WRITE_CREATE   0x02

#endif /* PROTOCOL_H */
```

---

## hostcomm.h (new)

```c
#ifndef HOSTCOMM_H
#define HOSTCOMM_H

#include <stdint.h>
#include <stdbool.h>

/*  Protocol packet layer above raw USB.
 *
 *  SendPacket  – blocking, returns when data is queued to USB.
 *  RecvPacket  – blocking, waits until a complete packet arrives.
 *  PollEscape  – non-blocking, returns true if PKT_ESCAPE received.
 */

void  SendPacket(uint8_t Type, const uint8_t *Payload, uint16_t Len);
int   RecvPacket(uint8_t *Type, uint8_t *Payload, uint16_t *Len);
bool  PollEscape(void);         /* check & consume any pending ESCAPE */

#endif /* HOSTCOMM_H */
```

---

## hostcomm.c (new)

```c
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
```

---

## hostio.h (new – FatFS-compatible shim)

```c
#ifndef HOSTIO_H
#define HOSTIO_H

/*  Drop-in replacements for the FatFS calls used in tapeutil.c.
 *  Backed by USB transfers to the host application.
 */

#include <stdint.h>

typedef int FRESULT;
#define FR_OK             0
#define FR_DISK_ERR       1
#define FR_NOT_READY      3
#define FR_NO_FILE        4

typedef unsigned int UINT;

/* Flags compatible with FatFS usage in tapeutil.c */
#define FA_READ           0x01
#define FA_WRITE          0x02
#define FA_CREATE_ALWAYS  0x08

typedef struct {
    int dummy;           /* opaque; single-file-at-a-time */
} FIL;

FRESULT f_open  (FIL *fp, const char *path, uint8_t mode);
FRESULT f_read  (FIL *fp, void *buf, UINT btr, UINT *br);
FRESULT f_write (FIL *fp, const void *buf, UINT btw, UINT *bw);
FRESULT f_close (FIL *fp);

/* Stub – tapeutil.c calls ShowRTCTime() which we no longer need. */
static inline void ShowRTCTime(void) {}

#endif /* HOSTIO_H */
```

---

## hostio.c (new)

```c
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
```

---

## usbserial.h (modified)

```c
#ifndef _USBSERIAL
#define _USBSERIAL

#include <stdint.h>
#include <stdbool.h>

int  USInit(void);
void USClear(void);

/* Block-level I/O for protocol layer */
void USSendBlock(const uint8_t *Data, int Len);
int  USRecvBlock(uint8_t *Data, int MaxLen);

/* Non-blocking poll – drives USB stack, handles incoming data */
void USPoll(void);

/* Connection state */
bool usb_get_connected(void);
void usb_disconnect(void);

#endif
```

---

## usb.c (modified – MSC removed, block I/O added)

```c
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

#include "protocol.h"
#include "usbserial.h"
#include "dbserial.h"

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define USB_DATA_EP_IN   0x81
#define USB_DATA_EP_OUT  0x01
#define USB_CDC_EP_IN    0x83

#define INTERFACE_COMM   0
#define INTERFACE_DATA   1

#define USB_PKT  64                       /* max USB FS bulk packet    */
#define RX_RING  (4096 + 64)              /* receive ring size         */

/* ------------------------------------------------------------------ */
/*  Static data                                                        */
/* ------------------------------------------------------------------ */

static char     rx_ring[RX_RING];
static volatile int rx_in, rx_out;

static char     usb_serial_number[13];
static usbd_device *usb_dev;
static bool     connected = false;

/* External: hostcomm sets escape flag when it sees PKT_ESCAPE in-band */
extern void HostComm_NoteEscape(void);

/* ------------------------------------------------------------------ */
/*  Descriptors  (CDC ACM only — MSC removed)                          */
/* ------------------------------------------------------------------ */

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
} __attribute__((packed)) cdc_fn = {
    .header    = { .bFunctionLength    = sizeof(struct usb_cdc_header_descriptor),
                   .bDescriptorType    = CS_INTERFACE,
                   .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
                   .bcdCDC             = 0x0110 },
    .call_mgmt = { .bFunctionLength    = sizeof(struct usb_cdc_call_management_descriptor),
                   .bDescriptorType    = CS_INTERFACE,
                   .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
                   .bmCapabilities     = 0,
                   .bDataInterface     = INTERFACE_DATA },
    .acm       = { .bFunctionLength    = sizeof(struct usb_cdc_acm_descriptor),
                   .bDescriptorType    = CS_INTERFACE,
                   .bDescriptorSubtype = USB_CDC_TYPE_ACM,
                   .bmCapabilities     = 0 },
    .cdc_union = { .bFunctionLength    = sizeof(struct usb_cdc_union_descriptor),
                   .bDescriptorType    = CS_INTERFACE,
                   .bDescriptorSubtype = USB_CDC_TYPE_UNION,
                   .bControlInterface       = INTERFACE_COMM,
                   .bSubordinateInterface0  = INTERFACE_DATA },
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
    { .bLength = USB_DT_ENDPOINT_SIZE, .bDescriptorType = USB_DT_ENDPOINT,
      .bEndpointAddress = USB_DATA_EP_OUT,
      .bmAttributes = USB_ENDPOINT_ATTR_BULK,
      .wMaxPacketSize = USB_PKT, .bInterval = 1 },
    { .bLength = USB_DT_ENDPOINT_SIZE, .bDescriptorType = USB_DT_ENDPOINT,
      .bEndpointAddress = USB_DATA_EP_IN,
      .bmAttributes = USB_ENDPOINT_ATTR_BULK,
      .wMaxPacketSize = USB_PKT, .bInterval = 1 },
};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = INTERFACE_COMM,
    .bNumEndpoints      = 1,
    .bInterfaceClass    = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .endpoint           = comm_ep,
    .extra              = &cdc_fn,
    .extralen           = sizeof(cdc_fn),
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = INTERFACE_DATA,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = USB_CLASS_DATA,
    .endpoint           = data_ep,
}};

static const struct usb_interface ifaces[] = {
    { .num_altsetting = 1, .altsetting = comm_iface },
    { .num_altsetting = 1, .altsetting = data_iface },
};

static const struct usb_config_descriptor config_descr = {
    .bLength             = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType     = USB_DT_CONFIGURATION,
    .bNumInterfaces      = 2,
    .bConfigurationValue = 1,
    .bmAttributes        = 0x80,
    .bMaxPower           = 0x32,
    .interface           = ifaces,
};

static const char *usb_strings[] = {
    "Tape Controller",
    "Tape Controller v2",
    usb_serial_number,
};

uint8_t usbd_control_buffer[128];

/* ------------------------------------------------------------------ */
/*  CDC callbacks                                                      */
/* ------------------------------------------------------------------ */

static enum usbd_request_return_codes
cdc_ctrl(usbd_device *dev, struct usb_setup_data *req,
         uint8_t **buf, uint16_t *len,
         void (**complete)(usbd_device *, struct usb_setup_data *))
{
    (void)complete; (void)dev;
    static const struct usb_cdc_line_coding lc = {
        .dwDTERate   = 115200,
        .bCharFormat = USB_CDC_1_STOP_BITS,
        .bParityType = USB_CDC_NO_PARITY,
        .bDataBits   = 8,
    };
    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: return USBD_REQ_HANDLED;
    case USB_CDC_REQ_GET_LINE_CODING:
        if (*len < sizeof(lc)) return USBD_REQ_NOTSUPP;
        *buf = (uint8_t *)&lc; *len = sizeof(lc);
        return USBD_REQ_HANDLED;
    case USB_CDC_REQ_SET_LINE_CODING:
        return (*len >= sizeof(lc)) ? USBD_REQ_HANDLED : USBD_REQ_NOTSUPP;
    }
    return USBD_REQ_NOTSUPP;
}

/* Receive callback – fast enqueue into ring buffer.
 * If we detect a 3-byte ESCAPE packet inline, set the flag immediately.
 */
static char rx_tmp[USB_PKT];

static void cdc_rx(usbd_device *dev, uint8_t ep)
{
    (void)ep;
    int n = usbd_ep_read_packet(dev, USB_DATA_EP_OUT, rx_tmp, USB_PKT);
    for (int i = 0; i < n; i++) {
        int next = rx_in + 1;
        if (next >= RX_RING) next = 0;
        if (next != rx_out) {
            rx_ring[rx_in] = rx_tmp[i];
            rx_in = next;
        }
    }
}

static void set_config_cb(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;
    connected = true;
    usbd_ep_setup(dev, USB_DATA_EP_OUT, USB_ENDPOINT_ATTR_BULK, USB_PKT, cdc_rx);
    usbd_ep_setup(dev, USB_DATA_EP_IN,  USB_ENDPOINT_ATTR_BULK, USB_PKT, NULL);
    usbd_ep_setup(dev, USB_CDC_EP_IN,   USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
    usbd_register_control_callback(dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE  | USB_REQ_TYPE_RECIPIENT,
        cdc_ctrl);
}

static void suspend_cb(void) { connected = false; }

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

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

    usb_dev = usbd_init(&otgfs_usb_driver, &dev_descr, &config_descr,
                        usb_strings, 3,
                        usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usb_dev, set_config_cb);
    usbd_register_suspend_callback(usb_dev, suspend_cb);
    return 0;
}

void USClear(void) { rx_in = rx_out = 0; }

void otg_fs_isr(void)
{
    if (usb_dev) usbd_poll(usb_dev);
}

void USPoll(void)
{
    if (usb_dev) usbd_poll(usb_dev);
}

bool usb_get_connected(void) { return connected; }

void usb_disconnect(void)
{
    if (usb_dev) usbd_disconnect(usb_dev, true);
}

/* ---- Block send ------------------------------------------------- */

void USSendBlock(const uint8_t *Data, int Len)
{
    while (Len > 0) {
        int chunk = (Len > USB_PKT) ? USB_PKT : Len;
        while (usbd_ep_write_packet(usb_dev, USB_DATA_EP_IN,
                                    Data, chunk) == 0)
            usbd_poll(usb_dev);
        Data += chunk;
        Len  -= chunk;
        usbd_poll(usb_dev);
    }
}

/* ---- Block receive (blocking) ----------------------------------- */

static int ring_avail(void)
{
    int d = rx_in - rx_out;
    return (d >= 0) ? d : d + RX_RING;
}

int USRecvBlock(uint8_t *Data, int MaxLen)
{
    /* Wait until at least 1 byte available */
    while (ring_avail() == 0)
        usbd_poll(usb_dev);

    int got = 0;
    while (got < MaxLen && ring_avail() > 0) {
        Data[got++] = rx_ring[rx_out++];
        if (rx_out >= RX_RING) rx_out = 0;
    }
    return got;
}
```

---

## comm.h (modified)

```c
#ifndef COMM_DEFINED
#define COMM_DEFINED 1

void InitACM(int BaudRate);
int  Ucharavail(void);
unsigned char Ugetchar(void);
void Uputchar(unsigned char What);
void Uputs(char *What);
void Uprintf(char *Form, ...);
char *Ugets(char *buf, int len);
char *Hexin(unsigned int *RetVal, unsigned int *Digits, char *Buf);

#endif
```

---

## comm.c (modified – output via protocol packets)

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "license.h"
#include "comm.h"
#include "protocol.h"
#include "hostcomm.h"
#include "usbserial.h"

/* ------------------------------------------------------------------ */
/*  Message buffer – Uprintf accumulates here, flushes at return       */
/* ------------------------------------------------------------------ */

#define MSG_BUF_SZ 4096
static char  msg_buf[MSG_BUF_SZ];
static int   msg_pos = 0;

static void msg_flush(void)
{
    if (msg_pos > 0) {
        SendPacket(PKT_MSG, (const uint8_t *)msg_buf, (uint16_t)msg_pos);
        msg_pos = 0;
    }
}

static void msg_putc(char c)
{
    if (c == '\n') {
        if (msg_pos < MSG_BUF_SZ) msg_buf[msg_pos++] = '\r';
    }
    if (msg_pos < MSG_BUF_SZ) msg_buf[msg_pos++] = c;
    if (msg_pos >= MSG_BUF_SZ - 2) msg_flush();
}

static void msg_puts(const char *s)
{
    while (*s) msg_putc(*s++);
}

/* ------------------------------------------------------------------ */
/*  Numout – identical logic to original, targeting msg_buf            */
/* ------------------------------------------------------------------ */

static void Numout(unsigned Num, int Dig, int Radix, int bwz)
{
    const char hex[] = "0123456789abcdef";
    char out[10];
    int i;
    memset(out, 0, sizeof out);
    if (Radix == 0 || Radix > 16) return;

    i = sizeof(out) - 2;
    do {
        out[i] = (char)(Num % Radix);
        Num /= Radix;
        i--;
    } while (Num);

    if (Dig == 0) Dig = sizeof(out) - 2 - i;
    for (i = sizeof(out) - Dig - 1; i < (int)sizeof(out) - 1; i++) {
        if (bwz && !out[i] && (i != (int)sizeof(out) - 2))
            msg_putc(' ');
        else { bwz = 0; msg_putc(hex[(int)out[i]]); }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void InitACM(int Baudrate)
{
    (void)Baudrate;
    USInit();
    USClear();
}

/*  Ucharavail / Ugetchar – used by CheckForEscape in tapeutil.c.
 *  We map them to the protocol escape mechanism.  */

static volatile bool esc_pending = false;

int Ucharavail(void)
{
    if (PollEscape()) esc_pending = true;
    return esc_pending ? 1 : 0;
}

unsigned char Ugetchar(void)
{
    if (esc_pending) { esc_pending = false; return '\033'; }
    /* Blocking wait – shouldn't be needed in normal operation */
    while (!Ucharavail()) {}
    esc_pending = false;
    return '\033';
}

void Uputchar(unsigned char What)
{
    msg_putc((char)What);
    msg_flush();
}

void Uputs(char *What)
{
    msg_puts(What);
    msg_flush();
}

void Uprintf(char *Form, ...)
{
    const char *p;
    va_list ap;
    int width, bwz;
    unsigned u;
    char *s;
    int i;

    va_start(ap, Form);
    for (p = Form; *p; p++) {
        if (*p != '%') { msg_putc(*p); continue; }
        ++p;
        width = 0; bwz = 1;
        if (*p == '0') { p++; bwz = 0; }
        while (*p >= '0' && *p <= '9')
            width = width * 10 + (*p++ - '0');
        switch (*p) {
        case 'd':
            u = va_arg(ap, unsigned);
            Numout(u, width, 10, bwz);
            break;
        case 'x':
            u = va_arg(ap, unsigned);
            Numout(u, width, 16, bwz);
            break;
        case 'c':
            i = va_arg(ap, int);
            msg_putc((char)i);
            break;
        case 's':
            s = va_arg(ap, char *);
            if (!width) { msg_puts(s); }
            else {
                i = (int)strlen(s) - width;
                if (i >= 0) msg_puts(s + i);
                else { msg_puts(s); for (; i; i++) msg_putc(' '); }
            }
            break;
        default: break;
        }
    }
    va_end(ap);
    msg_flush();
}

char *Ugets(char *Buf, int Len)
{
    /* Send prompt-request, then wait for PKT_INPUT from host. */
    msg_flush();                      /* flush any pending prompt text */
    SendPacket(PKT_INPUT_REQ, NULL, 0);

    uint8_t  type;
    uint16_t rlen;
    uint8_t  tmp[256];
    RecvPacket(&type, tmp, &rlen);

    if (type == PKT_INPUT && rlen > 0) {
        int copy = (rlen < (uint16_t)(Len - 1)) ? rlen : Len - 1;
        memcpy(Buf, tmp, copy);
        Buf[copy] = '\0';
        return Buf + copy;
    }
    *Buf = '\0';
    return Buf;
}

char *Hexin(unsigned int *RetVal, unsigned int *Digits, char *Buf)
{
    unsigned accum = 0;
    int dc = 0;
    char c;
    while (*Buf == ' ') Buf++;
    do {
        c = *Buf++;
        c = toupper(c);
        if (c >= '0' && c <= '9')       accum = (accum << 4) | (c - '0');
        else if (c >= 'A' && c <= 'F')  accum = (accum << 4) | (c - 'A' + 10);
        else { Buf--; *Digits = dc; *RetVal = accum; return Buf; }
        dc++;
    } while (1);
}
```

---

## main.c (modified)

```c
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
#include "comm.h"
#include "gpiodef.h"
#include "globals.h"
#include "miscsubs.h"
#include "tapedriver.h"
#include "tapeutil.h"
#include "dbserial.h"

/* ------------------------------------------------------------------ */
/*  Command table                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    void (*handler)(char *args[]);
} CMD_ENTRY;

static CMD_ENTRY cmd_table[] = {
    { "status",   CmdShowStatus   },
    { "brief",    (void(*)(char**))ShowBriefStatus },
    { "rewind",   CmdRewindTape   },
    { "unload",   CmdUnloadTape   },
    { "read",     CmdReadForward  },
    { "skip",     CmdSkip         },
    { "space",    CmdSpace        },
    { "create",   CmdCreateImage  },
    { "write",    CmdWriteImage   },
    { "addr",     CmdSetAddr      },
    { "retries",  CmdSetRetries   },
    { "stop",     CmdSetStop      },
    { "1600",     CmdSet1600      },
    { "6250",     CmdSet6250      },
    { "init",     CmdInitTape     },
    { "debug",    CmdTapeDebug    },
    { NULL, NULL }
};

/* ------------------------------------------------------------------ */
/*  Command parser                                                     */
/* ------------------------------------------------------------------ */

#define MAX_ARGS 8

static void DispatchCommand(char *line)
{
    char *args[MAX_ARGS + 1];
    char *tok;
    int   argc = 0;

    /* Tokenize by spaces */
    tok = strtok(line, " \t\r\n");
    if (!tok) return;

    /* Find command */
    char *cmd = tok;
    tok = strtok(NULL, " \t\r\n");
    while (tok && argc < MAX_ARGS) {
        args[argc++] = tok;
        tok = strtok(NULL, " \t\r\n");
    }
    args[argc] = NULL;

    /* Look up in table */
    for (int i = 0; cmd_table[i].name; i++) {
        if (strcasecmp(cmd, cmd_table[i].name) == 0) {
            cmd_table[i].handler(args);
            return;
        }
    }
    Uprintf("Unknown command: %s\n", cmd);
}

/* ------------------------------------------------------------------ */
/*  Protocol command loop – replaces old ProcessCommand()               */
/* ------------------------------------------------------------------ */

static void ProcessCommand(void)
{
    char     cmd_buf[512];
    uint8_t  pkt_type;
    uint16_t pkt_len;

    while (1) {
        SendPacket(PKT_READY, NULL, 0);

        if (RecvPacket(&pkt_type, (uint8_t *)cmd_buf, &pkt_len) < 0)
            continue;

        if (pkt_type != PKT_CMD || pkt_len == 0)
            continue;

        cmd_buf[pkt_len] = '\0';
        DispatchCommand(cmd_buf);
        SendPacket(PKT_DONE, NULL, 0);
    }
}

/* ------------------------------------------------------------------ */
/*  Initialization                                                     */
/* ------------------------------------------------------------------ */

static void Init(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    DBinit();
    Milliseconds = 0;
    InitGPIO();
    SetupSysTick();
    DelaySetup();
    InitACM(115200);
    TapeInit();
    DBprintf("Firmware ready.\n");
}

int main(void)
{
    Init();
    ProcessCommand();   /* never returns */
    return 0;
}
```

---

## Changes required in tapeutil.c

Only two kinds of edits — **replace includes** and **stub a missing function**. Everything else (logic, f_open/f_read/f_write/f_close calls, Uprintf calls) works unchanged through the shim layers.

```diff
--- a/tapeutil.c
+++ b/tapeutil.c
@@ -23,6 +23,9 @@
 #include "tapeutil.h"
 #include "tapedriver.h"
 #include "pertbits.h"
-#include "tap.h"
+#include "tap.h"          /* TAP_EOM, TAP_ERROR_FLAG, TAP_LENGTH_MASK */
+
+/* FatFS replaced by USB-backed file I/O shim */
+#include "hostio.h"
 
 //  Local variables.
```

If `tap.h` is missing from your tree, here are the defines it needs:

```c
/* tap.h – TAP image format constants */
#ifndef TAP_H
#define TAP_H

#define TAP_EOM            0xFFFFFFFF
#define TAP_ERROR_FLAG     0x80000000
#define TAP_LENGTH_MASK    0x00FFFFFF

#endif
```

No other changes to `tapeutil.c` are necessary. The `ShowRTCTime()` calls compile to no-ops via the `static inline` stub in `hostio.h`.

---

## tapehost.py (Python host application)

```python
#!/usr/bin/env python3
"""
tapehost.py – Host application for the STM32 Tape Controller.

Communicates over USB CDC with the controller firmware using a simple
binary packet protocol.  Provides a CLI and handles file I/O on behalf
of the controller.

Requirements:  pip install pyserial

Usage:  python tapehost.py [--port /dev/ttyACM0]
"""

import sys
import os
import struct
import time
import logging
import argparse
import threading

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
#  Protocol constants  (must match protocol.h on firmware)
# ---------------------------------------------------------------------------

# MCU → Host
PKT_MSG       = 0x01
PKT_READY     = 0x02
PKT_DONE      = 0x03
PKT_FOPEN     = 0x04
PKT_FWRITE    = 0x05
PKT_FREAD     = 0x06
PKT_FCLOSE    = 0x07
PKT_INPUT_REQ = 0x08

# Host → MCU
PKT_CMD       = 0x81
PKT_FRESULT   = 0x82
PKT_FDATA     = 0x83
PKT_INPUT     = 0x84
PKT_ESCAPE    = 0x85

FMODE_READ         = 0x01
FMODE_WRITE_CREATE = 0x02

PKT_HEADER_SIZE = 3

log = logging.getLogger("tapehost")

# ---------------------------------------------------------------------------
#  Connection class
# ---------------------------------------------------------------------------

class Connection:
    """Manages the USB‑CDC serial link and packet framing."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        log.info("Opening %s at %d baud", port, baudrate)
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    # ---- Packet I/O ------------------------------------------------------

    def send_packet(self, pkt_type: int, payload: bytes = b""):
        length = len(payload)
        if length > 0xFFFF:
            raise ValueError(f"Payload too large: {length}")
        hdr = struct.pack("<BH", pkt_type, length)
        data = hdr + payload
        log.debug("TX pkt type=0x%02X len=%d", pkt_type, length)
        self.ser.write(data)
        self.ser.flush()

    def recv_packet(self, timeout: float = 60.0) -> tuple:
        """Returns (pkt_type, payload_bytes).  Raises TimeoutError."""
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        try:
            hdr = self._read_exact(PKT_HEADER_SIZE, timeout)
            pkt_type, length = struct.unpack("<BH", hdr)
            payload = self._read_exact(length, timeout) if length else b""
            log.debug("RX pkt type=0x%02X len=%d", pkt_type, length)
            return pkt_type, payload
        finally:
            self.ser.timeout = old_timeout

    def _read_exact(self, n: int, timeout: float) -> bytes:
        buf = b""
        deadline = time.monotonic() + timeout
        while len(buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"Timed out reading {n} bytes (got {len(buf)})"
                )
            self.ser.timeout = min(remaining, 1.0)
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    # ---- High-level helpers ----------------------------------------------

    def send_command(self, cmd: str):
        self.send_packet(PKT_CMD, cmd.encode("utf-8"))

    def send_escape(self):
        log.info("Sending ESCAPE")
        self.send_packet(PKT_ESCAPE)

    def wait_for_ready(self, timeout: float = 10.0):
        """Block until the controller sends PKT_READY."""
        log.info("Waiting for controller READY...")
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Controller did not become ready")
            try:
                pkt_type, _ = self.recv_packet(timeout=remaining)
                if pkt_type == PKT_READY:
                    return
                if pkt_type == PKT_MSG:
                    pass  # ignore startup messages
            except TimeoutError:
                raise TimeoutError("Controller did not become ready")


# ---------------------------------------------------------------------------
#  File I/O handler  (services requests from firmware)
# ---------------------------------------------------------------------------

class FileHandler:
    """Manages a single open file on behalf of the firmware."""

    def __init__(self):
        self.fp = None
        self.path = None
        self.mode = None
        self.error = False
        self.bytes_transferred = 0

    def handle_fopen(self, conn: Connection, payload: bytes):
        mode_byte = payload[0]
        filename = payload[1:].decode("utf-8", errors="replace")
        self._close_if_open()
        log.info("FOPEN mode=0x%02X file='%s'", mode_byte, filename)
        try:
            if mode_byte == FMODE_READ:
                if not os.path.isfile(filename):
                    log.error("File not found: %s", filename)
                    conn.send_packet(PKT_FRESULT, bytes([1]))
                    return
                self.fp = open(filename, "rb")
                self.mode = "r"
            else:
                self.fp = open(filename, "wb")
                self.mode = "w"
            self.path = filename
            self.error = False
            self.bytes_transferred = 0
            conn.send_packet(PKT_FRESULT, bytes([0]))
            log.info("File opened successfully: %s", filename)
        except OSError as e:
            log.error("Failed to open %s: %s", filename, e)
            conn.send_packet(PKT_FRESULT, bytes([1]))

    def handle_fwrite(self, conn: Connection, payload: bytes):
        if not self.fp or self.mode != "w":
            log.error("FWRITE with no file open for writing")
            self.error = True
            return
        try:
            self.fp.write(payload)
            self.bytes_transferred += len(payload)
            log.debug("FWRITE %d bytes (total %d)", len(payload),
                      self.bytes_transferred)
        except OSError as e:
            log.error("Write error: %s", e)
            self.error = True

    def handle_fread(self, conn: Connection, payload: bytes):
        if not self.fp or self.mode != "r":
            log.error("FREAD with no file open for reading")
            conn.send_packet(PKT_FDATA, b"")
            return
        count = struct.unpack("<I", payload[:4])[0]
        try:
            data = self.fp.read(count)
            self.bytes_transferred += len(data)
            log.debug("FREAD req=%d got=%d (total %d)", count, len(data),
                      self.bytes_transferred)
            conn.send_packet(PKT_FDATA, data)
        except OSError as e:
            log.error("Read error: %s", e)
            conn.send_packet(PKT_FDATA, b"")

    def handle_fclose(self, conn: Connection):
        log.info("FCLOSE (transferred %d bytes)", self.bytes_transferred)
        err = self.error
        self._close_if_open()
        conn.send_packet(PKT_FRESULT, bytes([1 if err else 0]))

    def _close_if_open(self):
        if self.fp:
            try:
                self.fp.close()
            except OSError:
                pass
            self.fp = None
            self.path = None


# ---------------------------------------------------------------------------
#  Escape key listener (background thread)
# ---------------------------------------------------------------------------

class EscapeListener:
    """Watches stdin for Ctrl-C and sends PKT_ESCAPE to the controller."""

    def __init__(self, conn: Connection):
        self.conn = conn
        self.active = False

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    def send_if_active(self):
        """Call this from signal handler or keyboard interrupt."""
        if self.active:
            try:
                self.conn.send_escape()
            except Exception:
                pass


# ---------------------------------------------------------------------------
#  Command loop
# ---------------------------------------------------------------------------

def process_controller_output(conn: Connection, fh: FileHandler,
                              esc: EscapeListener):
    """
    Read packets from controller until PKT_DONE or PKT_READY.
    Handles all intermediate file-I/O and display packets.
    Returns the final packet type.
    """
    while True:
        try:
            pkt_type, payload = conn.recv_packet(timeout=120)
        except TimeoutError:
            print("\n[ERROR] Timed out waiting for controller response.")
            return PKT_DONE

        if pkt_type == PKT_MSG:
            text = payload.decode("utf-8", errors="replace")
            print(text, end="", flush=True)

        elif pkt_type == PKT_FOPEN:
            fh.handle_fopen(conn, payload)

        elif pkt_type == PKT_FWRITE:
            fh.handle_fwrite(conn, payload)

        elif pkt_type == PKT_FREAD:
            fh.handle_fread(conn, payload)

        elif pkt_type == PKT_FCLOSE:
            fh.handle_fclose(conn)

        elif pkt_type == PKT_INPUT_REQ:
            try:
                text = input()
            except EOFError:
                text = ""
            conn.send_packet(PKT_INPUT, text.encode("utf-8"))

        elif pkt_type == PKT_READY:
            return PKT_READY

        elif pkt_type == PKT_DONE:
            return PKT_DONE

        else:
            log.warning("Unknown packet type 0x%02X (len=%d)",
                        pkt_type, len(payload))


def find_port() -> str:
    """Auto-detect the tape controller CDC ACM port."""
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x5740:
            log.info("Auto-detected controller on %s", p.device)
            return p.device
    return None


HELP_TEXT = """
Available commands:

  status        Show detailed tape status
  brief         Show brief tape status
  rewind        Rewind tape
  unload        Unload tape (go offline)
  read [E]      Read and display next block (E = EBCDIC)
  skip [N]      Skip N blocks (negative = reverse)
  space [N]     Space N files (negative = reverse)
  create <file> [N]  Read tape to .tap image file (N = no rewind)
  write <file>  [N]  Write .tap image file to tape (N = no rewind)
  addr <hex>    Set drive/formatter address
  retries <N>   Set error retry count (0-9)
  stop <N> [E]  Set stop condition (N tapemarks; E = stop on error)
  1600          Set 1600 bpi PE density
  6250          Set 6250 bpi GCR density
  init          Re-initialize tape interface
  debug <hex>   Send raw command to formatter

  help          Show this help
  quit / exit   Exit the host application
"""


def main():
    parser = argparse.ArgumentParser(
        description="Tape Controller Host Application"
    )
    parser.add_argument(
        "--port", "-p", default=None,
        help="Serial port (e.g. /dev/ttyACM0 or COM3). Auto-detected if omitted."
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200,
        help="Baud rate (default: 115200, ignored for USB CDC)"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable debug logging"
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    port = args.port
    if port is None:
        port = find_port()
        if port is None:
            print("ERROR: No tape controller found. "
                  "Specify --port manually.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}  {p.description}  "
                      f"VID:PID={p.vid}:{p.pid}")
            sys.exit(1)

    try:
        conn = Connection(port, args.baud)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)

    fh  = FileHandler()
    esc = EscapeListener(conn)

    print(f"Tape Controller Host  (connected on {port})")
    print("Waiting for controller...")

    try:
        conn.wait_for_ready(timeout=15)
    except TimeoutError:
        print("ERROR: Controller did not respond. Check connection.")
        conn.close()
        sys.exit(1)

    print("Controller ready.  Type 'help' for commands.\n")

    while True:
        try:
            line = input("tape> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        lower = line.lower()
        if lower in ("quit", "exit", "q"):
            break
        if lower == "help":
            print(HELP_TEXT)
            continue

        esc.start()
        try:
            conn.send_command(line)
            process_controller_output(conn, fh, esc)
        except KeyboardInterrupt:
            print("\n[Sending abort to controller...]")
            esc.send_if_active()
            # Drain remaining output
            try:
                process_controller_output(conn, fh, esc)
            except Exception:
                pass
        except TimeoutError as e:
            print(f"\n[ERROR] {e}")
        except serial.SerialException as e:
            print(f"\n[ERROR] Serial error: {e}")
            break
        finally:
            esc.stop()

        print()  # blank line after each command

    print("Disconnecting.")
    conn.close()


if __name__ == "__main__":
    main()
```

---

## Build notes

### Firmware

Remove any FatFS / SD card source files from the build. Add the new source files:

```makefile
SRCS += hostcomm.c hostio.c
# Remove: ff.c diskio.c sdio.c (or equivalent FatFS / SD files)
```

Ensure `hostio.h` is on the include path so that `tapeutil.c` can find it. If your build previously pulled in FatFS headers via an implicit include, you may need to add `-include hostio.h` or adjust the include order.

### Host

```bash
pip install pyserial
python tapehost.py            # auto-detect port
python tapehost.py -p COM3    # or specify manually
python tapehost.py -v         # verbose / debug logging
```

---

## Summary of changes per file

| File | Change |
|------|--------|
| `protocol.h` | **New** – packet type constants |
| `hostcomm.h/c` | **New** – protocol packet layer |
| `hostio.h/c` | **New** – FatFS-compatible file shim over USB |
| `main.c` | **Rewritten** – command dispatch loop, no CLI |
| `usb.c` | **Modified** – MSC removed, block I/O added |
| `usbserial.h` | **Modified** – block I/O API |
| `comm.c` | **Modified** – output via PKT_MSG, input via protocol |
| `comm.h` | **Unchanged** (API identical) |
| `tapeutil.c` | **Minimal** – add `#include "hostio.h"`, remove FatFS include |
| `tapeutil.h` | **Unchanged** |
| `tapedriver.c/h` | **Unchanged** |
| `pertbits.h` | **Unchanged** |
| `gpiodef.h` | **Unchanged** |
| `globals.h` | **Unchanged** |
| `miscsubs.c` | **Unchanged** |
| `tapehost.py` | **New** – Python host application |
