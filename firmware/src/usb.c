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

/* ---- Block receive ---------------------------------------------- */

static int ring_avail(void)
{
    int d = rx_in - rx_out;
    return (d >= 0) ? d : d + RX_RING;
}

static int recv_from_ring(uint8_t *Data, int MaxLen)
{
    int got = 0;
    while (got < MaxLen && ring_avail() > 0) {
        Data[got++] = rx_ring[rx_out++];
        if (rx_out >= RX_RING) rx_out = 0;
    }
    return got;
}

/* Blocking: waits until at least 1 byte, then drains up to MaxLen */
int USRecvBlock(uint8_t *Data, int MaxLen)
{
    while (ring_avail() == 0)
        usbd_poll(usb_dev);
    return recv_from_ring(Data, MaxLen);
}

/* Non-blocking: polls USB once, returns however many bytes are available
 * (may be 0) */
int USRecvAvail(uint8_t *Data, int MaxLen)
{
    usbd_poll(usb_dev);
    return recv_from_ring(Data, MaxLen);
}