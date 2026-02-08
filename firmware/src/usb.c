
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
#include <libopencm3/cm3/nvic.h>

#include "dbserial.h"
#include "usbserial.h"

#define USB_DATA_EP_IN   0x81
#define USB_DATA_EP_OUT  0x01
#define USB_CDC_EP_IN    0x83

#define USB_PKT  64
#define INQ_SIZE 32768   /* large enough to pre-buffer tape records via ISR */

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

/*  USGetchar – block until a byte is available.
 *  The ISR fills InQ via cdc_rx, so no explicit usbd_poll needed.
 */
int USGetchar(void)
{
    while (InQIn == InQOut)
        ;                       /* ISR fills InQ in background */
    char c = InQ[InQOut++];
    if (InQOut >= INQ_SIZE) InQOut = 0;
    return (unsigned char)c;
}

/*  USCharReady – non-blocking check.  */
int USCharReady(void)
{
    return (InQIn != InQOut) ? 1 : 0;
}

/*  _usb_write_pkt – write one packet, guarded against ISR reentrancy.
 *  Spins until the endpoint accepts the data; briefly re-enables the
 *  interrupt between attempts so the ISR can process USB events
 *  (including TX completion that frees the FIFO).
 */
static void _usb_write_pkt(const uint8_t *buf, int len)
{
    int written;
    do {
        nvic_disable_irq(NVIC_OTG_FS_IRQ);
        written = usbd_ep_write_packet(udev, USB_DATA_EP_IN, buf, len);
        nvic_enable_irq(NVIC_OTG_FS_IRQ);
    } while (written == 0);
}

int USWritechar(char c)
{
    _usb_write_pkt((const uint8_t *)&c, 1);
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
    while (count > 0) {
        int pass = (count >= USB_PKT) ? USB_PKT : count;
        _usb_write_pkt(buf, pass);
        buf   += pass;
        count -= pass;
    }
    return 0;
}