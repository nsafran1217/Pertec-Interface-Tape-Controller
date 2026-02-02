/*
 * usb.c - USB CDC Interface with file-like streaming
 */

#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "usb.h"
#include "globals.h"

/* Firmware version */
#define FW_MAJOR 1
#define FW_MINOR 0

/* Buffer sizes */
#define RX_BUF_SIZE  4096
#define TX_BUF_SIZE  4096
#define CMD_BUF_SIZE 64

/* Packet framing */
#define FRAME_START  0xAA
#define FRAME_END    0x55
#define FRAME_ESC    0x1B

/* Static buffers */
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;
static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;

static uint8_t cmd_buf[CMD_BUF_SIZE];
static uint8_t usb_ctrl_buf[256];
static usbd_device *usb_dev;
static volatile bool usb_configured = false;
static volatile bool abort_requested = false;
static USB_Direction current_direction = USB_DIR_NONE;

/* USB Descriptors */
static const struct usb_device_descriptor dev_desc = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1209,
    .idProduct = 0x0001,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor comm_ep[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_ep[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    },
};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,
    .endpoint = comm_ep,
    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_ep,
}};

static const struct usb_interface ifaces[] = {
    { .num_altsetting = 1, .altsetting = comm_iface },
    { .num_altsetting = 1, .altsetting = data_iface },
};

static const struct usb_config_descriptor conf_desc = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,
    .interface = ifaces,
};

static const char *usb_strings[] = {
    "PERTEC Tape Interface",
    "STM32 Tape Controller",
    "001",
};

/* Ring buffer helpers */
static uint16_t rb_used(volatile uint16_t *head, volatile uint16_t *tail, uint16_t size) {
    return (*head - *tail) & (size - 1);
}

static uint16_t rb_free(volatile uint16_t *head, volatile uint16_t *tail, uint16_t size) {
    return size - 1 - rb_used(head, tail, size);
}

/* Low-level USB TX */
static void usb_tx_byte(uint8_t b) {
    if (rb_free(&tx_head, &tx_tail, TX_BUF_SIZE) > 0) {
        tx_buf[tx_head] = b;
        tx_head = (tx_head + 1) & (TX_BUF_SIZE - 1);
    }
}

static void usb_tx_flush(void) {
    uint8_t buf[64];
    while (rb_used(&tx_head, &tx_tail, TX_BUF_SIZE) > 0) {
        uint16_t len = rb_used(&tx_head, &tx_tail, TX_BUF_SIZE);
        if (len > 64) len = 64;
        for (uint16_t i = 0; i < len; i++) {
            buf[i] = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) & (TX_BUF_SIZE - 1);
        }
        uint32_t timeout = 100000;
        while (usbd_ep_write_packet(usb_dev, 0x82, buf, len) == 0) {
            usbd_poll(usb_dev);
            if (--timeout == 0) return;
        }
    }
}

/* Send framed packet: [START][TYPE][LEN_LO][LEN_HI][DATA...][END] */
static void send_packet(uint8_t type, const void *data, uint16_t len) {
    const uint8_t *p = (const uint8_t *)data;
    
    usb_tx_byte(FRAME_START);
    usb_tx_byte(type);
    usb_tx_byte(len & 0xFF);
    usb_tx_byte((len >> 8) & 0xFF);
    
    for (uint16_t i = 0; i < len; i++) {
        if (p[i] == FRAME_START || p[i] == FRAME_END || p[i] == FRAME_ESC) {
            usb_tx_byte(FRAME_ESC);
            usb_tx_byte(p[i] ^ 0x20);
        } else {
            usb_tx_byte(p[i]);
        }
        /* Flush periodically to prevent buffer overflow */
        if (rb_free(&tx_head, &tx_tail, TX_BUF_SIZE) < 64) {
            usb_tx_flush();
        }
    }
    
    usb_tx_byte(FRAME_END);
    usb_tx_flush();
}

/* USB callbacks */
static enum usbd_request_return_codes cdcacm_control(usbd_device *dev,
    struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
    void (**complete)(usbd_device*, struct usb_setup_data*)) {
    (void)dev; (void)complete; (void)buf; (void)len;
    if (req->bmRequestType != 0x21 || req->bRequest != USB_CDC_REQ_SET_CONTROL_LINE_STATE)
        return USBD_REQ_NOTSUPP;
    return USBD_REQ_HANDLED;
}

static void cdcacm_data_rx(usbd_device *dev, uint8_t ep) {
    (void)ep;
    uint8_t buf[64];
    int len = usbd_ep_read_packet(dev, 0x01, buf, 64);
    for (int i = 0; i < len; i++) {
        if (rb_free(&rx_head, &rx_tail, RX_BUF_SIZE) > 0) {
            rx_buf[rx_head] = buf[i];
            rx_head = (rx_head + 1) & (RX_BUF_SIZE - 1);
        }
    }
}

static void cdcacm_set_config(usbd_device *dev, uint16_t val) {
    (void)val;
    usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx);
    usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
    usbd_register_control_callback(dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control);
    usb_configured = true;
}

/* Read one byte from RX buffer, returns -1 if empty */
static int rx_get_byte(void) {
    if (rx_head == rx_tail) return -1;
    uint8_t b = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_BUF_SIZE - 1);
    return b;
}

/* Try to receive a framed packet, returns packet type or -1 */
static int receive_packet(uint8_t *data, uint16_t max_len, uint16_t *out_len) {
    static uint8_t state = 0;
    static uint16_t pkt_len = 0, pkt_idx = 0;
    static uint8_t pkt_type = 0;
    static bool escaped = false;
    
    while (rx_head != rx_tail) {
        int c = rx_get_byte();
        if (c < 0) break;
        uint8_t b = (uint8_t)c;
        
        switch (state) {
            case 0: /* Waiting for START */
                if (b == FRAME_START) state = 1;
                break;
            case 1: /* Got START, expect TYPE */
                pkt_type = b;
                state = 2;
                break;
            case 2: /* Expect LEN_LO */
                pkt_len = b;
                state = 3;
                break;
            case 3: /* Expect LEN_HI */
                pkt_len |= (b << 8);
                pkt_idx = 0;
                escaped = false;
                state = 4;
                break;
            case 4: /* Reading data or END */
                if (b == FRAME_END) {
                    state = 0;
                    *out_len = pkt_idx;
                    return pkt_type;
                } else if (b == FRAME_ESC) {
                    escaped = true;
                } else {
                    if (escaped) {
                        b ^= 0x20;
                        escaped = false;
                    }
                    if (pkt_idx < max_len) {
                        data[pkt_idx++] = b;
                    }
                }
                break;
        }
    }
    return -1;
}

/* Public API */
void USB_Init(void) {
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
    
    usb_dev = usbd_init(&otgfs_usb_driver, &dev_desc, &conf_desc,
        usb_strings, 3, usb_ctrl_buf, sizeof(usb_ctrl_buf));
    usbd_register_set_config_callback(usb_dev, cdcacm_set_config);
}

void USB_Poll(void) {
    usbd_poll(usb_dev);
}

bool USB_IsConnected(void) {
    return usb_configured;
}

uint8_t USB_CheckCommand(uint8_t *params, uint16_t *param_len) {
    uint16_t len = 0;
    int type = receive_packet(cmd_buf, CMD_BUF_SIZE, &len);
    
    if (type < 0) return 0;
    
    /* Check for abort */
    if (type == HOST_CMD_ABORT) {
        abort_requested = true;
        return HOST_CMD_ABORT;
    }
    
    /* Copy parameters if requested */
    if (params && len > 0) {
        uint16_t copy_len = (param_len && *param_len < len) ? *param_len : len;
        memcpy(params, cmd_buf, copy_len);
    }
    if (param_len) *param_len = len;
    
    return (uint8_t)type;
}

void USB_SendResponse(uint8_t code) {
    send_packet(code, NULL, 0);
}

void USB_SendResponseData(uint8_t code, const void *data, uint16_t len) {
    send_packet(code, data, len);
}

void USB_SendMessage(const char *msg) {
    send_packet(RESP_DATA, msg, strlen(msg));
}

/* File-like streaming interface */

USB_Result USB_OpenTransfer(USB_Direction dir) {
    current_direction = dir;
    abort_requested = false;
    
    /* Notify host of transfer start */
    uint8_t d = (dir == USB_DIR_WRITE) ? 'W' : 'R';
    send_packet(RESP_OK, &d, 1);
    
    return USB_OK;
}

USB_Result USB_Write(const void *data, uint32_t len, uint32_t *written) {
    if (current_direction != USB_DIR_WRITE) return USB_ERR_IO;
    if (abort_requested) return USB_ERR_ABORT;
    
    /* Send data in chunks */
    const uint8_t *p = (const uint8_t *)data;
    uint32_t sent = 0;
    
    while (sent < len) {
        uint16_t chunk = (len - sent > 2048) ? 2048 : (len - sent);
        send_packet(RESP_DATA, p + sent, chunk);
        sent += chunk;
        
        /* Check for abort while sending */
        USB_Poll();
        uint16_t plen = 0;
        if (receive_packet(cmd_buf, CMD_BUF_SIZE, &plen) == HOST_CMD_ABORT) {
            abort_requested = true;
            if (written) *written = sent;
            return USB_ERR_ABORT;
        }
    }
    
    if (written) *written = sent;
    return USB_OK;
}

USB_Result USB_Read(void *data, uint32_t len, uint32_t *bytesRead) {
    if (current_direction != USB_DIR_READ) return USB_ERR_IO;
    if (abort_requested) return USB_ERR_ABORT;
    
    uint8_t *p = (uint8_t *)data;
    uint32_t total = 0;
    uint32_t timeout;
    
    /* Request data from host */
    uint8_t req[4];
    req[0] = len & 0xFF;
    req[1] = (len >> 8) & 0xFF;
    req[2] = (len >> 16) & 0xFF;
    req[3] = (len >> 24) & 0xFF;
    send_packet(RESP_NEED_DATA, req, 4);
    
    /* Wait for data packets */
    timeout = 5000000;
    while (total < len && timeout > 0) {
        USB_Poll();
        
        uint16_t pkt_len = 0;
        int pkt_type = receive_packet(cmd_buf, CMD_BUF_SIZE, &pkt_len);
        
        if (pkt_type == HOST_CMD_ABORT) {
            abort_requested = true;
            if (bytesRead) *bytesRead = total;
            return USB_ERR_ABORT;
        }
        
        if (pkt_type == RESP_DATA) {
            uint32_t copy = (pkt_len > len - total) ? (len - total) : pkt_len;
            memcpy(p + total, cmd_buf, copy);
            total += copy;
            timeout = 5000000;  /* Reset timeout on data */
        } else if (pkt_type == RESP_DONE) {
            /* Host signals end of data */
            break;
        } else {
            timeout--;
        }
    }
    
    if (bytesRead) *bytesRead = total;
    return (timeout == 0) ? USB_ERR_TIMEOUT : USB_OK;
}

USB_Result USB_CloseTransfer(void) {
    send_packet(RESP_DONE, NULL, 0);
    current_direction = USB_DIR_NONE;
    return USB_OK;
}

bool USB_CheckAbort(void) {
    /* Poll and check for abort command */
    USB_Poll();
    uint16_t len = 0;
    if (receive_packet(cmd_buf, CMD_BUF_SIZE, &len) == HOST_CMD_ABORT) {
        abort_requested = true;
    }
    return abort_requested;
}