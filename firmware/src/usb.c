/*
 * usb.c - USB CDC Interface for Tape Utility
 */

#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include "usb.h"
#include "tapeutil.h"
#include "tapedriver.h"
#include "pertbits.h"
#include "globals.h"

/* Firmware version */
#define FW_MAJOR 1
#define FW_MINOR 0
#define FW_PATCH 0
#define PROTOCOL_VER 1

/* Ring buffer sizes (must be power of 2) */
#define RX_BUF_SIZE  4096
#define TX_BUF_SIZE  4096

/* Maximum bytes we can send in one read response */
/* Header(8) + ReadHdr(5) + Data must fit in TX buffer with margin */
#define MAX_READ_CHUNK  (TX_BUF_SIZE - 64)

/* Ring buffer */
typedef struct {
    uint8_t *buf;
    volatile uint16_t head, tail;
    uint16_t size;
} ringbuf_t;

/* Static buffers */
static uint8_t rx_buf[RX_BUF_SIZE], tx_buf[TX_BUF_SIZE];
static ringbuf_t rx_ring, tx_ring;
static uint8_t usb_ctrl_buf[256];
static usbd_device *usb_dev;
static volatile bool usb_configured = false;

/* Command parser state */
static uint8_t cmd_state = 0;
static usb_cmd_hdr_t cmd_hdr;
static uint8_t cmd_payload[USB_MAX_PAYLOAD];
static uint16_t cmd_payload_idx;

/* Current configuration */
static usb_config_t usb_config = {
    .retries = 3,
    .stop_tapemarks = 2,
    .stop_on_error = 0,
    .density = 0,
    .tape_address = 0,
    .timeout_ms = 5000
};

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
static void rb_init(ringbuf_t *rb, uint8_t *buf, uint16_t size) {
    rb->buf = buf; rb->head = rb->tail = 0; rb->size = size;
}
static uint16_t rb_used(ringbuf_t *rb) {
    return (rb->head - rb->tail) & (rb->size - 1);
}
static uint16_t rb_free(ringbuf_t *rb) {
    return rb->size - 1 - rb_used(rb);
}
static void rb_put(ringbuf_t *rb, uint8_t c) {
    rb->buf[rb->head] = c;
    rb->head = (rb->head + 1) & (rb->size - 1);
}
static uint8_t rb_get(ringbuf_t *rb) {
    uint8_t c = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) & (rb->size - 1);
    return c;
}
static bool rb_write(ringbuf_t *rb, const uint8_t *data, uint16_t len) {
    if (rb_free(rb) < len) return false;  /* Not enough space */
    while (len--) rb_put(rb, *data++);
    return true;
}
static void rb_read(ringbuf_t *rb, uint8_t *data, uint16_t len) {
    while (len--) *data++ = rb_get(rb);
}

/* CRC-16-CCITT */
uint16_t USB_CRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF;
        crc ^= *data++;
        crc ^= (crc & 0xFF) >> 4;
        crc ^= (crc << 12) & 0xFFFF;
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF;
    }
    return crc;
}

/* Flush TX buffer to USB - blocks until done */
static void usb_tx_flush_all(void) {
    uint8_t buf[64];
    while (rb_used(&tx_ring) > 0) {
        uint16_t len = rb_used(&tx_ring);
        if (len > 64) len = 64;
        rb_read(&tx_ring, buf, len);
        while (usbd_ep_write_packet(usb_dev, 0x82, buf, len) == 0)
            usbd_poll(usb_dev);
    }
}

/* Send response - handles large payloads by flushing as needed */
static void send_response(uint8_t status, uint16_t seq, const void *data, uint16_t len) {
    usb_resp_hdr_t hdr = { .sync = RESP_SYNC_BYTE, .status = status, .seq = seq, .len = len };
    uint16_t crc = USB_CRC16((uint8_t*)&hdr, 6);
    if (len && data) crc ^= USB_CRC16(data, len);
    hdr.crc = crc;
    
    /* Flush any pending data first */
    usb_tx_flush_all();
    
    /* Write header */
    rb_write(&tx_ring, (uint8_t*)&hdr, sizeof(hdr));
    
    /* Write payload in chunks, flushing as needed */
    if (len && data) {
        const uint8_t *p = (const uint8_t *)data;
        while (len > 0) {
            uint16_t chunk = rb_free(&tx_ring);
            if (chunk > len) chunk = len;
            if (chunk == 0) {
                usb_tx_flush_all();
                continue;
            }
            rb_write(&tx_ring, p, chunk);
            p += chunk;
            len -= chunk;
        }
    }
    
    /* Flush everything */
    usb_tx_flush_all();
}

/* Translate tape status to response code */
static uint8_t translate_status(uint16_t tstat) {
    if (tstat & TSTAT_OFFLINE)   return RESP_ERR_OFFLINE;
    if (tstat & TSTAT_HARDERR)   return RESP_HARDERR;
    if (tstat & TSTAT_TAPEMARK)  return RESP_TAPEMARK;
    if (tstat & TSTAT_EOT)       return RESP_EOT;
    if (tstat & TSTAT_BLANK)     return RESP_BLANK;
    if (tstat & TSTAT_LENGTH)    return RESP_LENGTH;
    if (tstat & TSTAT_PROTECT)   return RESP_ERR_PROTECTED;
    if (tstat & TSTAT_CORRERR)   return RESP_CORRERR;
    return RESP_OK;
}

/* Command handlers */
static void handle_get_info(uint16_t seq) {
    usb_device_info_t info = {
        .protocol_ver = PROTOCOL_VER,
        .fw_major = FW_MAJOR, .fw_minor = FW_MINOR, .fw_patch = FW_PATCH,
        .capabilities = 0x03,
        .max_payload = USB_MAX_PAYLOAD,
        .buffer_size = TAPE_BUFFER_SIZE
    };
    send_response(RESP_OK, seq, &info, sizeof(info));
}

static void handle_get_status(uint16_t seq) {
    tape_status_t ts;
    TapeUtil_GetStatus(&ts);
    usb_tape_status_t st = {
        .raw_status = ts.raw_status,
        .online = ts.online,
        .ready = ts.ready,
        .loadpoint = ts.loadpoint,
        .eot = ts.eot,
        .protected = ts.protected,
        .rewinding = ts.rewinding,
        .filemark = ts.filemark,
        .error = ts.hard_error || ts.soft_error,
        .position = ts.position
    };
    send_response(RESP_OK, seq, &st, sizeof(st));
}

static void handle_get_config(uint16_t seq) {
    usb_config.retries = TapeRetries;
    usb_config.stop_tapemarks = StopTapemarks;
    usb_config.stop_on_error = StopAfterError ? 1 : 0;
    send_response(RESP_OK, seq, &usb_config, sizeof(usb_config));
}

static void handle_set_config(uint16_t seq, const uint8_t *data, uint16_t len) {
    if (len < sizeof(usb_config_t)) {
        send_response(RESP_ERR_PARAM, seq, NULL, 0);
        return;
    }
    memcpy(&usb_config, data, sizeof(usb_config));
    TapeRetries = usb_config.retries;
    StopTapemarks = usb_config.stop_tapemarks;
    StopAfterError = usb_config.stop_on_error;
    send_response(RESP_OK, seq, NULL, 0);
}

static void handle_set_density(uint16_t seq, const uint8_t *data) {
    if (!TapeUtil_SetDensity(data[0])) {
        send_response(RESP_ERR_PARAM, seq, NULL, 0);  /* Must be at BOT */
        return;
    }
    send_response(RESP_OK, seq, NULL, 0);
}

static void handle_rewind(uint16_t seq) {
    uint16_t stat = TapeUtil_Rewind();
    send_response(translate_status(stat), seq, NULL, 0);
}

static void handle_unload(uint16_t seq) {
    uint16_t stat = TapeUtil_Unload();
    send_response(translate_status(stat), seq, NULL, 0);
}

static void handle_skip_block(uint16_t seq, const uint8_t *data) {
    int16_t count = (int16_t)(data[0] | (data[1] << 8));
    uint16_t stat = TapeUtil_SkipBlock(count);
    send_response(translate_status(stat), seq, NULL, 0);
}

static void handle_skip_file(uint16_t seq, const uint8_t *data) {
    int16_t count = (int16_t)(data[0] | (data[1] << 8));
    uint16_t stat = TapeUtil_SkipFile(count);
    send_response(translate_status(stat), seq, NULL, 0);
}

static void handle_read_block(uint16_t seq) {
    int bytesRead = 0;
    uint8_t flags = 0;
    uint16_t stat = TapeUtil_ReadBlock(TapeBuffer, TAPE_BUFFER_SIZE, &bytesRead, &flags);
    
    usb_read_hdr_t hdr = { .length = (uint32_t)bytesRead, .flags = flags };
    
    /* Determine response status */
    uint8_t resp_status = translate_status(stat);
    if (bytesRead > 0) resp_status = RESP_DATA;
    
    /* Build combined payload: read_hdr + data */
    uint16_t total_len = sizeof(hdr) + bytesRead;
    
    /* Compute CRC over: resp_header(6 bytes, excluding crc field) + read_hdr + data */
    usb_resp_hdr_t resp = { 
        .sync = RESP_SYNC_BYTE, 
        .status = resp_status, 
        .seq = seq, 
        .len = total_len,
        .crc = 0 
    };
    
    uint16_t crc = USB_CRC16((uint8_t*)&resp, 6);
    crc ^= USB_CRC16((uint8_t*)&hdr, sizeof(hdr));
    if (bytesRead > 0) {
        crc ^= USB_CRC16(TapeBuffer, bytesRead);
    }
    resp.crc = crc;
    
    /* Flush any pending data */
    usb_tx_flush_all();
    
    /* Send response header */
    rb_write(&tx_ring, (uint8_t*)&resp, sizeof(resp));
    usb_tx_flush_all();
    
    /* Send read header */
    rb_write(&tx_ring, (uint8_t*)&hdr, sizeof(hdr));
    usb_tx_flush_all();
    
    /* Send data in chunks */
    if (bytesRead > 0) {
        int sent = 0;
        while (sent < bytesRead) {
            uint16_t chunk = rb_free(&tx_ring);
            if (chunk > (bytesRead - sent)) chunk = bytesRead - sent;
            if (chunk == 0) {
                usb_tx_flush_all();
                continue;
            }
            rb_write(&tx_ring, &TapeBuffer[sent], chunk);
            DBprintf("stat=%d bytes=%d buffer[0]=%d\n", stat, bytesRead, TapeBuffer[0]);
            sent += chunk;
            usb_tx_flush_all();
        }
    }
}

static void handle_write_block(uint16_t seq, const uint8_t *data, uint16_t len) {
    uint16_t stat = TapeUtil_WriteBlock(data, len);
    send_response(translate_status(stat), seq, NULL, 0);
}

static void handle_write_filemark(uint16_t seq) {
    uint16_t stat = TapeUtil_WriteFilemark();
    send_response(translate_status(stat), seq, NULL, 0);
}

/* Debug info structure */
typedef struct __attribute__((packed)) {
    uint16_t tape_buffer_size;
    uint16_t usb_max_payload;
    uint16_t tx_buf_size;
    uint16_t rx_buf_size;
    uint16_t tx_used;
    uint16_t rx_used;
    uint32_t position;
    uint32_t bytes_transferred;
    uint32_t errors;
} debug_info_t;

static void handle_debug_info(uint16_t seq) {
    debug_info_t info = {
        .tape_buffer_size = TAPE_BUFFER_SIZE,
        .usb_max_payload = USB_MAX_PAYLOAD,
        .tx_buf_size = TX_BUF_SIZE,
        .rx_buf_size = RX_BUF_SIZE,
        .tx_used = rb_used(&tx_ring),
        .rx_used = rb_used(&rx_ring),
        .position = TapeUtil_GetPosition(),
        .bytes_transferred = TapeUtil_GetBytesTransferred(),
        .errors = TapeUtil_GetErrorCount()
    };
    send_response(RESP_OK, seq, &info, sizeof(info));
}

/* Process a complete command */
static void process_command(void) {
    uint16_t seq = cmd_hdr.seq;
    switch (cmd_hdr.cmd) {
        case CMD_NOP:           send_response(RESP_OK, seq, NULL, 0); break;
        case CMD_GET_INFO:      handle_get_info(seq); break;
        case CMD_GET_STATUS:    handle_get_status(seq); break;
        case CMD_GET_CONFIG:    handle_get_config(seq); break;
        case CMD_SET_CONFIG:    handle_set_config(seq, cmd_payload, cmd_hdr.len); break;
        case CMD_SET_DENSITY:   handle_set_density(seq, cmd_payload); break;
        case CMD_REWIND:        handle_rewind(seq); break;
        case CMD_UNLOAD:        handle_unload(seq); break;
        case CMD_SKIP_BLOCK:    handle_skip_block(seq, cmd_payload); break;
        case CMD_SKIP_FILE:     handle_skip_file(seq, cmd_payload); break;
        case CMD_READ_BLOCK:    handle_read_block(seq); break;
        case CMD_WRITE_BLOCK:   handle_write_block(seq, cmd_payload, cmd_hdr.len); break;
        case CMD_WRITE_FILEMARK: handle_write_filemark(seq); break;
        case CMD_RESET:
            TapeUtil_ResetCounters();
            send_response(RESP_OK, seq, NULL, 0);
            break;
        case CMD_DEBUG_INFO:
            handle_debug_info(seq);
            break;
        default:
            send_response(RESP_ERR_CMD, seq, NULL, 0);
            break;
    }
}

/* USB Callbacks */
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
    if (len > 0 && rb_free(&rx_ring) >= (uint16_t)len)
        rb_write(&rx_ring, buf, len);
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

/* Public API */
void USB_Init(void) {
    rb_init(&rx_ring, rx_buf, RX_BUF_SIZE);
    rb_init(&tx_ring, tx_buf, TX_BUF_SIZE);
    
    /* Initialize tape utility module */
    TapeUtil_Init();
    
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
    
    usb_dev = usbd_init(&otgfs_usb_driver, &dev_desc, &conf_desc,
        usb_strings, 3, usb_ctrl_buf, sizeof(usb_ctrl_buf));
    usbd_register_set_config_callback(usb_dev, cdcacm_set_config);
}

bool USB_IsConnected(void) {
    return usb_configured;
}

void USB_Poll(void) {
    usbd_poll(usb_dev);
    
    /* Transmit any pending data */
    if (rb_used(&tx_ring) > 0) {
        uint8_t buf[64];
        uint16_t len = rb_used(&tx_ring);
        if (len > 64) len = 64;
        rb_read(&tx_ring, buf, len);
        usbd_ep_write_packet(usb_dev, 0x82, buf, len);
    }
}

bool USB_ProcessCommands(void) {
    bool processed = false;
    
    while (rb_used(&rx_ring)) {
        uint8_t c = rb_get(&rx_ring);
        switch (cmd_state) {
            case 0: if (c == CMD_SYNC_BYTE) cmd_state = 1; break;
            case 1: cmd_hdr.cmd = c; cmd_state = 2; break;
            case 2: cmd_hdr.seq = c; cmd_state = 3; break;
            case 3: cmd_hdr.seq |= c << 8; cmd_state = 4; break;
            case 4: cmd_hdr.len = c; cmd_state = 5; break;
            case 5: cmd_hdr.len |= c << 8; cmd_state = 6; break;
            case 6: cmd_hdr.crc = c; cmd_state = 7; break;
            case 7:
                cmd_hdr.crc |= c << 8;
                cmd_payload_idx = 0;
                cmd_state = (cmd_hdr.len > 0 && cmd_hdr.len <= USB_MAX_PAYLOAD) ? 8 : 9;
                if (cmd_hdr.len > USB_MAX_PAYLOAD) cmd_state = 0;
                break;
            case 8:
                cmd_payload[cmd_payload_idx++] = c;
                if (cmd_payload_idx >= cmd_hdr.len) cmd_state = 9;
                break;
        }
        if (cmd_state == 9) {
            cmd_hdr.sync = CMD_SYNC_BYTE;
            uint16_t calc = USB_CRC16((uint8_t*)&cmd_hdr, 6);
            if (cmd_hdr.len) calc ^= USB_CRC16(cmd_payload, cmd_hdr.len);
            if (calc == cmd_hdr.crc) {
                process_command();
                processed = true;
            } else {
                send_response(RESP_ERR_CHECKSUM, cmd_hdr.seq, NULL, 0);
            }
            cmd_state = 0;
        }
    }
    return processed;
}