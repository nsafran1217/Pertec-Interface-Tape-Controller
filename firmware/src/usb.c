#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/msc.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dwc/otg_fs.h>

#include "dbserial.h"

#include "sdiosubs.h"
#include "comm.h"
#include "usbserial.h"

#define SIZEOFARRAY(x) (sizeof(x) / sizeof((x)[0]))

#define USB_DATA_ENDPOINT_IN  0x81
#define USB_DATA_ENDPOINT_OUT 0x01
#define USB_MSC_ENDPOINT_IN   0x82
#define USB_MSC_ENDPOINT_OUT  0x02
#define USB_CDC_ENDPOINT_IN   0x83

#define INTERFACE_NO_1 0
#define INTERFACE_NO_2 2

#define USB_PACKET_SIZE 64
#define INPUT_QUEUE_SIZE 1024+64        // size of input queue

//	Prototypes.

static uint32_t get_block_count(void);
static int read_block(uint32_t lba, uint8_t *copy_to);
static int write_block(uint32_t lba, const uint8_t *copy_from);
static void usb_suspend_callback( void);

static char
  ReceiveBuffer[ 65],                   // Where characters come in
  InputQueue[ INPUT_QUEUE_SIZE];        // where we queue input up 

static volatile int
  InQIn,
  InQOut;                               // input queue in/out

static char 
  usb_serial_number[13]; /* 12 bytes of desig and a \0 */

static usbd_device *usb_device;

static bool 
  usb_is_connected = false;

static const struct 
  usb_device_descriptor dev_descr = 
{
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = USB_CLASS_CDC,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x0483,         // ST Microelectronics
  .idProduct = 0x5740,        // STM32F407--the easiest one we can use
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct 
{
  struct usb_cdc_header_descriptor 
    header;
  struct usb_cdc_call_management_descriptor 
    call_mgmt;
  struct usb_cdc_acm_descriptor 
    acm;
  struct usb_cdc_union_descriptor 
    cdc_union;
} __attribute__((packed))

cdcacm_functional_descriptors = 
{
  .header =
  {
    .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
    .bcdCDC = 0x0110,
  },
  .call_mgmt =
  {
     .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
     .bDescriptorType = CS_INTERFACE,
     .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
     .bmCapabilities = 0,
     .bDataInterface = INTERFACE_NO_1 + 1,
  },
  .acm =
  {
     .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
     .bDescriptorType = CS_INTERFACE,
     .bDescriptorSubtype = USB_CDC_TYPE_ACM,
     .bmCapabilities = 0,
  },
  .cdc_union = 
  {
     .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
     .bDescriptorType = CS_INTERFACE,
     .bDescriptorSubtype = USB_CDC_TYPE_UNION,
     .bControlInterface = INTERFACE_NO_1,
     .bSubordinateInterface0 = INTERFACE_NO_1 + 1,
  }
};

static const struct usb_endpoint_descriptor comm_endp[] = 
{
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_CDC_ENDPOINT_IN,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
  }
};

static const struct usb_endpoint_descriptor data_endp[] = 
{
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DATA_ENDPOINT_OUT,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = USB_PACKET_SIZE,
    .bInterval = 1,
  },
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DATA_ENDPOINT_IN,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = USB_PACKET_SIZE,
    .bInterval = 1,
  }
};

#ifndef DISABLE_USB_MSC
static const struct usb_endpoint_descriptor msc_endp[] = 
{
  {
   .bLength = USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   .bEndpointAddress = USB_MSC_ENDPOINT_OUT,
   .bmAttributes = USB_ENDPOINT_ATTR_BULK,
   .wMaxPacketSize = USB_PACKET_SIZE,
   .bInterval = 1,
  },
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_MSC_ENDPOINT_IN,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = USB_PACKET_SIZE,
    .bInterval = 1,
  }
};
#endif

static const struct usb_interface_descriptor comm_iface[] = 
{
  {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = INTERFACE_NO_1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors),
  }
};

static const struct usb_interface_descriptor data_iface[] = 
{
  {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = INTERFACE_NO_1 + 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
  }
};

#ifndef DISABLE_USB_MSC
static const struct usb_interface_descriptor msc_iface[] = 
{
  {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = INTERFACE_NO_2,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_MSC,
    .bInterfaceSubClass = USB_MSC_SUBCLASS_SCSI,
    .bInterfaceProtocol = USB_MSC_PROTOCOL_BBB,
    .iInterface = 0,

    .endpoint = msc_endp,
  }
};
#endif

#ifndef DISABLE_USB_MSC
static const struct usb_interface ifaces[] = 
{
  {
    .num_altsetting = 1,
    .altsetting = comm_iface,
  },
  {
    .num_altsetting = 1,
    .altsetting = data_iface,
  },
  {
    .num_altsetting = 1,
    .altsetting = msc_iface,
  },
};
#else
static const struct usb_interface ifaces[] = 
{
  {
    .num_altsetting = 1,
    .altsetting = comm_iface,
  },
  {
    .num_altsetting = 1,
    .altsetting = data_iface,
  },
};
#endif

static const struct usb_config_descriptor config_descr = 
{
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
#ifdef DISABLE_USB_MSC
  .bNumInterfaces = 2,
#else
  .bNumInterfaces = 3,
#endif
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x32,
  .interface = ifaces,
};

static const char *usb_strings[] = 
{
  "Tape Controller",
  "",
  usb_serial_number,
};

// Buffer to be used for control requests. 

uint8_t 
  usbd_control_buffer[128];

//*	CDC ACM Control Request - Serivce CDC ACM control request.
//	----------------------------------------------------------
//

static enum usbd_request_return_codes cdcacm_control_request(
        usbd_device *usbd_dev, struct usb_setup_data *req, 
        uint8_t **buf, uint16_t *len, 
        void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) 
{

  (void)complete;
  (void)usbd_dev;

// Dummy line codig for the benefit of Windows

  static const struct usb_cdc_line_coding line_coding = 
  {
    .dwDTERate = 115200,
    .bCharFormat = USB_CDC_1_STOP_BITS,
    .bParityType = USB_CDC_NO_PARITY,
    .bDataBits = 0x08
  };

  switch (req->bRequest) 
  {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: 
      return USBD_REQ_HANDLED;

    case USB_CDC_REQ_GET_LINE_CODING:
      if ( *len < sizeof(struct usb_cdc_line_coding) ) 
      {
	return USBD_REQ_NOTSUPP;
      }
      *buf = (uint8_t *) &line_coding;
      *len = sizeof(struct usb_cdc_line_coding);
      return USBD_REQ_HANDLED;

    case USB_CDC_REQ_SET_LINE_CODING:
      if (*len < sizeof(struct usb_cdc_line_coding)) 
      {
        return USBD_REQ_NOTSUPP;
      }
      return USBD_REQ_HANDLED;
  } // switch
  return USBD_REQ_NOTSUPP;
} // cdcacm_control_request


//  CDC_ACM Received data request.
//  ------------------------------
//
//  This is where we get a data packet from the host.
//

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  int 
    len,
    i;

  (void)ep;

  len = usbd_ep_read_packet(usbd_dev, USB_DATA_ENDPOINT_OUT, 
            ReceiveBuffer, USB_PACKET_SIZE);

  for ( i = 0; i < len; i++)
  {
    int iq;
    iq = InQIn+1;
    if ( iq >= INPUT_QUEUE_SIZE)
      iq = 0;
    if ( iq != InQOut)
    {  
      InputQueue[InQIn] = ReceiveBuffer[i];
      InQIn = iq;               // next in
    }
  } // for each received character
} // cdcacm_data_rx_cb

static void usb_cdc_set_config_callback(usbd_device *usbd_dev, uint16_t wValue) {
    (void)wValue;

// We are being configured, usb must be connected.

  if (!usb_is_connected) 
  {
    usb_is_connected = true;
  }

  usbd_ep_setup(usbd_dev, USB_DATA_ENDPOINT_OUT, USB_ENDPOINT_ATTR_BULK, USB_PACKET_SIZE, cdcacm_data_rx_cb);
  usbd_ep_setup(usbd_dev, USB_DATA_ENDPOINT_IN, USB_ENDPOINT_ATTR_BULK, USB_PACKET_SIZE, NULL);
  usbd_ep_setup(usbd_dev, USB_CDC_ENDPOINT_IN, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(usbd_dev,
     USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
     USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
     cdcacm_control_request);
} // usb_cdc_set_config_callback

static void usb_suspend_callback() 
{
// We are being suspended, disconnect.
  if (usb_is_connected) 
  {
        usb_is_connected = false;
  }
} // usb_suspend

//*	Mass storage routines.
//	----------------------
//	
//	All refer to routines in dskio.c
//

static uint32_t get_block_count(void) 
{
  DBprintf( "Get block count returns %d\n", SD_GetCardSize());

    return SD_GetCardSize();
} // get_block_count

//*	Read an SD block.
//	-----------------
//
//	Called from the MSC driver
//

static int read_block(uint32_t lba, uint8_t *copy_to) 
{

  int stat;

  stat = SD_ReadBlocks( copy_to, lba, 1);
  SD_WaitComplete();
  return stat;
} // read_block

//*	Write an SD block
//	-----------------
//
//	Called from the MSC driver
//	

static int write_block(uint32_t lba, const uint8_t *copy_from) 
{

  int stat;

  stat = SD_WriteBlocks( (void *) copy_from, lba, 1);
  SD_WaitComplete();
  return stat;
} // write_block


//  USInit - Initialize USB interface.
//  -----------------------------------
//
//  Must be called before any I/O is done.  Returns 0.
//

int USInit(void) 
{

  rcc_periph_clock_enable(RCC_GPIOA);
//  rcc_periph_clock_enable(RCC_OTGHS);
  rcc_periph_clock_enable(RCC_OTGFS);

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
  gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

  desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));

  OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
  OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
  OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD;		// force device mode

  usb_device = usbd_init(&otgfs_usb_driver,
                         &dev_descr,
                         &config_descr,
                         usb_strings,
                         SIZEOFARRAY(usb_strings),
                         usbd_control_buffer,
                         sizeof(usbd_control_buffer));


  usbd_register_set_config_callback(usb_device, usb_cdc_set_config_callback);
  usbd_register_suspend_callback(usb_device, usb_suspend_callback);

  if (!SD_GetCardSize())
  {
    DBprintf( "Initializing SD I/O\n");
    SD_Init();				// if SD not initialized
    DBprintf( "SD init returns\n");
  }

#ifndef DISABLE_USB_MSC
  usb_msc_init(usb_device,
               USB_MSC_ENDPOINT_IN,
               USB_PACKET_SIZE,
               USB_MSC_ENDPOINT_OUT,
               USB_PACKET_SIZE,
               "Sydex",
               "SD Card",
               "0.01",
               get_block_count(),
               read_block,
               write_block);
#else
  DBprintf("USB MSC disabled at compile time\n");
#endif


  return 0;               
} // USInit

void otg_fs_isr(void) 
{
  if (usb_device) 
  {
      usbd_poll(usb_device);
  }
} // otg_fs_isr

bool usb_get_connected(void) 
{
  return usb_is_connected;
} // usb_get_connected

void usb_disconnect( void) 
{
  if (usb_device) 
  {
    usbd_disconnect(usb_device, true);
  }
} // usb_disconnect

//*	Character Input.
//	----------------

//*     USClear - Clear buffer contents.
//      --------------------------------
//
//      Removes any initialization messages from input buffer.
//

void USClear( void)
{

  InQIn = 0;            
  InQOut = 0;           // empty the input buffer
} // USClear

//  USGetchar - Get a character from input.
//  ---------------------------------------
//
//    Not the most efficient.  We ignore any buffer overflow.
//

int USGetchar( void)
{

  char
    retChar;

  if ( !usb_is_connected)
    USInit();             // if not initialized, do it.
  while (InQIn == InQOut)
  {
    usbd_poll( usb_device);       // wait for input
  } // get a character if there's none
  
//  At this point we know that there's something in the buffer.  
  
  retChar = InputQueue[ InQOut++];      // get the character
  if ( InQOut >= INPUT_QUEUE_SIZE)
    InQOut = 0;                         // wrap around to the start
  return (unsigned char) retChar;
} // USGetchar

//* USCharReady - See if a character is waiting.
//  --------------------------------------------
//
//  Return 0 if no character; 1 otherwise.
//

int USCharReady( void)
{
  if ( !usb_is_connected)
    USInit();             // if not initialized, do it.

  usbd_poll(usb_device);    // poll
  return ( InQIn == InQOut) ? 0 : 1;
} // USCharReady

//*	Character output.
//	-----------------


//* USWritechar - Write a single character without "cooking"
//  --------------------------------------------------------
//
//    Just passes the data through.
//

int USWritechar( char What)
{

  if ( !usb_is_connected)
    USInit();             // if not initialized, do it.

  while (usbd_ep_write_packet(usb_device, USB_DATA_ENDPOINT_IN, 
      (uint8_t *)&What, 1) == 0);

  usbd_poll(usb_device);    // poll
  return What;
} // USWritechar


//* USPutchar - Put a single character to output.
//  ---------------------------------------------
//
//    If you've got a string, use USPuts(); it's faster.
//

int USPutchar( char What)
{

  const char *crlf="\r\n";

  if ( !usb_is_connected)
    USInit();             // if not initialized, do it.
  if ( What == '\n')
  {
    while (usbd_ep_write_packet(usb_device, USB_DATA_ENDPOINT_IN, 
      (uint8_t *) crlf, 2) == 0);
  }
  else  
  {
    while (usbd_ep_write_packet(usb_device, USB_DATA_ENDPOINT_IN, 
      (uint8_t *)&What, 1) == 0);
  }
  usbd_poll(usb_device);    // poll
  return What;
} // USPutchar

//* USPuts - Put a character string to output.
//  ------------------------------------------
//
//  Uses packet-mode transfer, so is somewhat faster.
//

void USPuts( char *What)
{
 
  char
    localBuf[64];
  char 
    *p;
  int
    i;

   
  if ( !usb_is_connected)
    USInit();             // if not initialized, do it.

  while (*What)
  {
    p = localBuf;
    for (  i = 0; i < 62; i++)
    {
      if ( *What == 0)
        break;
      if ( *What == '\n')
      {
        *p++ = '\r';        // make sure of cr-lf  
        i++;
      } // convert LF to CR-LF  
      *p++ = *What++;
    } // for
    if ( i)
    {  // if we have anything to write
      while (usbd_ep_write_packet(usb_device, USB_DATA_ENDPOINT_IN, 
        (uint8_t *) localBuf, i) == 0);
    }
    usbd_poll(usb_device);    // poll
  } // until we've reached the end.
  return;
} // USPuts

//*  USWriteBlock - Write a block of characters without "cooking"
//   ------------------------------------------------------------
//
//      This is a bit more efficient than single-character writes.
//
//      Always returns zero.
//

int USWriteBlock( uint8_t *What, int Count)
{

  int 
    thisPass;

  while ( Count != 0)
  {
      thisPass = ( Count >= USB_PACKET_SIZE) ? USB_PACKET_SIZE-1 : Count;
      
      while( usbd_ep_write_packet(usb_device, USB_DATA_ENDPOINT_IN, 
           What, thisPass) == 0)
        usbd_poll(usb_device);  // poll
      What += thisPass;         // advance buffer
      Count -= thisPass;
  }
  return 0;

} //  USWriteBlock
