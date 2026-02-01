//  usbserial - procedures for usb and UART serial I/O.

#ifndef _USBSERIAL
#define _USBSERIAL

int USInit( void);  		// initialize
void USClear( void);		// clear buffer contents
int USPutchar( char);   	// put a character
int USWritechar( char);		// "raw" write character
int USGetchar( void);   	// get a character
int USCharReady( void); 	// test if character ready
void USPuts( char *What);	// put string
int USWriteBlock( uint8_t *What, int Count);	// write a block of data
// Second CDC interface (streaming) - same semantics but send on second ACM
int USPutchar2( char);
int USWritechar2( char);
int USGetchar2( void);
int USCharReady2( void);
int USWriteBlock2( uint8_t *What, int Count);

//  Utility usb defs.

bool usb_get_connected(void);
void usb_disconnect(void);
bool usb_get_connected2(void);

#endif
