#define MAIN 1

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


#include "license.h"


#include "comm.h"
#include "rtcsubs.h"
#include "gpiodef.h"
#include "globals.h"

#include "miscsubs.h"

#include "tapedriver.h"

#include "dbserial.h"

// Private function prototypes

static void Init( void);


//  Main program entry.
//  -------------------
//
//  Perform initialiation (see "Init" routine)
//  then exit to command processor.
//

int main(void) 
{

  Init();             // go do some initialization

//   Uprintf( "Delay is %d\n",rcc_apb1_frequency / 4000000 - 1);

  ProcessCommand();                     // never comes back
  return 0; 
} // main


//* Initialization tasks.
//  ---------------------
//
//	Sets things up:
//
//	  1)  GPIO configuration
//	  2)  USB ACM (or UART) configuration
//	  3)  Millisecond tick interrupt
//	  4)  Microsecond delay timer
//	  5)  Time of day clock
//	  6)  SD card interface
//	  7)  Tape interface
//
//	  8)  If SERIAL_DEBUG is defined, initialize UART1
//
//  

static void Init( void) 
{

//  Set system clock.
  
 rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

 DBinit();				// initialize debugger

//  Set up the Systick timer for 1 msec tick.

  Milliseconds = 0;                     // start off ticker
  InitGPIO();				// set up primary GPIOs
  SD_GPIO_Init();			// initialize the SD interface
  SetupSysTick();			// get the milliscond timer going
  DelaySetup();				// set up delay timer

  InitACM(115200);			// initialize USB comm

//  Wait for console check-in.  

  Ugets( (char *) Buffer, 256);         // just clear out any input garbage

  while(1)
  {
    char c;

    Uprintf( "\nPress \'G\' start\n");
 
    c =  Ugetchar();
    if ( c == 'g' || c == 'G')
      break;
  } // wait for a go
    
  Uprintf("\nTape Utility version " VERSION " ready...\n"
          "Enter \"HELP\" for a command description\n");

//  Get the real-time clock going.

  InitializeRTC();

  Uprintf( "\nRTC initialized.\n");

//  Get SDIO going.

  SD_Init();				// initialize SDcard.
  MountSD( 0);				// do it twice
  Uprintf( "SDIO initialized.\n");
  
//  Initialize the tape interface.

  TapeInit();  
  
  Uprintf( "Tape I/O initialized.\n"); 

//  Initialize the UART if serial debugging..

  DBprintf( "Debug I/O ready.\n");

  return;
      
} // Init

