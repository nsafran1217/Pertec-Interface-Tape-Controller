/*
 * main.c - USB Host Mode Only (No CLI)
 * 
 * Initializes hardware and runs USB command loop.
 */

#define MAIN 1
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "license.h"
#include "gpiodef.h"
#include "globals.h"
#include "miscsubs.h"
#include "usb.h"
#include "dbserial.h"

/* Private function prototypes */
static void Init(void);

int main(void) 
{
    Init();
    DBprintf("Init done\n");
    /* Main loop - just poll USB and process commands */
    while (1)
    {
        USB_Poll();
        USB_ProcessCommands();
    }
    
    return 0;
}

/*
 * Init - Initialization tasks
 */
static void Init(void) 
{
    /* Set system clock */
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    DBinit();
    DBprintf("Debug Term Setup\n");
    /* Set up the Systick timer for 1 msec tick */
    Milliseconds = 0;
    InitGPIO();
    SetupSysTick();
    DelaySetup();
    
    /* Initialize USB - includes tape initialization via TapeUtil_Init() */
    rcc_periph_clock_enable(RCC_OTGFS);
    USB_Init();
    
    
    /* Wait for USB enumeration */
    Delay(100*1000);
}