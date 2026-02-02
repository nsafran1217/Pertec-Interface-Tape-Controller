/*
 * main.c - USB Host Mode
 * 
 * Receives commands from host and dispatches to tapeutil.c handlers.
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
#include "tapedriver.h"
#include "tapeutil.h"
#include "usb.h"

static void Init(void);
static void ProcessHostCommand(void);

int main(void) 
{
    Init();
    
    /* Main loop */
    while (1) {
        USB_Poll();
        ProcessHostCommand();
    }
    
    return 0;
}

/*
 * ProcessHostCommand - Check for and handle host commands
 */
static void ProcessHostCommand(void)
{
    uint8_t params[16];
    uint16_t param_len = sizeof(params);
    
    uint8_t cmd = USB_CheckCommand(params, &param_len);
    
    if (cmd == 0) return;  /* No command */
    
    switch (cmd) {
        case HOST_CMD_NOP:
            USB_SendResponse(RESP_OK);
            break;
            
        case HOST_CMD_STATUS:
            GetTapeStatus();
            break;
            
        case HOST_CMD_REWIND:
            DoRewind();
            break;
            
        case HOST_CMD_UNLOAD:
            DoUnload();
            break;
            
        case HOST_CMD_SKIP:
            DoSkip(params);
            break;
            
        case HOST_CMD_SPACE:
            DoSpace(params);
            break;
            
        case HOST_CMD_READ_BLOCK:
            DoReadBlock();
            break;
            
        case HOST_CMD_SET_DENSITY:
            DoSetDensity(params);
            break;
            
        case HOST_CMD_CREATE_IMAGE:
            DoCreateImage(params);
            break;
            
        case HOST_CMD_WRITE_IMAGE:
            DoWriteImage(params);
            break;
            
        default:
            USB_SendResponse(RESP_ERR);
            break;
    }
}

/*
 * Init - Hardware initialization
 */
static void Init(void) 
{
    /* Set system clock */
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    /* Set up the Systick timer for 1 msec tick */
    Milliseconds = 0;
    InitGPIO();
    SetupSysTick();
    DelaySetup();
    
    /* Initialize USB CDC */
    rcc_periph_clock_enable(RCC_OTGFS);
    USB_Init();
    
    /* Initialize tape interface */
    TapeInit();
    
    /* Wait for USB enumeration */
    Delay(100*1000);
}