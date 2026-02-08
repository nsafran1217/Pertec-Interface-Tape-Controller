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
#include "usbserial.h"
#include "gpiodef.h"
#include "globals.h"
#include "miscsubs.h"
#include "tapedriver.h"
#include "tapeutil.h"
#include "dbserial.h"

/* Receive buffer for command payloads (small – max command payload) */
static uint8_t CmdBuf[64];

static void Init(void);
static void ProcessCommands(void);

int main(void)
{
    Init();
    ProcessCommands();   /* never returns */
    return 0;
}

static void Init(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    DBinit();

    Milliseconds = 0;
    InitGPIO();
    SetupSysTick();
    DelaySetup();

    HostCommInit();      /* USB CDC init + protocol layer */

    TapeInit();


    DBprintf("Tape controller v" VERSION " ready\n");
}

static void ProcessCommands(void)
{
    uint8_t type;
    uint32_t len;

    while (1) {
        if (PktRecv(&type, CmdBuf, sizeof(CmdBuf), &len) != 0) {
            SendError(ERR_INVALID, "Packet too large");
            continue;
        }

        switch (type) {

        case CMD_PING:
            PktSend(RSP_PONG, NULL, 0);
            break;

        case CMD_STATUS:
            HandleStatus();
            break;

        case CMD_INIT:
            HandleInit();
            break;

        case CMD_REWIND:
            HandleRewind();
            break;

        case CMD_UNLOAD:
            HandleUnload();
            break;

        case CMD_READ_FWD:
            HandleReadForward(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_SKIP: {
            int16_t cnt = 1;
            if (len >= 2)
                cnt = (int16_t)((uint16_t)CmdBuf[0] | ((uint16_t)CmdBuf[1] << 8));
            HandleSkip(cnt);
            break;
        }

        case CMD_SPACE: {
            int16_t cnt = 1;
            if (len >= 2)
                cnt = (int16_t)((uint16_t)CmdBuf[0] | ((uint16_t)CmdBuf[1] << 8));
            HandleSpace(cnt);
            break;
        }

        case CMD_SET_ADDR:
            HandleSetAddr(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_SET_STOP:
            HandleSetStop(
                len >= 1 ? CmdBuf[0] : 2,
                len >= 2 ? CmdBuf[1] : 0);
            break;

        case CMD_SET_RETRIES:
            HandleSetRetries(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_DEBUG: {
            uint16_t cmd = 0;
            if (len >= 2)
                cmd = (uint16_t)CmdBuf[0] | ((uint16_t)CmdBuf[1] << 8);
            HandleDebug(cmd);
            break;
        }

        case CMD_SET_1600:
            HandleSet1600();
            break;

        case CMD_SET_6250:
            HandleSet6250();
            break;

        case CMD_CREATE_IMG:
            HandleCreateImage(len >= 1 ? CmdBuf[0] : 0);
            break;

        case CMD_WRITE_IMG:
            HandleWriteImage(len >= 1 ? CmdBuf[0] : 0);
            break;

        default:
            SendError(ERR_INVALID, "Unknown command");
            break;
        }
    }
}