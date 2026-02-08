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
#include "comm.h"
#include "gpiodef.h"
#include "globals.h"
#include "miscsubs.h"
#include "tapedriver.h"
#include "tapeutil.h"
#include "dbserial.h"

/* ------------------------------------------------------------------ */
/*  Command table                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    void (*handler)(char *args[]);
} CMD_ENTRY;

static CMD_ENTRY cmd_table[] = {
    { "status",   CmdShowStatus   },
    { "brief",    (void(*)(char**))ShowBriefStatus },
    { "rewind",   CmdRewindTape   },
    { "unload",   CmdUnloadTape   },
    { "read",     CmdReadForward  },
    { "skip",     CmdSkip         },
    { "space",    CmdSpace        },
    { "create",   CmdCreateImage  },
    { "write",    CmdWriteImage   },
    { "addr",     CmdSetAddr      },
    { "retries",  CmdSetRetries   },
    { "stop",     CmdSetStop      },
    { "1600",     CmdSet1600      },
    { "6250",     CmdSet6250      },
    { "init",     CmdInitTape     },
    { "debug",    CmdTapeDebug    },
    { NULL, NULL }
};

/* ------------------------------------------------------------------ */
/*  Command parser                                                     */
/* ------------------------------------------------------------------ */

#define MAX_ARGS 8

static void DispatchCommand(char *line)
{
    char *args[MAX_ARGS + 1];
    char *tok;
    int   argc = 0;

    /* Tokenize by spaces */
    tok = strtok(line, " \t\r\n");
    if (!tok) return;

    /* Find command */
    char *cmd = tok;
    tok = strtok(NULL, " \t\r\n");
    while (tok && argc < MAX_ARGS) {
        args[argc++] = tok;
        tok = strtok(NULL, " \t\r\n");
    }
    args[argc] = NULL;

    /* Look up in table */
    for (int i = 0; cmd_table[i].name; i++) {
        if (strcasecmp(cmd, cmd_table[i].name) == 0) {
            cmd_table[i].handler(args);
            return;
        }
    }
    Uprintf("Unknown command: %s\n", cmd);
}

/* ------------------------------------------------------------------ */
/*  Protocol command loop – replaces old ProcessCommand()               */
/* ------------------------------------------------------------------ */

static void ProcessCommand(void)
{
    char     cmd_buf[512];
    uint8_t  pkt_type;
    uint16_t pkt_len;

    while (1) {
        SendPacket(PKT_READY, NULL, 0);

        if (RecvPacket(&pkt_type, (uint8_t *)cmd_buf, &pkt_len) < 0)
            continue;

        if (pkt_type != PKT_CMD || pkt_len == 0)
            continue;

        cmd_buf[pkt_len] = '\0';
        DispatchCommand(cmd_buf);
        SendPacket(PKT_DONE, NULL, 0);
    }
}

/* ------------------------------------------------------------------ */
/*  Initialization                                                     */
/* ------------------------------------------------------------------ */

static void Init(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    DBinit();
    Milliseconds = 0;
    InitGPIO();
    SetupSysTick();
    DelaySetup();
    InitACM(115200);
    TapeInit();
    DBprintf("Firmware ready.\n");
}

int main(void)
{
    Init();
    ProcessCommand();   /* never returns */
    return 0;
}