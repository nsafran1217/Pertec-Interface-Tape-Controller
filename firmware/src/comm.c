#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "license.h"
#include "comm.h"
#include "protocol.h"
#include "hostcomm.h"
#include "usbserial.h"

/* ------------------------------------------------------------------ */
/*  Message buffer – Uprintf accumulates here, flushes at return       */
/* ------------------------------------------------------------------ */

#define MSG_BUF_SZ 4096
static char  msg_buf[MSG_BUF_SZ];
static int   msg_pos = 0;

static void msg_flush(void)
{
    if (msg_pos > 0) {
        SendPacket(PKT_MSG, (const uint8_t *)msg_buf, (uint16_t)msg_pos);
        msg_pos = 0;
    }
}

static void msg_putc(char c)
{
    if (c == '\n') {
        if (msg_pos < MSG_BUF_SZ) msg_buf[msg_pos++] = '\r';
    }
    if (msg_pos < MSG_BUF_SZ) msg_buf[msg_pos++] = c;
    if (msg_pos >= MSG_BUF_SZ - 2) msg_flush();
}

static void msg_puts(const char *s)
{
    while (*s) msg_putc(*s++);
}

/* ------------------------------------------------------------------ */
/*  Numout – identical logic to original, targeting msg_buf            */
/* ------------------------------------------------------------------ */

static void Numout(unsigned Num, int Dig, int Radix, int bwz)
{
    const char hex[] = "0123456789abcdef";
    char out[10];
    int i;
    memset(out, 0, sizeof out);
    if (Radix == 0 || Radix > 16) return;

    i = sizeof(out) - 2;
    do {
        out[i] = (char)(Num % Radix);
        Num /= Radix;
        i--;
    } while (Num);

    if (Dig == 0) Dig = sizeof(out) - 2 - i;
    for (i = sizeof(out) - Dig - 1; i < (int)sizeof(out) - 1; i++) {
        if (bwz && !out[i] && (i != (int)sizeof(out) - 2))
            msg_putc(' ');
        else { bwz = 0; msg_putc(hex[(int)out[i]]); }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void InitACM(int Baudrate)
{
    (void)Baudrate;
    USInit();
    USClear();
}

/*  Ucharavail / Ugetchar – used by CheckForEscape in tapeutil.c.
 *  We map them to the protocol escape mechanism.  */

static volatile bool esc_pending = false;

int Ucharavail(void)
{
    if (PollEscape()) esc_pending = true;
    return esc_pending ? 1 : 0;
}

unsigned char Ugetchar(void)
{
    if (esc_pending) { esc_pending = false; return '\033'; }
    /* Blocking wait – shouldn't be needed in normal operation */
    while (!Ucharavail()) {}
    esc_pending = false;
    return '\033';
}

void Uputchar(unsigned char What)
{
    msg_putc((char)What);
    msg_flush();
}

void Uputs(char *What)
{
    msg_puts(What);
    msg_flush();
}

void Uprintf(char *Form, ...)
{
    const char *p;
    va_list ap;
    int width, bwz;
    unsigned u;
    char *s;
    int i;

    va_start(ap, Form);
    for (p = Form; *p; p++) {
        if (*p != '%') { msg_putc(*p); continue; }
        ++p;
        width = 0; bwz = 1;
        if (*p == '0') { p++; bwz = 0; }
        while (*p >= '0' && *p <= '9')
            width = width * 10 + (*p++ - '0');
        switch (*p) {
        case 'd':
            u = va_arg(ap, unsigned);
            Numout(u, width, 10, bwz);
            break;
        case 'x':
            u = va_arg(ap, unsigned);
            Numout(u, width, 16, bwz);
            break;
        case 'c':
            i = va_arg(ap, int);
            msg_putc((char)i);
            break;
        case 's':
            s = va_arg(ap, char *);
            if (!width) { msg_puts(s); }
            else {
                i = (int)strlen(s) - width;
                if (i >= 0) msg_puts(s + i);
                else { msg_puts(s); for (; i; i++) msg_putc(' '); }
            }
            break;
        default: break;
        }
    }
    va_end(ap);
    msg_flush();
}

char *Ugets(char *Buf, int Len)
{
    /* Send prompt-request, then wait for PKT_INPUT from host. */
    msg_flush();                      /* flush any pending prompt text */
    SendPacket(PKT_INPUT_REQ, NULL, 0);

    uint8_t  type;
    uint16_t rlen;
    uint8_t  tmp[256];
    RecvPacket(&type, tmp, &rlen);

    if (type == PKT_INPUT && rlen > 0) {
        int copy = (rlen < (uint16_t)(Len - 1)) ? rlen : Len - 1;
        memcpy(Buf, tmp, copy);
        Buf[copy] = '\0';
        return Buf + copy;
    }
    *Buf = '\0';
    return Buf;
}

char *Hexin(unsigned int *RetVal, unsigned int *Digits, char *Buf)
{
    unsigned accum = 0;
    int dc = 0;
    char c;
    while (*Buf == ' ') Buf++;
    do {
        c = *Buf++;
        c = toupper(c);
        if (c >= '0' && c <= '9')       accum = (accum << 4) | (c - '0');
        else if (c >= 'A' && c <= 'F')  accum = (accum << 4) | (c - 'A' + 10);
        else { Buf--; *Digits = dc; *RetVal = accum; return Buf; }
        dc++;
    } while (1);
}