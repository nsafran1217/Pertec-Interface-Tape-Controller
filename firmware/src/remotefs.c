/* Remote filesystem implementation - communicates with host over USB CDC.
   Protocol (ASCII line-based control):
     MCU -> HOST: FSOPEN|R|filename\n  (or W for write)
       HOST -> MCU: OK|<handle>\n  or ERR|<code>\n
     MCU -> HOST: FSREAD|<handle>|<len>\n
       HOST -> MCU: <len> bytes raw

     MCU -> HOST: FSWRITE|<handle>|<len>\n  followed by <len> bytes raw
       HOST -> MCU: OK\n or ERR|<code>\n
     MCU -> HOST: FSCLOSE|<handle>\n  -> OK\n

   This is a blocking, simple implementation intended only for the tape
   image transfers in `tapeutil.c`.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "usbserial.h"
#include "remotefs.h"

/* Simple line send/recv helpers */
static void send_line(const char *s)
{
  USPuts((char *)s);
}

static int read_line(char *buf, int maxlen)
{
  int i = 0;
  int c;
  while (i + 1 < maxlen) {
    c = USGetchar();
    if (c == '\r') continue;
    if (c == '\n') break;
    buf[i++] = (char)c;
  }
  buf[i] = '\0';
  return i;
}

/* Parse integer from reply token */
static int parse_handle(const char *s)
{
  return atoi(s);
}

FRESULT f_open(FIL *f, const char *path, uint32_t mode)
{
  char line[256];
  char cmd[16];
  /* mode: simple: read -> 'R', write -> 'W' */
  const char *m = (mode & 0x01) ? "W" : "R"; /* caller decides convention */
  snprintf(line, sizeof(line), "FSOPEN|%s|%s\n", m, path);
  send_line(line);
  /* expect reply */
  read_line(line, sizeof(line));
  if (strncmp(line, "OK|", 3) == 0) {
    f->handle = parse_handle(line + 3);
    return FR_OK;
  }
  return -1;
}

FRESULT f_read(FIL *f, void *buf, UINT btr, UINT *br)
{
  char line[64];
  int i;
  snprintf(line, sizeof(line), "FSREAD|%d|%u\n", f->handle, (unsigned)btr);
  send_line(line);
  /* read exactly btr bytes into buf */
  i = 0;
  while ((unsigned)i < btr) {
    int c = USGetchar();
    if (c < 0) continue;
    ((unsigned char *)buf)[i++] = (unsigned char)c;
  }
  if (br) *br = i;
  return FR_OK;
}

FRESULT f_write(FIL *f, const void *buf, UINT btw, UINT *bw)
{
  char line[64];
  snprintf(line, sizeof(line), "FSWRITE|%d|%u\n", f->handle, (unsigned)btw);
  send_line(line);
  USWriteBlock((uint8_t *)buf, btw);
  /* wait for OK/ERR response */
  read_line(line, sizeof(line));
  if (strncmp(line, "OK", 2) == 0) {
    if (bw) *bw = btw;
    return FR_OK;
  }
  return -1;
}

FRESULT f_close(FIL *f)
{
  char line[64];
  snprintf(line, sizeof(line), "FSCLOSE|%d\n", f->handle);
  send_line(line);
  read_line(line, sizeof(line));
  if (strncmp(line, "OK", 2) == 0) return FR_OK;
  return -1;
}
