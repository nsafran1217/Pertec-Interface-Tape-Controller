/* Minimal remote filesystem shim for MCU - forwards file ops to host via USB */
#ifndef _REMOTEFS_H
#define _REMOTEFS_H

#include <stdint.h>

typedef int FRESULT;
typedef struct { int handle; } FIL;
typedef unsigned int UINT;

#define FR_OK 0

FRESULT f_open(FIL *f, const char *path, uint32_t mode);
FRESULT f_read(FIL *f, void *buf, UINT btr, UINT *br);
FRESULT f_write(FIL *f, const void *buf, UINT btw, UINT *bw);
FRESULT f_close(FIL *f);

#endif
