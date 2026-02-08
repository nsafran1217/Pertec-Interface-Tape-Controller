#ifndef HOSTIO_H
#define HOSTIO_H

/*  Drop-in replacements for the FatFS calls used in tapeutil.c.
 *  Backed by USB transfers to the host application.
 */

#include <stdint.h>

typedef int FRESULT;
#define FR_OK             0
#define FR_DISK_ERR       1
#define FR_NOT_READY      3
#define FR_NO_FILE        4

typedef unsigned int UINT;

/* Flags compatible with FatFS usage in tapeutil.c */
#define FA_READ           0x01
#define FA_WRITE          0x02
#define FA_CREATE_ALWAYS  0x08

typedef struct {
    int dummy;           /* opaque; single-file-at-a-time */
} FIL;

FRESULT f_open  (FIL *fp, const char *path, uint8_t mode);
FRESULT f_read  (FIL *fp, void *buf, UINT btr, UINT *br);
FRESULT f_write (FIL *fp, const void *buf, UINT btw, UINT *bw);
FRESULT f_close (FIL *fp);

/* Stub – tapeutil.c calls ShowRTCTime() which we no longer need. */
static inline void ShowRTCTime(void) {}

#endif /* HOSTIO_H */