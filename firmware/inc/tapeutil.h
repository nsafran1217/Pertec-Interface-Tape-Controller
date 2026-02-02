/*
 * tapeutil.h - Tape Utilities (USB Host Mode)
 */

#ifndef TAPEUTIL_H
#define TAPEUTIL_H

#include <stdint.h>

/* Command handlers - called from main loop based on host commands */
void GetTapeStatus(void);
void DoRewind(void);
void DoUnload(void);
void DoSkip(const uint8_t *params);
void DoSpace(const uint8_t *params);
void DoReadBlock(void);
void DoSetDensity(const uint8_t *params);
void DoCreateImage(const uint8_t *params);
void DoWriteImage(const uint8_t *params);

#endif /* TAPEUTIL_H */