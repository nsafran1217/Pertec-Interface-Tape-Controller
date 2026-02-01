#ifndef HOSTSTREAM_H
#define HOSTSTREAM_H

#include <stdint.h>
#include <stdbool.h>

bool HostStreamInit(void);
void HostStreamWrite(const uint8_t *buf, int len);
void HostStreamClose(void);

#endif
