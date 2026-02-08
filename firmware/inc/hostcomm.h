#ifndef HOSTCOMM_H
#define HOSTCOMM_H

#include <stdint.h>
#include <stdbool.h>

/*  Protocol packet layer above raw USB.
 *
 *  SendPacket      – blocking, returns when data is queued to USB.
 *  RecvPacket      – blocking, waits until a complete packet arrives.
 *  TryRecvPacket   – non-blocking with timeout (milliseconds).
 *                    Returns true if a packet was received.
 *  PollEscape      – non-blocking, returns true if PKT_ESCAPE received.
 */

void  SendPacket(uint8_t Type, const uint8_t *Payload, uint16_t Len);
int   RecvPacket(uint8_t *Type, uint8_t *Payload, uint16_t *Len);
bool  TryRecvPacket(uint8_t *Type, uint8_t *Payload, uint16_t *Len,
                    uint32_t TimeoutMs);
bool  PollEscape(void);

#endif /* HOSTCOMM_H */