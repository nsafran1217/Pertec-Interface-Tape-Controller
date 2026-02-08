/***********************************************************************
 * FILE: hostcomm.h
 * Host communication layer - packet send/receive and stream I/O.
 ***********************************************************************/
#ifndef _HOSTCOMM_H
#define _HOSTCOMM_H

#include <stdint.h>
#include <stdbool.h>

void HostCommInit(void);

/* --- Packet I/O --- */
void PktSend(uint8_t type, const uint8_t *data, uint32_t len);
int  PktRecv(uint8_t *type, uint8_t *data, uint32_t maxlen, uint32_t *len);
void PktRecvExact(uint8_t *buf, uint32_t count);

/* --- Convenience senders --- */
void SendOK(const uint8_t *data, uint32_t len);
void SendError(uint16_t code, const char *msg);
void SendMsg(const char *msg);
void SendMsgF(const char *fmt, ...);
void SendStatus(uint16_t raw, uint32_t position);
void SendImgDone(uint32_t blocks, uint32_t files, uint32_t bytes, uint8_t aborted);

/* --- Stream I/O (replaces FatFS f_read / f_write) --- */
void StreamReset(void);
int  StreamWrite(const void *data, uint32_t count, uint32_t *written);
int  StreamRead(void *data, uint32_t count, uint32_t *bytesRead);

/* --- Abort check (replaces CheckForEscape) --- */
bool CheckAbort(void);

#endif /* _HOSTCOMM_H */
