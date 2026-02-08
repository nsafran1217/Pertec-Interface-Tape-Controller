/***********************************************************************
 * FILE: tapeutil.h  (updated signatures)
 ***********************************************************************/
#ifndef _TAPEUTIL_INC
#define _TAPEUTIL_INC

#include <stdint.h>

void HandleStatus(void);
void HandleInit(void);
void HandleRewind(void);
void HandleUnload(void);
void HandleReadForward(uint8_t flags);
void HandleSkip(int16_t count);
void HandleSpace(int16_t count);
void HandleSetAddr(uint8_t addr);
void HandleSetRetries(uint8_t count);
void HandleSetStop(uint8_t mode, uint8_t stopOnError);
void HandleDebug(uint16_t cmd);
void HandleCreateImage(uint8_t flags);
void HandleWriteImage(uint8_t flags);
void HandleSet1600(void);
void HandleSet6250(void);

#endif