#ifndef _MISCSUBS_DEFINED
#define _MISCSUBS_DEFINED 

//void LedCtrl( unsigned int LED, int Func);

void InitGPIO( void);
void DelaySetup (void);
void Delay(uint16_t Howmuch);
void SetupSysTick( void);

#endif // _MISCSUBS_DEFINED