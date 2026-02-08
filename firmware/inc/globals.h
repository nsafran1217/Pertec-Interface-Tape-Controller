#ifndef GLOBAL_H_DEFINED
#define GLOBAL_H_DEFINED 1

#define VERSION "1.1"

#ifndef MAIN
#define SCOPE extern
#else
#define SCOPE
#endif


#define NO_OP {}          // avoids misplaced semicolons

//  Milliseconds since boot.

SCOPE volatile uint32_t Milliseconds;      // Just keeps counting


//	Tape buffer - 32K bytes.

#define TAPE_BUFFER_SIZE 32768

SCOPE uint8_t __attribute__ ((aligned(4))) 
    TapeBuffer[TAPE_BUFFER_SIZE];

SCOPE int 
  TapeRetries;			// how many retries on tape reads?

SCOPE int
  StopTapemarks;		// stop at how many consecutivetapemarks?
SCOPE bool
  StopAfterError;		// true if stopping after error

SCOPE uint16_t
  TapeAddress;			// address of tape drive

#undef SCOPE
#endif
