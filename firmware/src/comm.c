#define _SUPPRESS_PLIB_WARNING
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "license.h"

#include "comm.h"
#include "usbserial.h"
#include "tapeutil.h"

static void Numout( unsigned Num, int Dig, int Radix, int bwz);

//  InitACM - Set up USB as a UART
//  -------------------------------
//
//  Takes an ignored argument for baudrate.
//

void InitACM( int Baudrate)
{
  (void) Baudrate;
  USInit();
  USClear();
} // SetupUART


//  Ugetchar - Get a character from input.
//  --------------------------------------
//
//  Serves as an alias to the USB entry.
//

unsigned char Ugetchar( void)
{
  return USGetchar();
} // Ugetchar

//  Ucharavail - See if a character is ready for input.
//  ---------------------------------------------------
//
//  Serves as an alias to the USB entry.
//

int Ucharavail( void)
{
  return USCharReady();
} // Ucharavail

//  Uputchar - Write a Single Character
//  -----------------------------------
//
//  Just an alias for USPutchar
//

void Uputchar( unsigned char What)
{
  USPutchar( What);
  return;
} // Uputchar

//  Uputs - Put a string to UART.
//  -----------------------------
//
//  Ends with a null.  If a newline is present, adds a CR.
//

void Uputs( char *What)
{

  char
    c;

  while( (c = *What++))
    USPutchar( c);

} // Uputs

//* Uprintf - Simple printf function to UART.
//  -----------------------------------------
//
//    We understand %s and %nx, where n is either null or 1-8
//

void Uprintf( char *Form,...)
{

  const char *p;
  va_list argp;
  int width;
  int i, bwz;
  unsigned u;
  char *s;
  
  va_start(argp, Form);
  
  for ( p = Form; *p; p++)
  {  // go down the format string
    if ( *p != '%')
    { // if this is just a character, echo it
      USPutchar( *p);    
      continue;
    } // ordinary character
    
    ++p;                  // advance
    width = 0;            // assume no fixed width
    bwz = 1;              // say blank when zero
    
    if ( *p == '0')
    {
      p++;
      bwz = 0;            // don't blank when zero
    } // say don't suppress lead zeros
    
    while ( (*p >= '0') && (*p <= '9'))
      width = (width*10) + (*p++ - '0'); // get width of field

    switch (*p)
    {  

      case 'd':
        u = va_arg( argp, unsigned int);
        Numout( u, width, 10, bwz);    // output decimal string
        break;
              
      case 'x':
        u = va_arg( argp, unsigned int);
        Numout( u, width, 16, bwz);     // output hex string
        break;      

      case 'c':
        i = va_arg( argp, int);
        USPutchar(i);
        break;
        
      case 's':
        s = va_arg( argp, char *);
        
//  We have to handle lengths here, so eventually space-pad or truncate.

        if ( !width)
          USPuts(s);            // no width specified, just put it out
        else
        {  
          i = strlen(s) - width;
          if ( i >= 0)
            USPuts( s+i);       // truncate
          else
          {  // if pad
            USPuts(s);
            for ( ; i; i++)
              USPutchar(' ');   // pad  
          }
        } // we have a width specifier
        break;
      
      default:
        break;   
    } // switch
  } // for each character
  va_end( argp);
  return;
} // Uprintf

//  Ugets - Get a string.
//  ---------------------
//
//  Reads a string from input; ends with CR or LF on input
//  Strips the terminal CR or LF; terminal null appended.
//
//  Echo and backspace are also processed.
//
//  On input, address of string, length of string buffer.
//  Returns the address of the null.
//

char *Ugets( char *Buf, int Len)
{

  char c;
  int pos;
 
  for ( pos = 0; Len;)
  {
    c = USGetchar();
    if ( c == '\r' || c == '\n')
      break;
    if ( c == '\b')
    {	// handle backspace.
      if (pos)
      {
      	USPuts( "\b \b");	// backspace-space-backspace
        pos--;
        Buf--;
        Len++;
      } // if not at the start of line
      continue;			// don't store anything
    } // if backspace
    if ( c >= ' ')
    {
      USPutchar( c);		// echo the character;
      *Buf++ = c;		// store character
      pos++;
      Len--;
      if ( !Len)
      	break;
    } // if printable character
  } // for
  *Buf = 0;
  USPuts( "\r\n");		// end with a newline
  return Buf;
} // Ugets


//  ProcessCommand - receive commands from host and dispatch to tape utilities.
//  Expected input: a single-line command with arguments separated by spaces.
//  Examples: "READ image.bin N" or "STATUS"

void ProcessCommand(void)
{
  char line[256];
  char *argv[8];
  int argc;
  int pos;

  for (;;) {
    // read a line
    pos = 0;
    while (pos < (int)sizeof(line)-1) {
      int c = USGetchar();
      if (c == '\r') continue;
      if (c == '\n') break;
      line[pos++] = (char)c;
    }
    line[pos] = '\0';
    if (pos == 0) continue;

    // tokenize by spaces
    argc = 0;
    char *p = line;
    while (*p && argc < (int)(sizeof(argv)/sizeof(argv[0]))) {
      while (*p == ' ') p++;
      if (!*p) break;
      argv[argc++] = p;
      while (*p && *p != ' ') p++;
      if (*p) *p++ = '\0';
    }
    if (argc == 0) continue;

    // uppercase command
    for (char *pc = argv[0]; *pc; ++pc) *pc = (char) toupper((int)*pc);

    // Dispatch
    if (strcmp(argv[0], "STATUS") == 0) {
      CmdShowStatus(NULL);
    } else if (strcmp(argv[0], "REWIND") == 0) {
      CmdRewindTape(NULL);
    } else if (strcmp(argv[0], "READ") == 0) {
      CmdCreateImage(&argv[1]);
    } else if (strcmp(argv[0], "WRITE") == 0) {
      CmdWriteImage(&argv[1]);
    } else if (strcmp(argv[0], "DUMP") == 0) {
      CmdReadForward(&argv[1]);
    } else if (strcmp(argv[0], "INIT") == 0) {
      CmdInitTape(NULL);
    } else if (strcmp(argv[0], "ADDRESS") == 0) {
      CmdSetAddr(&argv[1]);
    } else if (strcmp(argv[0], "SKIP") == 0) {
      CmdSkip(&argv[1]);
    } else if (strcmp(argv[0], "SPACE") == 0) {
      CmdSpace(&argv[1]);
    } else if (strcmp(argv[0], "UNLOAD") == 0) {
      CmdUnloadTape(&argv[1]);
    } else if (strcmp(argv[0], "STOP") == 0) {
      CmdSetStop(&argv[1]);
    } else if (strcmp(argv[0], "DEBUG") == 0) {
      CmdTapeDebug(&argv[1]);
    } else if (strcmp(argv[0], "HELP") == 0) {
      Uprintf("Available: STATUS REWIND READ WRITE DUMP INIT ADDRESS SKIP SPACE UNLOAD STOP DEBUG\n");
    } else {
      Uprintf("Unknown command\n");
    }
  }
}


//  Numout - Output a number in any radix.
//  --------------------------------------
//
//  On entry, Num = number to display, Dig = digits, Radix = radix up to 16.
//  bwz = nonzero if blank when zero.
//  Nothing on return.
//

static void Numout( unsigned Num, int Dig, int Radix, int bwz)
{

  int i;
  const char hexDigit[] = "0123456789abcdef";
  char  outBuf[10];
  
  memset( outBuf, 0, sizeof outBuf);  // zero it all
  if (Radix  == 0 || Radix > 16)
    return;
  
// Use Chinese remainder theorem to develop conversion.

  i = (sizeof( outBuf) - 2); 
  do
  {  
    outBuf[i] = (char) (Num % Radix);
    Num = Num / Radix;
    i--;
   } while (Num);
   
//  If the number of digits is zero, just print the significant number.

  if ( Dig == 0)
    Dig = sizeof( outBuf) - 2 - i;
  for ( i = sizeof( outBuf) - Dig - 1; i < (int) sizeof( outBuf)-1; i++)
  {
    if ( bwz && !outBuf[i] && (i != (sizeof(outBuf)-2)) )
      USPutchar( ' ');
    else
    {
      bwz = 0;  
      USPutchar( hexDigit[ (int) outBuf[i]] );
    }  // not blanking lead zeroes
  }  
} // Numout


//  Hexin - Read a hex number until a non-hex.
//  ------------------------------------------
//
//    On exit, the hex value is stored in RetVal.
//    and the address of the next non-hex digit is returned.
//
//    If no valid digits are encountered, the return value is 0xffffffff;
//
//    Leading spaces are permitted.
//

char *Hexin( unsigned int *RetVal, unsigned int *Digits, char *Buf)
{

  unsigned int
    accum;
  char
    c;
  int
    digCount;

  digCount = 0;				// no digits yet
  accum = 0;				// set null accumulator

//  First, strip off any leading spaces.

  while( *Buf == ' ')
    Buf++;				// skip until non-space

//  Now, pick up digits.

  do
  {
    c = *Buf++;
    c = toupper(c);
    if ( c >= '0' && c <= '9')
      accum = (accum << 4) | (c - '0');
    else if (c >= 'A' && c <= 'F')
      accum = (accum << 4) | (c - 'A' +10);
    else
    {
      Buf--;
      *Digits = digCount;
      *RetVal = accum;
      return Buf;
    }
    digCount++;
  } while(1);
} // Hexin
