I have an application that runs on an STM32 microcontroller, using libopencm3 library. Your goal is to convert the project from using a local CLI interface where the user interacts with it over a USB serial connection ACM and writes data to an SD card, to one that takes commands and reads and writes data over USB to an application running on the host. 

Try to modify existing files relating to tape operations as little as possible. 
I have already removed files relating to functions which will no longer be used.
usbserial.h, usb.c, main.c, comm.c and related headers will need modified. The MCU application will never use USGetchar or related functions. All communication will happen with the host application.

tapeutil.c should have its functions replicated in the host application. logic for interacting with the tape drive and assembling files should remain in tapeutil.c. f_read and f_write calls should be replaced with functions which behave very similarly.

The USB interface must be able to move 100 KB/s to the host.

You must write the host application in python. The application should be readable and simple. Verbose error reporting and logging is a welcome addition.

The github repository is at https://github.com/nsafran1217/Pertec-Interface-Tape-Controller/tree/USB. Use the USB branch. If you must reference origianl implementations, use the main branch.