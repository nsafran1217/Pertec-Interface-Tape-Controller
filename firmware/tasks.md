# Overview
I have an application that runs on an STM32 microcontroller, using libopencm3 library. Your goal is to convert the project from using a local CLI interface where the user interacts with it over a USB serial connection ACM and writes data to an SD card, to one that takes commands and reads and writes data over USB to an application running on the host. 

Try to modify existing files relating to tape operations as little as possible. 
usbserial.h, usb.c, main.c, comm.c and related headers will need modified. The MCU application will never use USGetchar or related functions. All communication will happen with the host application.