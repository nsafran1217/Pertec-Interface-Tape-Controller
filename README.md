# Host Based control only branch

This branch contains a version of the pertec controller which eliminates the local CLI and SD card functions and is controlled by a python script running on the host. 

When using the orignal version that reads and writes from an SD card, I found that I could not read tapes with larger than 4k blocks without the tape drive (Qualstar 1260 with no buffer) repositioning after every block. Attempts to speed up writes to the SD failed, so I decided to move to a model where the MCU would read and write data directly from the computer over USB.


A compiled Binary has been provided in `firmware/bin/target.elf`. The Makefile has been updated. I use Platform.io to develop and upload the firmware to the MCU board with an ST-Link.

### Usage:
In the tools directory is tape_host.py. This scirpt will interact with the controller board. It should autodetect which serial port to use, but one can be provided if needed. Help functions is included in the script.

#### Examples:


`./tape_host.py read ~/test_tape.tap -n`  
Create .tap file from tape, don't rewind

`./tape_hist.py write ~/unixv7.tap`  
Write unixv7.tap to the tape, rewind after complete

`./tape_host.py unload`  
Unload and offline the drive


There have been major changes to this program. If the orignal version was working for you, then I recommend continuing to use the orignal. This version works for me and was tested up to 10k block sizes, both reading and writing. 

*An LLM chat tool was used in the making of this version.*  
*This has been written for and only tested on Linux.*

### Orig Readme:
This is the repository for a controller to formatted Pertec tape interface.

The directories are as follows:

	doc - Documentation
	firmware - Controller firmware
	kicad - KiCAD schematics, board layout, etc.

The firmware can be built from source using the gcc-arm-none-eabi compiler
and the libopencm3 development library (libopencm3.org) which is covered by
the GNU General Public License v3.0, as is this project.

UPDATE APRIL 2024
-----------------

1. Some STM32F4VE MCU boards include a pullup resistor on USB pin D+; others
do not.  The latter do not work with the old code.  Changes have been made
to the usb driver to work with either type of board.

2. A "move" command has been added to facilitate renaming and moving files
and directories.
 
3. The "DUMP" command includes an optional "E" argument, which causes the
display to be EBCDIC rather than ASCII.
 
4. If USB operation is selected, the onboard SD card can be mounted on the
host system as a FAT32 device.  This facilitates file transfer.  Note that
this feature has been tested only on Linux. 
 
5. Observe that if the capacity of the SD card is more than 32GB, that the
"current directory" functions may not operate correctly.  This is a
limitation in the FATFS file manager implementation and may be updated in
the future.

