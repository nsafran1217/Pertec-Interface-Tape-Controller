This is the repository for a controller to formatted Pertec tape interface.

The directories are as follows:

	doc - Documentation
	firmware - Controller firmware
	kicad - KiCAD schematics, board layout, etc.

## 2026 Updates
The main branch in this fork contains:  
1. a fix for an off by one error when writing .tap files to a tape
2. A platform.io config file to enable building with platform.io in vscode. The make file should still work

### Problems reading tapes with large blocks
When using this, I found that I could not reliably read tapes which had block sizes over 4k because the SD card writes were too slow.  
The HostUSBOnly branch resolves this by completley changing the controller to use an application running on a host computer to read and write the data. More info is contained in the branches readme.  
[https://github.com/nsafran1217/Pertec-Interface-Tape-Controller/tree/HostUSBOnly](https://github.com/nsafran1217/Pertec-Interface-Tape-Controller/tree/HostUSBOnly)



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

