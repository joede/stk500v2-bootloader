# stk500v2-bootloader

This project is forked from Peter Fleury's [stk500v2bootloader](http://homepage.hispeed.ch/peterfleury/avr-software.html).
The project implements a *STK500v2 compatible bootloader* for Atmel AVR microcontrolers.

## Why doing a fork?

The original code is not hosted an a platform allowing collaborative work. Since I need some special
features, I decided to place a fork at github to add my features. The original source is placed in the
branch `vendor`. All my changes are done in the branch `develop` and than released into the branch
`master`.

Since the original code has no version number, I use a version number for my changes. This version
starts at 1.0.

## Overview

This program allows an AVR with bootloader capabilities to read/write its own
Flash/EEprom. To enter programming mode, either an input pin is checked (called
"pin mode" here), or the serial port is checked for a defined pattern (called
"forced mode" here). Note: the bootloader would *not leave* if there is *no
application code* found (flash at address 0x0000 is 0xFF)!

**pin mode:** If a defined pin is pulled low, programming mode is entered. If not,
normal execution is done from $0000 "reset" vector in Application area.

**forced mode:** In "forced mode", the bootloader waits up to 3 seconds for a
sequence of 5+ consecutive '*' at the UART. If this '*' sequence is received,
the bootloader starts replying '*' and wait for an empty UART buffer. If no
more data is fetched, the regular bootloader is entered.

In best case, the size of the bootloader is less than 500 words, so it will
fit into a 512 word bootloader section!

## Usage

- Set AVR MCU type and clock-frequency (`F_CPU`) in the `Makefile`.
- Set bootloader start address in bytes (`BOOTLOADER_ADDRESS`) in `Makefile`
  this must match selected "Boot Flash section size" fuses below
- configure the source in `config.h` to fit your application. Set the baud rate
  according to your clock-frequency in order to avoid unstable communication.
  Check the manuals to see the baud rate error.
  (AVRISP only works with 115200 bps)
- compile/link the bootloader with the supplied `Makefile`.
- program the "Boot Flash section size" (BOOTSZ fuses),
  for boot-size 512 words:  program BOOTSZ1
- enable the BOOT Reset Vector (program BOOTRST)
- Upload the hex file to the AVR using any ISP programmer
- Program Boot Lock Mode to save the bootloader against overwriting.
- Reset your AVR while keeping `PROG_PIN` pulled low
- Start AVRISP Programmer (AVRStudio/Tools/Program AVR)
- AVRISP will detect the bootloader
- Program your application FLASH file and optional EEPROM file using AVRISP

## Some more notes

### Entering the bootloader

The bootload is always entered as long as there is no application available
(flash at 0x0000 is 0xFF if no application is flashed). If an application
is available (which is the normal use case), the bootloader would start the
applcation unless the user forces the bootload to enter programming mode.

To force this, this release of stk500boot supports two modes.

1. "pin mode": defines a bootmode pin which must be pressed (low active).
2. "forced mode": wait for a pattern at the UART

### Leaving the bootloader

Normally the bootloader accepts further commands after programming.
The bootloader exits and starts applicaton code after programming
when `ENABLE_LEAVE_BOOTLADER` is defined.

Use Auto Programming mode to programm both flash and eeprom,
otherwise bootloader will exit after flash programming.

### AVRdude

Please uncomment `#define REMOVE_CMD_SPI_MULTI` when using AVRdude.
Comment `#define REMOVE_PROGRAM_LOCK_BIT_SUPPORT` and
`#define REMOVE_READ_LOCK_FUSE_BIT_SUPPORT` to reduce code size. All these
defines are located in `config.h`.

Read Fuse Bits and Read/Write Lock Bits is not supported when using AVRdude!

### Limitations

Erasing the device without flashing, through AVRISP GUI button "Erase Device"
is not implemented, due to AVRStudio limitations.
Flash is always erased before programming.

### Application Notes

* Atmel Application Note AVR109 - Self-programming
* Atmel Application Note AVR068 - STK500v2 Protocol

### License

Copyright (C) 2006 Peter Fleury

Modifications (C) 2016 Jörg Desch

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
