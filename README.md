# RS41HUP (Ham Use Project) - Project Horus Fork
Firmware for RS41 for HAM use.

It is possible to recycle RS41-SGP sondes for amateur radio use without any electrical changes! You just have to build a new firmware (this one) and apply it via a cheap programmer, the "ST-Linkv2" (or alternatives). The modified sonde can now transmit on a user-defined frequency in the 70cm band, with Habhub-compatible RTTY and (better performing) 4FSK telemetry!

Released under GPL v2.

# Important info
* This is a fork from  github.com/darksidelemm/RS41HUP  (not longer supported)
* Major adds:
  * Horus V2 protocoll (32 bytes) 
  * GPS-Watchdog: reboots RS41 if GPS gets lost longer then timeout as defined
  * GPS-TX Sync if Fix is available every minute
  * Second frequency if defined. changes every TX intervall between both

* This RS41HUB is recomended for floating flights with battery. It needs less mA then the RS41ng Version. 
* If power does not matter, than have a look on RS41ng.
* RS41ng is created by https://github.com/mikaelnousiainen/RS41ng.
* An Version with some special needs will also be found on https://github.com/whallmann/RS41ng


Original Repository: https://github.com/Qyon/STM32_RTTY, though this fork is based on [DF8OE's version](https://github.com/df8oe/RS41HUP).

Modifications by Mark Jessop <vk5qi@rfhead.net> include:
* Compatability with existing Project Horus RTTY Formats.
* Removed APRS support - no 70cm APRS infrastructure in Australia, so not really useful to us.
* Addition of the 4FSK 'Horus Binary' high performance telemetry mode
  * A decoder for the 4FSK mode is available here: https://github.com/projecthorus/horusdemodlib
  * Information on the 4FSK mode's performance is available here: https://www.rowetel.com/?p=5906


# Compilation
## Linux / OSX:  (Remark: this forked info, wont work in my workspace - i found no solution)
* Grab the latest GNU ARM Embedded toolchain from here: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
* Extract the tarball to somewhere useful. In my case I'm using ~/opt/
* Within the RS41HUP directory:
  * Edit CMakeLists.txt and set the correct path to the un-tar'd directory.
  * `cmake .`
  * `make`


## Windows:
(Note, is likely broken - currently targeting Linux / OSX builds)
I used to edit and compile the CooCox CoIDE - this works fine and it also uploads to the STM32 Board with ST-Link dongle.

Use:
https://www.softpedia.com/get/Programming/Coding-languages-Compilers/CooCox-CoIDE.shtml

And:
https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q3-update/+download/gcc-arm-none-eabi-5_4-2016q3-20160926-win32.exe

# Programming
Either:
* Use the ST Micro ST-LINK utility (windows only it seems?), or
* [stlink](https://github.com/texane/stlink) under Linux/OSX (will need to be unlocked first), or
* [OpenOCD](http://openocd.org) on Linux / RaspberryPi (see openocd_rs41.cfg file for usage) or
* Use `flash.sh` with a [Black Magic Probe](https://1bitsquared.com/products/black-magic-probe). You will need to modify the path to the debugger's serial interface.

Refer to [this file](./docs/programming_header.md) for programming header pinouts.

# Configuration
Configuration settings are located in [config.h](./config.h). Modify as appropriate before compiling/programming.

#Changelog
 * 26.12.2023 - Added choice for 2nd TX frequency: if activated, changes between each tx intervall.
 * 26.12.2023 - Added GPS-Sync: TX starts every full minute if gps-fix is availble
 * 23.12.2023 - Added a GPS-Watchdog: set a GPS-timeout after this the cpu makes a restart (GPS-jamming)
 * 23.12.2023 - Added Horus V2 32-Byte Format. Set by compiler switch in config.h
 * 14.12.2016 - Reverse engineeded connections, initial hard work, resulting in working RTTY by SQ7FJB
 * 07.01.2017 - GPS now using proprietiary UBLOX protocol, more elastic code to set working frequency by SQ5RWU
 * 23.01.2017 - Test APRS code, small fixes in GPS code by SQ5RWU
 * 06.06.2017 - APRS code fix, some code cleanup
 * June 2017 - starting with Linux support, making configuration more flexible by DF8OE
 * March 2018 - Addition of 4FSK binary mode support by Mark VK5QI


