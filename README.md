# ZX Spectrum Expansion System, Recreated

## An Interface One / Microdrive Emulation for the ZX Spectrum

This device plugs into the back of a 48K ZX Spectrum and provides the
functionality of the original 1984 "ZX Spectrum Expansion System". It
emulates a ZX Interface One with 8 ZX Microdrives.

The goal is total compatibility with all 1980s software with no ROM
modifications or other hacks.

The first version supports the ZX Microdrives; RS232 and ZX Net might follow.

Open source, open hardware design.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/kph2aS7qcVc/0.jpg)](http://www.youtube.com/watch?v=kph2aS7qcVc "ZX Spectrum Expansion System, Recreated")

https://youtube.com/shorts/kph2aS7qcVc?feature=share

## Status

As of July 2023 version 1.0 is complet:

![alt text](images/zses_2.jpg "v1.0")

It is functionally complete:

* Spectrum can read and write all 8 microdrives
* Uses "MDR" image format for "cartridges", fully compatible with emulators
* Full ZX Interface One ROM support, all native BASIC commands and extensions work as designed
* Bug for bug compatible
* MDR files loaded from, and saved back to, SD card
* User interface uses file selector to choose and "insert" a cartridge
* Cartridge eject works, allowing cartridges to be changed as required
* Configuration file allows auto-insertion of selected cartridges at start up

Derek Fountain, July 2023
