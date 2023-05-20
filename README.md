# zx-spectrum-if1-emulator

## An Interface One / Microdrive Emulation for the ZX Spectrum

This device plugs into the back of a 48K ZX Spectrum and provides the
functionality of the original 1984 "ZX Spectrum Expansion System". It
emulates a ZX Interface One with 8 ZX Microdrives.

The goal is total compatibility with all 1980s software with no ROM
modifications or other hacks.

The first version will support the ZX Microdrives; RS232 and ZX Net
might follow.

Open source, open hardware design.

## Status

As of May 2023 the hardware design is on the second prototype. It looks like
this:

![alt text](images/prototype2.jpg "Prototype 2 board")

Yes, it's enormous, but I'm currently working with the hardware all the
time and having it this big means it's easier to get the soldering iron
between the components. I envisage the final version to be much smaller,
probably with a lay-flat design.

The user interface is currently very fluid. At the moment the hardware
for that consists of an OLED screen and a rotary encoder:

![alt text](images/prototype2_gui.jpg "Prototype 2 board user interface")

The hardware and software have reached what might be termed "Minimum
Viable Product" stage:

* Spectrum can read and write all 8 microdrives
* Full ZX Interface One ROM support, all native BASIC commands and extensions work as designed
* Uses "MDR" image format, supported by all emulators
* MDR files loaded from, and saved back to, SD card

The project is a work in progress.



Derek Fountain, May 2023
