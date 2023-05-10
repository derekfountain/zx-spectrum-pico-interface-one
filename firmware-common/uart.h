/*
 * ZX Pico IF1 Firmware, a Raspberry Pi Pico based ZX Interface One emulator
 * Copyright (C) 2023 Derek Fountain
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __UART_H
#define __UART_H

/*
 * UI Pico and IO Pico talk using UARTs. The UI one needs to send the MDR data from
 * the SD card to the IO Pico which delivers it to the ZX. The IO Pico sends the
 * MDR data back when it's been changed and needs saving back to SD card.
 *
 * Plus, there's a command interface for things like ejecting, setting write protect, etc.
 */

#include "hardware/uart.h"

/* 921600 is as fast as it goes. 135KB goes over in about 1.5secs */
#define PICOS_BAUD_RATE     921600
#define PICOS_DATA_BITS     8
#define PICOS_STOP_BITS     1
#define PICOS_PARITY        UART_PARITY_NONE

#define UI_PICO_UART_TX_PIN 0
#define UI_PICO_UART_RX_PIN 1
#define UI_PICO_UART_ID     uart0

#define IO_PICO_UART_TX_PIN 12
#define IO_PICO_UART_RX_PIN 13
#define IO_PICO_UART_ID     uart0

#endif
