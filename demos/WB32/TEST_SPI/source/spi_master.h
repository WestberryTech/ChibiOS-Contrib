#ifndef __SPI_H_
#define __SPI_H_


/* Copyright 2020 Nick Brassel (tzarc)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <ch.h>
#include <hal.h>
#include <stddef.h>
#include <stdbool.h>
#include "wb32f3g71xx.h"

#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN PAL_LINE(GPIOA, 5)
#define SPI_MOSI_PIN PAL_LINE(GPIOA, 7)
#define SPI_MISO_PIN PAL_LINE(GPIOA, 6)
#define SPI_MISO_PAL_MODE 5
#define SPI_MOSI_PAL_MODE 5
#define SPI_SCK_PAL_MODE 5
//#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN PAL_LINE(GPIOB, 14)

typedef int16_t spi_status_t;

#define SPI_STATUS_SUCCESS (0)
#define SPI_STATUS_ERROR (-1)
#define SPI_STATUS_TIMEOUT (-2)

#define SPI_TIMEOUT_IMMEDIATE (0)
#define SPI_TIMEOUT_INFINITE (0xFFFF)

typedef ioline_t pin_t;

/* Operation of GPIO by pin. */

#define setPinInput(pin) palSetLineMode(pin, PAL_MODE_INPUT)
#define setPinInputHigh(pin) palSetLineMode(pin, PAL_MODE_INPUT_PULLUP)
#define setPinInputLow(pin) palSetLineMode(pin, PAL_MODE_INPUT_PULLDOWN)
#define setPinOutput(pin) palSetLineMode(pin, PAL_MODE_OUTPUT_PUSHPULL)

#define writePinHigh(pin) palSetLine(pin)
#define writePinLow(pin) palClearLine(pin)
#define writePin(pin, level) ((level) ? (writePinHigh(pin)) : (writePinLow(pin)))

#define readPin(pin) palReadLine(pin)

#define togglePin(pin) palToggleLine(pin)

/* Operation of GPIO by port. */

typedef uint16_t port_data_t;

#define readPort(pin) palReadPort(PAL_PORT(pin))

#define setPortBitInput(pin, bit) palSetPadMode(PAL_PORT(pin), bit, PAL_MODE_INPUT)
#define setPortBitInputHigh(pin, bit) palSetPadMode(PAL_PORT(pin), bit, PAL_MODE_INPUT_PULLUP)
#define setPortBitInputLow(pin, bit) palSetPadMode(PAL_PORT(pin), bit, PAL_MODE_INPUT_PULLDOWN)
#define setPortBitOutput(pin, bit) palSetPadMode(PAL_PORT(pin), bit, PAL_MODE_OUTPUT_PUSHPULL)

#define writePortBitLow(pin, bit) palClearLine(PAL_LINE(PAL_PORT(pin), bit))
#define writePortBitHigh(pin, bit) palSetLine(PAL_LINE(PAL_PORT(pin), bit))

#ifdef __cplusplus
extern "C" {
#endif
void spi_init(void);

bool spi_start(pin_t slavePin, bool lsbFirst, uint8_t mode, uint16_t divisor);

spi_status_t spi_write(uint8_t data);

spi_status_t spi_read(void);

spi_status_t spi_transmit(const uint8_t *data, uint16_t length);

spi_status_t spi_receive(uint8_t *data, uint16_t length);

void spi_stop(void);
#ifdef __cplusplus
}
#endif


#endif
