/**
 * \file            comm.c
 * \brief           UART communication for STM32H735G-DK
 */

/*
 * Copyright (c) 2020 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Detailed implementation is well described at
 * https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 */
#include <stdint.h>
#include <string.h>
#include "comm.h"
#include "lwrb/lwrb.h"

#include "stm32f7xx.h"
#include "stm32f7xx_ll_rcc.h"

/**
 * \brief           Start transmit to send data
 * \return          `1` if just started, `0` if still on-going or no data to transmit
 */
uint8_t
comm_start_transfer(void) {
    uint8_t started = 0;

    return started;
}

/**
 * \brief           Initialize debug module
 * \return          `1` on success, `0` otherwise
 */
uint8_t
comm_init(void) {
    return 1;
}
