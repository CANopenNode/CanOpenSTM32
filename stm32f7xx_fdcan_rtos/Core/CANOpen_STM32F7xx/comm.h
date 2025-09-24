/**
 * \file            comm.h
 * \brief           Communication interface for UART
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
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 */
#ifndef COMM_HDR_H
#define COMM_HDR_H

#include <stdint.h>
#include <string.h>
#include "lwrb/lwrb.h"
#include "lwprintf/lwprintf.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined( __ICCARM__ )
#define DMA_BUFFER                  _Pragma("location=\".dma_buffer\"")
#else
#define DMA_BUFFER                  __attribute__((section(".dma_buffer")))
#endif

/* External buffer for received data */
extern      lwrb_t comm_rb_rx;

uint8_t     comm_init(void);
uint8_t     comm_start_transfer(void);
#define     comm_printf             lwprintf_printf

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* COMM_HDR_H */
