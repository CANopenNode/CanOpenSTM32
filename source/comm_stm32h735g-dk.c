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

#include "stm32h7xx.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"

/* Baudrate setup */
#define DEBUG_BAUDRATE                              115200

/* Maximum number of bytes to be transmitted in one DMA shot */
#define TX_MAX_LEN                                  512

/* Priority for DMA and UART peripheral */
#define UART_DMA_IRQ_PRIO                           6

/* Calculate length of statically allocated array */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/* Empty DMA buffer definition */
#ifndef DMA_BUFFER
#define DMA_BUFFER
#endif

/* LwRB setup */
static lwrb_t rb_tx;
static DMA_BUFFER uint8_t rb_tx_data[0x1000];
static volatile size_t tx_len;

/* RX variables */
lwrb_t comm_rb_rx;
static uint8_t rb_rx_data[0x1000];

/* Receive buffer for raw data received with DMA */
static DMA_BUFFER uint8_t usart_rx_dma_buffer[128];

/**
 * \brief           Process received data to user buffer
 */
static void
prv_check_rx(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_6);
    if (pos != old_pos) {
        if (pos > old_pos) {
            lwrb_write(&comm_rb_rx, &usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            lwrb_write(&comm_rb_rx, &usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            lwrb_write(&comm_rb_rx, &usart_rx_dma_buffer[0], pos);
        }
        old_pos = pos;
    }
}

/**
 * \brief           LwPRINTF callback function to actually print characters
 * \param[in]       ch: character to output
 * \param[in]       lw: LwPRINTF handle
 * \return          `ch` on success, `0` to stop further processing
 */
static int
prv_printf_out(int ch, lwprintf_t* lw) {
    if (ch > 0) {
        uint8_t c = (uint8_t)ch;
        lwrb_write(&rb_tx, &c, 1);
    } else {
        /* End of string, start transfer */
        comm_start_transfer();
    }
    return ch;
}

/**
 * \brief           Start transmit to send data
 * \return          `1` if just started, `0` if still on-going or no data to transmit
 */
uint8_t
comm_start_transfer(void) {
    uint32_t primask;
    uint8_t started = 0;

    primask = __get_PRIMASK();
    __disable_irq();
    if (tx_len == 0 && (tx_len = lwrb_get_linear_block_read_length(&rb_tx)) > 0) {
        const void* d = lwrb_get_linear_block_read_address(&rb_tx);

        /* Limit tx len up to some size to optimize buffer reading process */
        if (tx_len > TX_MAX_LEN) {
            tx_len = TX_MAX_LEN;
        }

        /*
         * Cleanup cache to ensure latest data are in target memory
         * Address and length have be 32-bytes aligned
         */
        //SCB_CleanDCache_by_Addr((uint32_t *)(((uint32_t)d) & ~(uint32_t)0x1F), (tx_len + 32) & 0x1F);

        /* Disable channel if enabled */
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);
        LL_USART_DisableDMAReq_TX(USART3);

        /* Configure DMA */
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_7, (uint32_t)d);
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_7, tx_len);

        /* Clean flags */
        LL_DMA_ClearFlag_TC7(DMA1);
        LL_DMA_ClearFlag_HT7(DMA1);
        LL_DMA_ClearFlag_TE7(DMA1);

        /* Enable instances */
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);
        LL_USART_Enable(USART3);
        LL_USART_EnableDMAReq_TX(USART3);
        started = 1;
    }
    __set_PRIMASK(primask);
    return started;
}

/**
 * \brief           Initialize debug module
 * \return          `1` on success, `0` otherwise
 */
uint8_t
comm_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Initialize LwRB prior any further actions */
    lwrb_init(&comm_rb_rx, rb_rx_data, sizeof(rb_rx_data));
    lwrb_init(&rb_tx, rb_tx_data, sizeof(rb_tx_data));

    /* Initialize printf lib */
    lwprintf_init(prv_printf_out);

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* Setup UART kernel clock */
    LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);

    /*
     * USART3 GPIO Configuration
     *
     * PD8   ------> USART3_TX
     * PD9   ------> USART3_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* DMA1_Channel6_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_DMA_IRQ_PRIO, 0));
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    /* USART3_RX DMA Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_6, LL_DMAMUX1_REQ_USART3_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, ARRAY_LEN(usart_rx_dma_buffer));

    /* Enable interrupts for RX */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_6);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);

    /* DMA1_Channel7_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_DMA_IRQ_PRIO, 0));
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    /* USART3_TX DMA Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_7, LL_DMAMUX1_REQ_USART3_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_7, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_7);

    /* Memory address and size is set before every transmit */

    /* Enable DMA interrupts */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_7);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_7);

    /* USART3 interrupt Init */
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_DMA_IRQ_PRIO, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    /* USART configuration */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = DEBUG_BAUDRATE;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
    LL_USART_EnableIT_IDLE(USART3);
    LL_USART_Enable(USART3);

    /* Acknowledge UART started well */
    while (!LL_USART_IsActiveFlag_TEACK(USART3) || !LL_USART_IsActiveFlag_REACK(USART3)) {}

    /* Start RX DMA */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);

    /* Start TX operation */
    comm_start_transfer();

    return 1;
}

/******************************/
/*     Interrupt handlers     */
/******************************/

/**
 * \brief           This function handles DMA1 channel7 global interrupt.
 */
void
DMA1_Stream7_IRQHandler(void) {
    /* Handle TX complete */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_7) && LL_DMA_IsActiveFlag_TC7(DMA1)) {
        LL_DMA_ClearFlag_TC7(DMA1);
        lwrb_skip(&rb_tx, tx_len);
        tx_len = 0;
        comm_start_transfer();
    }
}

/**
 * \brief           This function handles DMA1 channel6 global interrupt
 */
void
DMA1_Stream6_IRQHandler(void) {
    /* Handle RX operations */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_6) && LL_DMA_IsActiveFlag_TC6(DMA1)) {
        LL_DMA_ClearFlag_TC6(DMA1);
        prv_check_rx();
    }
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_6) && LL_DMA_IsActiveFlag_HT6(DMA1)) {
        LL_DMA_ClearFlag_HT6(DMA1);
        prv_check_rx();
    }
}

/**
 * \brief           This function handles USART3 global interrupt.
 */
void
USART3_IRQHandler(void) {
    /* Handle IDLE line interrupt */
    if (LL_USART_IsActiveFlag_IDLE(USART3)) {
        LL_USART_ClearFlag_IDLE(USART3);
        prv_check_rx();
    }
}
