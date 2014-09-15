/**
 * @file uart_dma.c
 * @brief RX UART CIRCULAR DMA
 * @author Stany MARCEL
 * @date 2014
 */

/*
 *  Copyright (c) 2014, Stany MARCEL <stanypub@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 *  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *  DAMAGE.
 */
#include <string.h>

#include "stm32f302x8.h"
#include "stm32f3xx_hal.h"
#include "usart.h"

#define  TX_BUFFER_LEN  1024

char     tx_buffer[TX_BUFFER_LEN];
uint32_t tx_idx_dma   = 0;
uint32_t tx_idx_write = 0;

char     rx_buffer[0xFF];
uint8_t  rx_idx_in  = 0;
uint8_t  rx_idx_out = 0;
int      rx_full    = 0;
char     rx_dma_buffer[2];


int uart_dma_init(void)
{
    tx_idx_dma   = 0;
    tx_idx_write = 0;
    rx_idx_in    = 0;
    rx_idx_out   = 0;
    rx_full      = 0;
    /* start the dma must be configured in circular mode */
    return HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_dma_buffer, 2) != HAL_OK;
}

int uart_dma_read(char * data, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (rx_idx_out == rx_idx_in && !rx_full)
            return i;
        else {
            data[i] = rx_buffer[rx_idx_out++];
            rx_full = 0;
        }
    }
    return len;
}

/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    rx_buffer[rx_idx_in++] = rx_dma_buffer[1];
    if (rx_full)
        rx_idx_out = rx_idx_in;
    rx_full = rx_idx_in == rx_idx_out;

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
       in the UART CR3 register */
    huart->Instance->CR3 |= USART_CR3_DMAR;
}

/**
  * @brief  Rx Half Transfer completed callbacks.
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    rx_buffer[rx_idx_in++] = rx_dma_buffer[0];
    if (rx_full)
        rx_idx_out = rx_idx_in;
    rx_full = rx_idx_in == rx_idx_out;
}

int uart_dma_write(char *ptr, int len)
{
    HAL_UART_StateTypeDef ustate;


    if (len > TX_BUFFER_LEN - tx_idx_write)
        len = TX_BUFFER_LEN - tx_idx_write;

    if (len <= 0) /* full  */
        return 0;

    memcpy(tx_buffer + tx_idx_write, ptr, len);
    tx_idx_write += len;

    ustate = HAL_UART_GetState(&huart1);

    if (ustate == HAL_UART_STATE_READY ||
        ustate == HAL_UART_STATE_BUSY_RX) {

        int       l = tx_idx_write - tx_idx_dma;
        uint8_t * p = (uint8_t *)(tx_buffer + tx_idx_dma);

        tx_idx_dma = tx_idx_write;
        HAL_UART_Transmit_DMA(&huart1, p, l);
    }
    return len;
}

/**
  * @brief Tx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (tx_idx_dma == tx_idx_write) {
        /* DMA catched up on TX so reset indexes */
        tx_idx_dma = tx_idx_write = 0;
    }
    else {
        /* More data to transmit */
        HAL_UART_Transmit_DMA(huart,
                              (uint8_t *)(tx_buffer + tx_idx_dma),
                              tx_idx_write - tx_idx_dma);
        tx_idx_dma = tx_idx_write;
    }
}
