/**
 * @file program.c
 * @brief cdc test program
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

#include <stdio.h>
#include <unistd.h>

#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "stm32f3xx.h"
#include "arm_math.h"

#include "uart_dma.h"
#include "display.h"
#include "program.h"

#define PROG_PERIOD_MS  100

static uint32_t cycle_start;

int program(void)
{
    uint32_t next_cycle   = 0;
    uint32_t now;

    float32_t count = 0.0f;
    uint8_t buf[64];

    uart_dma_init();
    display_init();

    while (1) {

        now = HAL_GetTick();

        if (now >= next_cycle) {

            next_cycle = next_cycle + PROG_PERIOD_MS;
            cycle_start = now;

            display_set_blink(1, 0);
            display_set_val(count);

            count = count + 0.1f;
            snprintf((char *)buf, sizeof(buf), "%3d.%d\r\n", (int)count, (int)(count*10.0f) % 10);

            CDC_Transmit_FS((uint8_t*)&buf, 7);
            write(1, buf, 7);
        }
    }
    return 1; /* return 0 to reset */
}


/**
 * @brief Error function
 */
void error(int code)
{
    display_set_digits((uint16_t)code);

    while (1)
        ;
}
