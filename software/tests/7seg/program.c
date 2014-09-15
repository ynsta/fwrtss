/**
 * @file program.c
 * @brief 7seg test program
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

#include "stm32f3xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "iwdg.h"

#include "stm32f3xx.h"
#include "arm_math.h"

#include "display.h"
#include "program.h"
#include "smsg.h"

#define PROG_PERIOD_MS  100

int program(void)
{
    uint32_t next_cycle   = 0;
    uint32_t now;
    uint32_t cycle_start;

    float32_t count = 0.0f;

    smsg_init(PROG_PERIOD_MS);
    display_init();

    while (1) {

        now = HAL_GetTick();

        if (now >= next_cycle) {

            next_cycle = next_cycle + PROG_PERIOD_MS;
            cycle_start = now;

            display_set_blink(1, 0);
            display_set_val(count);

            count = count + 0.1f;

            smsg_set(count,
                     0,
                     0.0f,
                     0.0f,
                     HAL_GetTick() - cycle_start,
                     0);
            smsg_write();
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
