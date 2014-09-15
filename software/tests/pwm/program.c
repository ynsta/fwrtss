/**
 * @file program.c
 * @brief encoder test program
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
#include "encoder.h"
#include "pwm.h"
#include "program.h"
#include "smsg.h"

#define PROG_PERIOD_MS  100

#define BLINK_D0_MS     100
#define BLINK_D1_SET_MS 500
#define BLINK_D1_CAL_MS 200

#define PWM_MAX         100.0f

enum program_mode {
    e_settings = 0,
    e_run      = 1,
    e_backup   = 2,
    e_calib    = 3,
};

static enum program_mode pmode = e_settings;

static float32_t encoder_filter(float32_t init, float32_t min, float32_t max);
static void esw_init(void);

int program(void)
{
    uint32_t next_cycle   = 0;
    uint32_t now;
    uint32_t cycle_start;

    float32_t count = 0.0f;

    enum program_mode _pmode;

    pwm_init();
    encoder_init();
    esw_init();
    smsg_init(PROG_PERIOD_MS);
    display_init();

    while (1) {

        now = HAL_GetTick();

        if (now >= next_cycle) {

            next_cycle = next_cycle + PROG_PERIOD_MS;
            cycle_start = now;

            _pmode = pmode;

            switch (_pmode) {
            case e_calib:
                display_set_blink(BLINK_D1_CAL_MS, BLINK_D0_MS);
                display_set_val(count);
                pwm_set_dc(0.0);
                break ;
            case e_settings:
                display_set_blink(BLINK_D1_SET_MS, BLINK_D0_MS);
                display_set_val(count);
                pwm_set_dc(0.0);
                break ;
            case e_backup:
                display_clr();
                pmode = e_run;
                pwm_set_dc(0.0);
                break ;
            case e_run:
                display_set_blink(1, 0);
                display_set_val(count);
                pwm_set_dc(count);
                break ;
            }

            count = encoder_filter(count, 0.0f, 100.0f);

            smsg_set(0.0f,
                     0,
                     count,
                     0.0f,
                     HAL_GetTick() - cycle_start,
                     _pmode);
            smsg_write();
        }
    }
    return 1; /* return 0 to reset */
}

static float32_t encoder_filter(float32_t init, float32_t min, float32_t max)
{
    int encoder_val;
    float32_t count = init;

    encoder_val = encoder_read();

    if (encoder_val >= 10)
        count = (int)count + 10;
    else if (encoder_val <= -10)
        count = (int)count - 10;
    else {
        if (count >= 100)
            count += encoder_val / 2.f;
        else
            count += encoder_val / 20.f;
    }

    if (count > max)
        count = max;

    if (count < min)
        count = min;

    return count;
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

static int esw_index;
static uint32_t esw_date;
static uint32_t esw_count1;
static uint32_t esw_count2;

/**
 * @brief Initialize button reader for input capture.
 */
static void esw_init(void)
{
    esw_index = 0;
    esw_date  = 0;

    if (HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2) != HAL_OK) {
        error(0xE04);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t diff;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {

        if (HAL_GetTick() < esw_date)
            return ;

        if (0 == esw_index) {

            esw_count1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            esw_index  = 1;
        }
        else if(esw_index == 1) {

            esw_count2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            /* If pin set must have missed an edge */
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
                esw_count1 = esw_count2;
                return ;
            }

            esw_index = 0;

            if (esw_count2 >= esw_count1)
                diff = (esw_count2 - esw_count1);
            else
                diff = ((0xFFFF - esw_count1) + esw_count2);

            /* Set a 1s timeout before next click */
            esw_date = HAL_GetTick() + 1000;

            switch (pmode) {
            case e_run:
                if (diff > 5000) /* 5000 * 0.001 = 5 s */
                    pmode = e_calib;
                else
                    pmode = e_settings;
                break ;
            case e_settings:
                pmode = e_backup;
                break ;
            case e_calib:
                pmode = e_backup;
                break ;
            case e_backup:
                break ;
            }
        }
    }
}
