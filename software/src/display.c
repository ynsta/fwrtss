/**
 * @file display.c
 * @brief 7seg display functions
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
#include "gpio.h"
#include "tim.h"
#include "stm32f3xx.h"
#include "arm_math.h"
#include "display.h"


/* ===================================================================
 * = DISPLAY                                                         =
 * ===================================================================
 */
static int      s7_blink   = 0;
static int      s7_state   = 1;
static uint32_t s7_dur_on  = 1;
static uint32_t s7_dur_off = 1;

static int s7_d[3] = { 0x10 };
static int s7_p[3] = { 0 };

struct s7_out {
    uint16_t pa;
    uint16_t pb;
};
static const struct s7_out s7_table[] = {
    { GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_5|GPIO_PIN_4, GPIO_PIN_1|GPIO_PIN_0             }, /*  0 = ACEF, BD  */
    {            GPIO_PIN_6                      , GPIO_PIN_1                        }, /*  1 =  C  , B   */
    { GPIO_PIN_7|           GPIO_PIN_5           , GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10 }, /*  2 = A E , BDG */
    { GPIO_PIN_7|GPIO_PIN_6                      , GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10 }, /*  3 = AC  , BDG */
    {            GPIO_PIN_6|           GPIO_PIN_4, GPIO_PIN_1|           GPIO_PIN_10 }, /*  4 =  C F, B G */
    { GPIO_PIN_7|GPIO_PIN_6|           GPIO_PIN_4,            GPIO_PIN_0|GPIO_PIN_10 }, /*  5 = AC F,  DG */
    { GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4,            GPIO_PIN_0|GPIO_PIN_10 }, /*  6 = ACEF,  DG */
    { GPIO_PIN_7|GPIO_PIN_6                      , GPIO_PIN_1                        }, /*  7 = AC  , B  */
    { GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4, GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10 }, /*  8 = ACEF, BDG */
    { GPIO_PIN_7|GPIO_PIN_6|           GPIO_PIN_4, GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10 }, /*  9 = AC F, BDG */
    { GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4, GPIO_PIN_1|           GPIO_PIN_10 }, /*  A = ACEF, B G */
    {            GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4,            GPIO_PIN_0|GPIO_PIN_10 }, /*  b =  CEF,  DG */
    { GPIO_PIN_7|           GPIO_PIN_5|GPIO_PIN_4,            GPIO_PIN_0             }, /*  C = A EF,  D  */
    {            GPIO_PIN_6|GPIO_PIN_5           , GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10 }, /*  d =  CE , BDG */
    { GPIO_PIN_7|           GPIO_PIN_5|GPIO_PIN_4,            GPIO_PIN_0|GPIO_PIN_10 }, /*  E = A EF,  DG */
    { GPIO_PIN_7|           GPIO_PIN_5|GPIO_PIN_4,                       GPIO_PIN_10 }, /*  F = A EF,   G */
    { 0                                          , 0                                 }  /* 10 =     ,     */
};

static void display_manage(void);
static uint32_t disp_period_ms = 1;


void display_init(void)
{
    s7_blink   = 0;
    s7_state   = 1;
    s7_dur_on  = 1;
    s7_dur_off = 1;

    disp_period_ms = ((htim6.Init.Period+1) * htim6.Init.Prescaler) / 72000;

    display_clr();
    display_manage();

    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief Set value to be displayed.
 */
void display_set_val(float32_t value)
{
    int tmp;

    if (value < 0.0f || value > 999.0f)
        s7_d[0] = s7_d[1] = s7_d[2] = 0xe;
    else {

        s7_p[0] = s7_p[2] = 0;
        if (value >= 100.0f) {
            s7_p[1] = 0;
            tmp = (int)value;
        }
        else {
            s7_p[1] = 1;
            tmp = (int)(value*10.0f);
        }

        s7_d[2] = tmp % 10;
        tmp /= 10;
        s7_d[1] = tmp % 10;
        tmp /= 10;
        s7_d[0] = tmp % 10;

        /* remove left 0 */
        if (!s7_d[0]) {
            s7_d[0] = 0x10;
            if (!(s7_d[1] || s7_p[1]))
                s7_d[1] = 0x10;
        }
    }
}

void display_set_digits(uint16_t digits)
{
    uint16_t tmp = digits & 0x0FFF;

    s7_d[0] = (tmp >> 8) & 0xF;
    s7_d[1] = (tmp >> 4) & 0xF;
    s7_d[2] = (tmp >> 0) & 0xF;
}


void display_set_blink(uint32_t on_ms, uint32_t off_ms)
{
    s7_dur_on  = on_ms / disp_period_ms;
    s7_dur_off = off_ms / disp_period_ms;

    if (s7_dur_on  == 0)
        s7_dur_on   = 1;
    if (s7_dur_off == 0)
        s7_dur_off  = 1;

    if (off_ms == 0)
        s7_blink = 0;
    else
        s7_blink = 1;
}

/**
 * @brief Clear display.
 */
void display_clr(void)
{
    s7_d[0] = s7_d[1] = s7_d[2] = 0x10;
    HAL_GPIO_WritePin(GPIOB, S7_SEL_MASK, GPIO_PIN_RESET);
}


/**
 * @brief Update display command.
 */
static void display_manage(void)
{
    static uint16_t value = GPIO_PIN_15;
    static uint32_t cnt = 0;

    uint16_t a_pins = 0;
    uint16_t b_pins = 0;

    cnt++;

    HAL_GPIO_WritePin(GPIOB, S7_SEL_MASK, GPIO_PIN_RESET);

    if (s7_blink) {
        if (s7_state) {
            HAL_GPIO_WritePin(GPIOB, value, GPIO_PIN_SET);
            if (!(cnt % s7_dur_on))
                s7_state = ! s7_state;
        }
        else
            if (!(cnt % s7_dur_off))
                s7_state = ! s7_state;
    }
    else
        HAL_GPIO_WritePin(GPIOB, value, GPIO_PIN_SET);


    switch (value) {
    case GPIO_PIN_15:
        a_pins = s7_table[s7_d[0]].pa;
        b_pins = s7_table[s7_d[0]].pb | (s7_p[0]? GPIO_PIN_2 : 0);
        value = GPIO_PIN_14;
        break ;
    case GPIO_PIN_14:
        a_pins = s7_table[s7_d[1]].pa;
        b_pins = s7_table[s7_d[1]].pb | (s7_p[1]? GPIO_PIN_2 : 0);
        value = GPIO_PIN_13;
        break ;
    case GPIO_PIN_13:
        a_pins = s7_table[s7_d[2]].pa;
        b_pins = s7_table[s7_d[2]].pb | (s7_p[2]? GPIO_PIN_2 : 0);
        value = GPIO_PIN_15;
        break ;
    case GPIO_PIN_12:
        a_pins = 0;
        b_pins = 0;
        value = GPIO_PIN_15;
        break ;
    default:
        a_pins = 0;
        b_pins = 0;
        value = GPIO_PIN_15;
    }

#ifndef PCBR6
    HAL_GPIO_WritePin(GPIOA, S7_DIG_MASKA, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, a_pins, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, S7_DIG_MASKB, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, b_pins, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(GPIOA, S7_DIG_MASKA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, a_pins, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, S7_DIG_MASKB, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, b_pins, GPIO_PIN_SET);
#endif
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
        display_manage();
}
