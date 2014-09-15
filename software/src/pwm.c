/**
 * @file pwm.c
 * @brief PWM generation functions
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

#include "pwm.h"
#include "program.h"

/**
 * @brief Initialize PWM generation.
 */
void pwm_init(void)
{
    pwm_set_pulse(0);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        error(0xE01);
    };
}

/**
 * @brief set PWM pulse must be between 0 and ARR (set to 1000).
 */
void pwm_set_pulse(uint32_t pulse)
{
    if (pulse > htim1.Instance->ARR)
        pulse = htim1.Instance->ARR;
    htim1.Instance->CCR1 = pulse;
}

/**
 * @brief get PWM pulse between 0 and ARR (set to 1000).
 */
uint32_t pwm_get_pulse(void)
{
    return htim1.Instance->CCR1;
}

/**
 * @brief set PWM duty cycle must be between 0.0f and 100.0f.
 */
void pwm_set_dc(float32_t dc)
{
    uint32_t pulse;

    if (dc <= 0.001f)
        pulse = 0;
    else {
        pulse = (uint32_t)((dc / 100.0f) * htim1.Instance->ARR);
        if (pulse > htim1.Instance->ARR)
            pulse = htim1.Instance->ARR;
    }
    pwm_set_pulse(pulse);
}
