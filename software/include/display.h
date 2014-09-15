/**
 * @file display.h
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

#ifndef __DISPLAY_H__
#define __DISPLAY_H__
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include "gpio.h"
#include "stm32f3xx.h"
#include "arm_math.h"

#define S7_SEL_MASK     (GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15)
#define S7_DIG_MASKA    (GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4)
#define S7_DIG_MASKB    (GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_2)

void display_init(void);
void display_set_val(float32_t value);
void display_set_digits(uint16_t digits);
void display_set_blink(uint32_t on_ms, uint32_t off_ms);
void display_clr(void);

#ifdef __cplusplus
}
#endif
#endif /* __DISPLAY_H__ */
