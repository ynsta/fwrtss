/**
 * @file program.c
 * @brief Main program
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
#include "eeprom.h"

#define SENSOR_OFFSET   -19.0f
#define SENSOR_GAIN     0.34f

#define PROG_PERIOD_MS  50

#define BLINK_D0_MS     100
#define BLINK_D1_SET_MS 500
#define BLINK_D1_CAL_MS 200

#define SLEEP_TIMEOUT_CY ((60000 / PROG_PERIOD_MS) * 15) /* 15 minutes */

#define TEMPDC_MAX      450
#define TEMPDC_MIN      150


#define PID_INT_MAX     (500)
#define PID_INT_MIN     (-PID_INT_MAX)

#define PID_KP          2.00f
#define PID_KI          0.15f
#define PID_KD          0.05f

#define PWM_MAX         100.0f


enum program_mode {
    e_settings = 0,
    e_run      = 1,
    e_backup   = 2,
    e_calib    = 3,
};


static enum program_mode pmode = e_settings;

static float tempdc_read(void);

static int wdg_init(int period_ms, int window_ms);
static void wdg_refresh(void);

static void pid_init(void);
static float pid_update(float target, float current);

static void esw_init(void);

static float32_t encoder_filter(float32_t init, float32_t min, float32_t max);
static void esw_init(void);


static float32_t tempdc_display_filter(float32_t tempdc)
{
    static float32_t tempdc_tab[10] = { 0.0f };
    static int       tempdc_idx    = -1;

    float32_t tempdc_avg;

    if (tempdc_idx == -1) {
        int i;
        for (i = 0; i < 10; i++)
            tempdc_tab[i] = tempdc;
        tempdc_idx = 0;
    }

    tempdc_tab[tempdc_idx] = tempdc;

    tempdc_avg = ((tempdc_tab[(tempdc_idx+0) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+1) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+2) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+3) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+4) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+5) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+6) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+7) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+8) % 10] * 1) +
                  (tempdc_tab[(tempdc_idx+9) % 10] * 1)) / 10.0f;
    tempdc_idx = (tempdc_idx + 1) % 10;

    return tempdc_avg;
}


int program(void)
{
    float32_t tempdc_target = TEMPDC_MIN;
    float32_t sensor_offset = SENSOR_OFFSET;
    float32_t prev_offset   = sensor_offset;
    float32_t tempdc        = 0.0f;
    float32_t pwm_dc        = 0.0f;
    float32_t tmpf;

    uint32_t sleep_cnt      = SLEEP_TIMEOUT_CY;
    uint32_t next_cycle;
    uint32_t now;
    uint32_t set_to = 0;
    uint32_t cycle_start;
    uint32_t tmp;

    enum program_mode _pmode;

    if (eeprom_init(&tmp, &sensor_offset)) {
        tmp           = TEMPDC_MIN;
        sensor_offset = SENSOR_OFFSET;
    }
    tempdc_target = tmp;

    pwm_init();
    encoder_init();
    pid_init();
    esw_init();
    smsg_init(PROG_PERIOD_MS);
    display_init();

    wdg_init(PROG_PERIOD_MS, PROG_PERIOD_MS/5);
    now = HAL_GetTick();
    next_cycle = now + PROG_PERIOD_MS;

    while (1) {

        now = HAL_GetTick();

        if (now >= next_cycle) {

            wdg_refresh();

            next_cycle = next_cycle + PROG_PERIOD_MS;
            cycle_start = now;


            if (--sleep_cnt == 0)
                pmode = e_settings;

            _pmode = pmode;

            switch (_pmode) {

            case e_run:

                /* Pause PWM */
                pwm_set_dc(0.0f);

                /* Wait 3ms for AD8495 output to stabilize */
                HAL_Delay(3);

                /* Read ADC */
                tempdc = tempdc_read() + sensor_offset;

                /* Set back PWM */
                pwm_set_dc(pwm_dc);

                /* Update DC with PID */
                pwm_dc = pid_update(tempdc_target, tempdc);
                pwm_set_dc(pwm_dc);

                /* Moving the encoder while in run mode permits to change target
                 * but it is not saved in eeprom */
                if ((tmpf = encoder_filter(tempdc_target, TEMPDC_MIN, TEMPDC_MAX)) !=
                    tempdc_target) {
                    tempdc_target = tmpf;
                    set_to = 5; /* keep target display and blinking for 5 cycles */
                    sleep_cnt = SLEEP_TIMEOUT_CY;
                }

                if (set_to) {
                    set_to--;
                    display_set_blink(BLINK_D0_MS, BLINK_D0_MS);
                    display_set_val((unsigned)(tempdc_target));
                }
                else
                {
                    /* Go back to temperature display */
                    set_to = 0;
                    display_set_blink(1, 0);
                    display_set_val(tempdc_display_filter(tempdc));
                }

                /* Backup offset for callib mode */
                prev_offset = sensor_offset;

                break ;


            case e_calib:

                pwm_set_dc(0.0f);
                HAL_Delay(3);
                tempdc = tempdc_read() + sensor_offset;
                pwm_set_dc(pwm_dc);
                pwm_dc = pid_update(tempdc_target, tempdc - sensor_offset + prev_offset);
                pwm_set_dc(pwm_dc);

                /* Activate disaplay blinking */
                display_set_blink(BLINK_D1_CAL_MS, BLINK_D0_MS);

                /* Display temperature with offset read from encoder */
                display_set_val(tempdc_display_filter(tempdc));

                /* Read encoder to adjust temperature offset */
                sensor_offset = encoder_filter(sensor_offset, -50.0, +50);

                break;


            case e_settings:

                /* Refresh sleep timeout */
                sleep_cnt = SLEEP_TIMEOUT_CY;

                /* Disable PWM */
                pwm_dc = 0.0f;
                pwm_set_dc(pwm_dc);

                 /* Read temperature for serial message */
                tempdc = tempdc_read() + sensor_offset;

                display_set_blink(BLINK_D1_SET_MS, BLINK_D0_MS);

                /* Read encoder */
                tempdc_target = encoder_filter(tempdc_target, TEMPDC_MIN, TEMPDC_MAX);

                display_set_val((unsigned)(tempdc_target));

                break ;

            case e_backup:

                /* refresh sleep timeout */
                sleep_cnt = SLEEP_TIMEOUT_CY;

                /* Read temperature for serial message */
                tempdc = tempdc_read() + sensor_offset;

                display_clr();
                eeprom_write(tempdc_target, sensor_offset);

                /* reset PID */
                pid_init();

                pmode = e_run;
                break;
            }

            smsg_set(tempdc,
                     tempdc_target,
                     pwm_dc,
                     sensor_offset,
                     HAL_GetTick() - cycle_start,
                     _pmode);
            smsg_write();
        }


    }
    return 1; /* return 0 to reset */
}

/**
 * @brief Read temperature in Degree Celsius without offset applied.
 */
static float32_t tempdc_read(void)
{
    uint16_t adcvalue;

    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 10);

    adcvalue = HAL_ADC_GetValue(&hadc1);

    return (adcvalue * SENSOR_GAIN);
}

/**
 * @brief Initialize IWDG watchdog, to be refreshed between period_min_ms and
 * period_max_ms.
 */
static int wdg_init(int period_ms, int window_ms)
{
#ifndef NO_WDG
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
    hiwdg.Init.Reload = (uint16_t)((40 * (period_ms + (window_ms / 2))) / 8);
    hiwdg.Init.Window = (uint16_t)((40 * window_ms) / 8);

    if (HAL_IWDG_Start(&hiwdg) != HAL_OK)
        return 1;

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        return 1;
#endif
    return 0;
}

/**
 * @brief Refresh IWDG watchdog.
 */
static void wdg_refresh(void)
{
#ifndef NO_WDG
    HAL_IWDG_Refresh(&hiwdg);
#endif
}



static float32_t pid_integral;
static float32_t pid_error;

/**
 * @brief reset PID values.
 */
static void pid_init(void)
{
    pid_integral = 0.0f;
    pid_error    = 0.0f;
}

/**
 * @brief update PID.
 */
static float32_t pid_update(float32_t target, float32_t current)
{
    float32_t derivative;
    float32_t error;
    float32_t pwm;

    error = (float32_t)(target - current);
    pid_integral += error;
    derivative = error - pid_error;
    pid_error = error;

    if (pid_integral > PID_INT_MAX)
        pid_integral = PID_INT_MAX;
    if (pid_integral < PID_INT_MIN)
        pid_integral = PID_INT_MIN;

    pwm = (PID_KP * error)
        + (PID_KI * pid_integral)
        + (PID_KD * derivative);

    if (pwm > PWM_MAX)
        pwm = PWM_MAX;
    if (pwm < 0.0f)
        pwm = 0.0f;

    return pwm;
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
