/**
 * @file program.c
 * @brief i2c test program
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

#include "stm32f3xx_hal.h"
#include "adc.h"
#include "crc.h"
#include "i2c.h"
#include "tim.h"
#include "iwdg.h"

#include "stm32f3xx.h"
#include "arm_math.h"

#include "display.h"
#include "encoder.h"
#include "pwm.h"
#include "program.h"
#include "uart_dma.h"

#define SENSOR_OFFSET   0.0f
#define SENSOR_GAIN     0.0f

#define PROG_PERIOD_MS  100

#define BLINK_D0_MS     100
#define BLINK_D1_SET_MS 500
#define BLINK_D1_CAL_MS 200

#define SLEEP_TIMEOUT_CY 18000 /* 30 min  */

#define TEMPDC_MAX      450
#define TEMPDC_MIN      100

#define EEPROM_MAGIC    0xBADBAD00
#define EEPROM_ADDRESS  0xA0
#define EEPROM_AWIDTH   I2C_MEMADD_SIZE_16BIT
#define EEPROM_PSIZE    32
#define EEPROM_NPAGE    32
#define EEPROM_TO_MS    50

#define PWM_MAX         100.0f


enum program_mode {
    e_settings = 0,
    e_run      = 1,
    e_backup   = 2,
    e_calib    = 3,
};

struct eeprom_backup {
    uint32_t  magic;
    uint32_t  tempdc_target;
    uint32_t  count;
    float32_t sensor_offset;
    uint8_t   padding[EEPROM_PSIZE - (4 * sizeof(uint32_t)) - (1 * sizeof(float32_t))];
    uint32_t  crc;
} __packed;


uint32_t eeprom_page2wr;

static enum program_mode pmode = e_settings;

static uint32_t cycle_start;

static struct eeprom_backup eeprom_backup = { 0 };

static float tempdc_read(void);

static int eeprom_init(uint32_t  * tempdc_target,
                       float32_t * sensor_offset);
static int eeprom_write(uint32_t  tempdc_target,
                        float32_t sensor_offset);
static void eeprom_erase(void);

static void esw_init(void);

static float32_t encoder_filter(float32_t init, float32_t min, float32_t max);
static void esw_init(void);

int program(void)
{
    uint32_t  tempdc_target = TEMPDC_MIN;
    float     sensor_offset = SENSOR_OFFSET;
    float32_t tempdc = 0.0f;
    float32_t pwm_dc = 0.0f;
    float32_t tmpf;

    uint32_t sleep_cnt    = SLEEP_TIMEOUT_CY;
    uint32_t next_cycle;
    uint32_t now;
    uint32_t set_to = 0;

    enum program_mode _pmode;

    uart_dma_init();

    if (eeprom_init(&tempdc_target, &sensor_offset)) {
        tempdc_target = TEMPDC_MIN;
        sensor_offset = SENSOR_OFFSET;
    }

    pwm_init();
    encoder_init();
    esw_init();
    display_init();

    now = HAL_GetTick();
    next_cycle = now + PROG_PERIOD_MS;

    while (1) {

        now = HAL_GetTick();

        if (now >= next_cycle) {

            next_cycle = next_cycle + PROG_PERIOD_MS;
            cycle_start = now;

                pwm_dc = 0.0;
                pwm_set_dc(pwm_dc);

            if (--sleep_cnt == 0)
                pmode = e_settings;

            _pmode = pmode;

            switch (_pmode) {

            case e_run:

                /* Read temperature */
                tempdc = tempdc_read() + sensor_offset;

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
                    display_set_val((unsigned)(tempdc));
                }

                break ;


            case e_calib:

                /* Read temperature with previous offset applied */
                tempdc = tempdc_read() + sensor_offset;

                /* Activate disaplay blinking */
                display_set_blink(BLINK_D1_CAL_MS, BLINK_D0_MS);

                /* Display temperature with offset read from encoder */
                display_set_val((unsigned)(tempdc));

                /* Read encoder to adjust temperature offset */
                sensor_offset = encoder_filter(sensor_offset, -50.0, +50);

                break;


            case e_settings:

                /* Refresh sleep timeout */
                sleep_cnt = SLEEP_TIMEOUT_CY;

                /* Disable PWM */
                pwm_dc = 0.0f;
                pwm_set_dc(pwm_dc);

                tempdc = tempdc_read() + sensor_offset;
                display_set_blink(BLINK_D1_SET_MS, BLINK_D0_MS);

                /* Read encoder */
                tempdc_target = encoder_filter(tempdc_target, TEMPDC_MIN, TEMPDC_MAX);

                display_set_val((unsigned)(tempdc_target));

                break ;

            case e_backup:

                /* refresh sleep timeout */
                sleep_cnt = SLEEP_TIMEOUT_CY;
                tempdc = tempdc_read() + sensor_offset;

                display_clr();
                eeprom_write(tempdc_target, sensor_offset);

                pmode = e_run;
                break;
            }

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

static int eeprom_init(uint32_t  * tempdc_target,
                       float32_t * sensor_offset)
{
    struct eeprom_backup tmp;
    uint32_t crc;
    uint32_t count_max = 0;
    int idx_max = -1;
    int i;

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
        eeprom_erase();

    for (i = 0; i < EEPROM_NPAGE; i++) {

        printf("R%d", i);

        if (HAL_I2C_Mem_Read(&hi2c1,
                             (uint16_t)EEPROM_ADDRESS,
                             i*EEPROM_PSIZE,
                             EEPROM_AWIDTH,
                             (uint8_t*)&tmp,
                             sizeof(tmp),
                             EEPROM_TO_MS) != HAL_OK) {
            printf(" Fail (read)\r\n");
            continue ;
        }

        if (tmp.magic != EEPROM_MAGIC) {
            printf(" Fail (MAGIC)\r\n");
            continue ;
        }

        crc = HAL_CRC_Calculate(&hcrc,
                                (uint32_t *)&tmp,
                                sizeof(tmp)/4 -1);
        if (crc != tmp.crc) {
            printf(" Fail (CRC)\r\n");
            continue ;
        }

        printf(" %lu\r\n", tmp.count);
        if (tmp.count >= count_max) {
            memcpy(&eeprom_backup, &tmp, sizeof(tmp));
            count_max = tmp.count;
            idx_max = i;
        }
    }

    if (idx_max >= 0) {
        printf("S%d\r\n", idx_max);
        eeprom_backup.count++;
        eeprom_page2wr = (idx_max + 1) % EEPROM_NPAGE;
        *tempdc_target = eeprom_backup.tempdc_target;
        *sensor_offset = eeprom_backup.sensor_offset;
        return 0;
    }
    else {
        printf("S none\r\n");
        memset(&eeprom_backup, 0, sizeof(tmp));
        eeprom_backup.magic = EEPROM_MAGIC;
        eeprom_page2wr = 0;
        return 1;
    }
}


static int eeprom_write(uint32_t  tempdc_target,
                        float32_t sensor_offset)
{
    eeprom_backup.tempdc_target = tempdc_target;
    eeprom_backup.sensor_offset = sensor_offset;
    eeprom_backup.crc = HAL_CRC_Calculate(&hcrc,
                                          (uint32_t *)&eeprom_backup,
                                          sizeof(eeprom_backup)/4 -1);

    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
        return 1;

    printf("W%lu %lu",
           eeprom_page2wr,
           eeprom_backup.count);

    if (HAL_I2C_Mem_Write(&hi2c1,
                          (uint16_t)EEPROM_ADDRESS,
                          eeprom_page2wr * EEPROM_PSIZE,
                          EEPROM_AWIDTH,
                          (uint8_t*)&eeprom_backup,
                          sizeof(eeprom_backup),
                          EEPROM_TO_MS) == HAL_OK) {
        eeprom_page2wr = (eeprom_page2wr + 1) % EEPROM_NPAGE;
        eeprom_backup.count++;
        printf("\r\n");
        return 0;
    }
    else
        printf(" Fail (write)\r\n");
    return 1;
}

static void eeprom_erase(void)
{
    uint8_t buf[EEPROM_PSIZE] = { 0xFF };
    int i;

    eeprom_page2wr = 0;
    eeprom_backup.count = 0;

    HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS, 10, EEPROM_TO_MS*10);

    for (i = 0; i < EEPROM_NPAGE; i++) {

        printf("E%d", i);

        if (HAL_I2C_Mem_Write(&hi2c1,
                              (uint16_t)EEPROM_ADDRESS,
                              i * EEPROM_PSIZE,
                              EEPROM_AWIDTH,
                              buf,
                              EEPROM_PSIZE,
                              EEPROM_TO_MS) == HAL_OK) {
            printf("\r\n");
        }
        else
            printf(" Fail\r\n");
        HAL_Delay(10);
    }
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
