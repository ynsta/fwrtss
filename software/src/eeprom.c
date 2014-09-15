/**
 * @file eeprom.c
 * @brief EEPROM settings backup defitions
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

#include "eeprom.h"

#include "i2c.h"
#include "crc.h"

#define EEPROM_MAGIC    0x600D0001
#define EEPROM_ADDRESS  0xA0
#define EEPROM_AWIDTH   I2C_MEMADD_SIZE_16BIT
#define EEPROM_PSIZE    32
#define EEPROM_NPAGE    32
#define EEPROM_TO_MS    50

struct eeprom_backup {
    uint32_t  magic;
    uint32_t  tempdc_target;
    float32_t sensor_offset;
    uint32_t  count;
    uint8_t   padding[EEPROM_PSIZE - (4 * sizeof(uint32_t)) - (1 * sizeof(float32_t))];
    uint32_t  crc;
} __packed;

static struct eeprom_backup eeprom_backup = { 0 };
static uint32_t eeprom_page2wr;

static void eeprom_erase(void);

/**
 * @brief Read EEPROM_NPAGE get the one with max count.
 */
int eeprom_init(uint32_t  * tempdc_target,
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

        if (HAL_I2C_Mem_Read(&hi2c1,
                             (uint16_t)EEPROM_ADDRESS,
                             i*EEPROM_PSIZE,
                             EEPROM_AWIDTH,
                             (uint8_t*)&tmp,
                             sizeof(tmp),
                             EEPROM_TO_MS) != HAL_OK)
            continue ;

        if (tmp.magic != EEPROM_MAGIC)
            continue ;

        crc = HAL_CRC_Calculate(&hcrc,
                                (uint32_t *)&tmp,
                                sizeof(tmp)/4 -1);
        if (crc != tmp.crc)
            continue ;

        if (tmp.count >= count_max) {
            memcpy(&eeprom_backup, &tmp, sizeof(tmp));
            count_max = tmp.count;
            idx_max = i;
        }
    }
    if (idx_max >= 0) {
        eeprom_backup.count++;
        eeprom_page2wr = (idx_max + 1) % EEPROM_NPAGE;
        *tempdc_target = eeprom_backup.tempdc_target;
        *sensor_offset = eeprom_backup.sensor_offset;
        return 0;
    }
    else {
        memset(&eeprom_backup, 0, sizeof(tmp));
        eeprom_backup.magic = EEPROM_MAGIC;
        eeprom_page2wr = 0;
        return 1;
    }
}

/**
 * @brief Initialize EEPROM backup.
 */
int eeprom_write(uint32_t  tempdc_target,
                 float32_t sensor_offset)
{
    eeprom_backup.tempdc_target = tempdc_target;
    eeprom_backup.sensor_offset = sensor_offset;
    eeprom_backup.crc = HAL_CRC_Calculate(&hcrc,
                                          (uint32_t *)&eeprom_backup,
                                          sizeof(eeprom_backup)/4 -1);

    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
        return 1;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          (uint16_t)EEPROM_ADDRESS,
                          eeprom_page2wr * EEPROM_PSIZE,
                          EEPROM_AWIDTH,
                          (uint8_t*)&eeprom_backup,
                          sizeof(eeprom_backup),
                          EEPROM_TO_MS) == HAL_OK) {
        eeprom_page2wr = (eeprom_page2wr + 1) % EEPROM_NPAGE;
        eeprom_backup.count++;
        return 0;
    }
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

        HAL_I2C_Mem_Write(&hi2c1,
                          (uint16_t)EEPROM_ADDRESS,
                          i * EEPROM_PSIZE,
                          EEPROM_AWIDTH,
                          buf,
                          EEPROM_PSIZE,
                          EEPROM_TO_MS);
        HAL_Delay(10);
    }
}
