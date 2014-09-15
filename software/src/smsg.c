/**
 * @file smsg.c
 * @brief Serial message to host definitions
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
#include <unistd.h>

#include "smsg.h"

#include "crc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "uart_dma.h"

struct serial_message {
    uint8_t   id;
    uint8_t   pmode;
    uint16_t  count;
    uint32_t  period_ms;
    float32_t tempdc;
    float32_t target;
    float32_t pwm_dc;
    float32_t offset;
    uint32_t  cycle_ms;
    uint32_t  crc;
} __packed;

static struct serial_message smsg;

void smsg_init(uint32_t period_ms)
{
    smsg.id        = 0xAA;
    smsg.count     = 0;
    smsg.period_ms = period_ms;
    uart_dma_init();
}
void smsg_set(float32_t tempdc,
              float32_t target,
              float32_t pwm_dc,
              float32_t offset,
              uint32_t  cycle_ms,
              uint8_t   pmode)
{
    smsg.tempdc   = tempdc;
    smsg.target   = target;
    smsg.pwm_dc   = pwm_dc;
    smsg.offset   = offset;
    smsg.cycle_ms = cycle_ms;
    smsg.pmode    = pmode;
}

void smsg_write() {
    smsg.count++;
    smsg.crc = HAL_CRC_Calculate(&hcrc,
                                 (uint32_t *)&smsg,
                                 sizeof(smsg)/4 - 1);
    /* write to uart */
    write(1, &smsg, sizeof(smsg));

    /* write to usb */
    CDC_Transmit_FS((uint8_t*)&smsg, sizeof(smsg));
}
