/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @date    23/08/2014 13:36:01
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* External variables --------------------------------------------------------*/

extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles DMA1 Channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
  
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/**
* @brief This function handles TIM6 global interrupt, DAC interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  HAL_TIM_IRQHandler(&htim6);
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
* @brief This function handles TIM1 break and TIM15 interrupts.
*/
void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(TIM1_BRK_TIM15_IRQn);
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim15);
}

/**
* @brief This function handles CAN RX0 and USB Low Priority interrupts.
*/
void USB_LP_CAN_RX0_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(USB_LP_CAN_RX0_IRQn);
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

/**
* @brief This function handles DMA1 Channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);
  
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
* @brief This function handles Non Maskable Interrupt.
*/
void NMI_Handler(void)
{
  HAL_RCC_NMI_IRQHandler();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
