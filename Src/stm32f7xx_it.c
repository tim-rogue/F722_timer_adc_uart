/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"



/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart4;
extern ADC_HandleTypeDef hadc1;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{

}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{

  while (1)
  {

  }

}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{

  while (1)
  {

  }

}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{

  while (1)
  {

  }
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{

  while (1)
  {

  }

}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{

}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{

}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{

}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{

  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();

}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream2 global interrupt.
*/
void DMA1_Stream2_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_uart4_rx);

}

void UART4_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart4);
}
/**
* @brief This function handles DMA1 stream4 global interrupt.
*/
void DMA1_Stream4_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_uart4_tx);

}


// the ADC IRQ handler that could be used for interrupt style adc polling
// Tim added this because MX cube doesn't assume you we want to do interrupt style ADC conversions
void ADC_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&hadc1);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
