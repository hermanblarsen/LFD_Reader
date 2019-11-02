/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* Define float and double to their embeeded equivalents.
    Double is not supported by most architectures and should not be used.*/
typedef float float32_t;
typedef double float64_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define APB1_TIMER_CLOCK_MHz 90
#define APB1_TIMER_1MHz_PSC APB1_TIMER_CLOCK_MHz-1
#define CCD_CLEAR_T1_ARR 30-1
#define CCD_CLEAR_T2_ARR ((CCD_CLEAR_T1_ARR+1)*2)-1
#define CCD_READ_T2_ARR 150-1
#define CCD_READ_T1_ARR ((CCD_READ_T2_ARR+1)/2)-1
#define CCD_READ_ADC_ARR ((CCD_READ_T2_ARR+1)*2)-1
#define UINT32_MAX_ 4294967295
#define APB1_TIMER_90MHz_PSC 0
#define USER_BUTTON_Pin LL_GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define USER_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define CCD_SI_Pin LL_GPIO_PIN_2
#define CCD_SI_GPIO_Port GPIOC
#define CCD_CLK_Pin LL_GPIO_PIN_3
#define CCD_CLK_GPIO_Port GPIOC
#define USER_LED_Pin LL_GPIO_PIN_5
#define USER_LED_GPIO_Port GPIOA
#define LED_MEASUREMENT_Pin LL_GPIO_PIN_10
#define LED_MEASUREMENT_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
/* The above definitions are set in STM32CubeMX, and will be refreshed with code regeneration.*/

/* Time in ms for the LED to stabilise after turning on before a measurement commences */
#define T_STABILISE 1000
/* Number of pixels to average on the side of extrema */
#define NUM_PIX_AVERAGE_MAX 4
#define NUM_PIX_AVERAGE_MIN 3
/* Number of averages taken for reading the CCD sensor. */
#define CCD_AVERAGE_SAMPLES 200

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
