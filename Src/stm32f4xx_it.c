/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "ccd_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern volatile uint32_t clk_count;
extern volatile CCD_Cycle_t CCD_state;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void) {
  /* USER CODE BEGIN TIM3_IRQn 0 */
    /**
    * This is used for testing where the timing of ADC readings (triggered 
    *  by the updating of this timer) was verified by toggling CCD_SI for a short 
    *  period of time. Enable testing by uncommenting TCL1401CL_READ_ONLY_TESTING
    */
    TIM3->SR &= ~(TIM_SR_UIF); //Clear the interrupt event
#ifdef TCL1401CL_READ_ONLY_TESTING
    if (CCD_state != IDLE_SETUP) {
        //Toggle SI for testing purposes when ADC fires, TODO REMOVE after testing
        CCD_SI_GPIO_Port->BSRR = CCD_SI_Pin; //Set the CCD SI high 
        ADC_count_per_read++;
    /* Depending on the time scale, toggling between reads or toggling per 
        single read can be useful. Comment the below line to toggle once for a single read,
        resulting in a square wave with edges at adc reading starts. */
        CCD_SI_GPIO_Port->BSRR = (uint32_t)CCD_SI_Pin << 16U; //Set the CCD SI low
    }
#endif
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
    /**
      * This function is the start of the pulse train, and will enable TIM6 for then to be disabled.
      * First interrput it will set CLK high in the middle of SI being high, and start TIM6
      * Second interrupt it will set SI low and stop the timer.
      */
    
    TIM4->SR &= ~(TIM_SR_UIF); // Clear the timer 'update' interrupt event
    
    //This check prevents any unintiontional toggling by updating timer intervals (which are loaded at interrupt)
    if (CCD_state != IDLE_SETUP) {
        if(clk_count == 0) {
             /* Set CCD_CLK HIGH and enable timers - Setup should be done in CCD_Clear or CCD_Read respectively */
            CCD_CLK_GPIO_Port->BSRR = CCD_CLK_Pin; //Set CLK high
            TIM6->CR1  |= TIM_CR1_CEN; //Enable TIM6
            clk_count++;                 
        } else {
            /* Set CCD_SI low, start TIM3 (ADC Timer) if READING, and disable TIM4 */
            CCD_SI_GPIO_Port->BSRR = (uint32_t)CCD_SI_Pin << 16U; //Set the CCD SI low
            TIM4->CR1 &= ~(TIM_CR1_CEN); // Turn off this timer, leave rest to TIM6
            TIM4->SR &= ~(TIM_SR_UIF); //Clear the interrupt event in case it has triggered another time - should only happen twice.
         }
    }
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
    /**
      *  This ISR is simply used for waking from sleep. Can also be used for additional 
      *   interrupt behaviour which is currently commented.
      */
    
    /* Toggling of the user LED can be enabled by uncommenting the below 4 lines. */
//    //If user button is pressed, toggle the on board LED.
//    if (!(USER_BUTTON_GPIO_Port->IDR & USER_BUTTON_Pin)) {
//        USER_LED_GPIO_Port->ODR ^= USER_LED_Pin;
//    }

  /* USER CODE END EXTI15_10_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    /* USER CODE BEGIN LL_EXTI_LINE_13 */
    
    /* USER CODE END LL_EXTI_LINE_13 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    /**
      * This function performs the rest of the pulse train, started bt TIM4.
      * It will toggle CLK until CCD_PIXEL_COUNT+1 cycles have occured.
      * It will also start the integration timer at the correct time, if clearing.
      * Once finished, it will change the CD state to complete and stop the timer
      */
    
    // The DAC is not used. If used, this test should be enabled to separate timer events from DAC events
    //if (TIM6->SR & TIM_SR_UIF) {
        // Handle a timer 'update' interrupt event
        TIM6->SR &= ~(TIM_SR_UIF); //Clear the interrupt event
        
        if (CCD_state != IDLE_SETUP) {
            if (clk_count < CCD_PIXEL_COUNT + 1) {
                /* Toggle the CLK GPIO. Increment clk_count on rising edge.
                    Start integration Timer when appropriate */
                CCD_CLK_GPIO_Port->ODR ^= CCD_CLK_Pin; //Toggle CLK
                if (CCD_CLK_GPIO_Port->IDR & CCD_CLK_Pin) clk_count++; //If rising edge CLK, increment count
                if (clk_count == CCD_INTEGRATION_START && CCD_state == CLEARING) {
                    TIM2->CR1 |= TIM_CR1_CEN; //Enable TIM2
                }
            } else {
                /* Set CCD_SI low, set CCD_state, and disable TIM6 */ 
                CCD_CLK_GPIO_Port->BSRR = CCD_CLK_Pin << 16U; //Set CLK low
                if (CCD_state == CLEARING) CCD_state = CLEARING_COMPLETE; //Update CCD cycle state
                else if (CCD_state == READING_SINGLE) CCD_state = READING_SINGLE_COMPLETE;
                TIM6->CR1 &= ~(TIM_CR1_CEN); // Turn off TIM 6 (this timer)
                TIM6->SR &= ~(TIM_SR_UIF); //Clear the interrupt event to make sure it does not reoccur unexpectedly.
            }
        }
    //} else {DAC handler}
  /* USER CODE END TIM6_DAC_IRQn 0 */
  
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
    /* Clear the transfer complete flags for the specific DMA stream*/
    /* Check whether DMA transfer complete caused the DMA interruption */
    if(LL_DMA_IsActiveFlag_TC0(DMA2) == 1) {
        /* Clear flag DMA transfer complete */
        LL_DMA_ClearFlag_TC0(DMA2);
        /* Clear flag DMA global interrupt */
        /* (global interrupt flag: half transfer and transfer complete flags) */
        LL_DMA_ClearFlag_HT0(DMA2);
        //CCD_SI_GPIO_Port->ODR ^= CCD_SI_Pin;

        /* Call interruption treatment function */
        ADC_ConvFullCpltCallback();
    }

    /* Check whether DMA half transfer caused the DMA interruption */
    if(LL_DMA_IsActiveFlag_HT0(DMA2) == 1) {
        /* Clear flag DMA half transfer */
        LL_DMA_ClearFlag_HT0(DMA2);
        
        DEBUG_ASSERT(0);
        /* Call interruption treatment function */
        //AdcDmaTransferHalf_Callback();
    }

    /* Check whether DMA transfer error caused the DMA interruption */
    if(LL_DMA_IsActiveFlag_TE0(DMA2) == 1) {
        /* Clear flag DMA transfer error */
        LL_DMA_ClearFlag_TE0(DMA2);
        
        DEBUG_ASSERT(0);
        /* Call interruption treatment function */
        //AdcDmaTransferError_Callback();
    }
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
