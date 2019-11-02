/**
  ******************************************************************************
  * File Name          : dma.c
  * Description        : This file provides code for the configuration
  *                      of all the requested memory to memory DMA transfers.
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
/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */
#include "ccd_sensor.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */
/* DMA buffer for a full CCD sensor reading, form ccd_sensor.h*/
extern volatile uint16_t ADC_DMA_buffer[CCD_PIXEL_COUNT];
/* USER CODE END 1 */

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* USER CODE BEGIN 2 */
/**
 * @brief Setup the remaining DMA ADC setup that is not performed automatically above or
 *  in setting up the adc in adc.c.
 *  Especially, this relates to setting the exact addresses for source and destination,
 *   which here is the ADC (constant address) and CCD_Array (incrementing address).
 */
void DMA_ADC_Init(void) {
    /* Set the DMA buffer size, equal to the length of the sensor */
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, CCD_PIXEL_COUNT);
    
    /* Configure Source and Destination Addresses. Source Address has to be retrieved
        from ADC via macro */
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, \
        LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), \
            (uint32_t)&ADC_DMA_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    
    /* Enable DMA transfer interruption: transfer complete */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

    /* Enable DMA transfer interruption: half transfer */
    /* this interrupt is not needed for one-shot reading, but should most likely be used for 
      constant reading by reading to one half of the buffer and processing the other 
       half at the same time, and the swtiching halves and doing the same thing.*/
//    LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_0);

    /* Enable DMA transfer interruption: transfer error */
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);
}
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
