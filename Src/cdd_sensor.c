/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ccd_sensor.c
  * @brief          : Code for interfacing with Linear CCD Sensors
  ******************************************************************************
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ccd_sensor.h"
#include "tim.h"
#include "utilities.h"

#include "debug.h"
#ifdef DEBUG
#include <stdio.h>
#endif

/* Definitions ---------------------------------------------------------------*/
/* Define some strategic number used for calibration in CCD_calibrate */
#define CCD_SIGNAL_LEVEL_CENTER (CCD_SIGNAL_LEVEL_LIMIT_UPPER+CCD_SIGNAL_LEVEL_LIMIT_LOWER)/2
#define CCD_SIGNAL_LEVEL_DELTA 4095-CCD_SIGNAL_LEVEL_LIMIT_UPPER
#define CCD_CALIBRATION_USE_STEPS   //if defined, will use min and maj steps rather than single steps
#define CCD_CALIBRATION_MAJOR 20    //Stepped Calibration major step
#define CCD_CALIBRATION_MINOR 1     //Stepped Calibration minor step
#define CCD_NOT_CALIBRATED 1


/* Variables -----------------------------------------------------------------*/
volatile uint32_t clk_count; //Counter for CLK pulses for the CCD sensor.
volatile CCD_Cycle_t CCD_state = IDLE_SETUP; //State of the CCD sensor
/* Global to hold the integration time for the system. Initially set to an
    initially defined time, later to hold the last calibration value. */
uint32_t t_integration_us = CCD_INTEGRATION_INITIAL_TIME;

/*  ADC / DMA / CCD Specific control variables and buffers. */
//Buffer which will be used as destination by the DMA to transfer readings without CPU intervention 
volatile uint16_t ADC_DMA_buffer[CCD_PIXEL_COUNT];
volatile uint_fast8_t DMA_buffer_copy_complete = 0; //Status flag for copying data from DMA 
volatile uint32_t CCD_readout[CCD_PIXEL_COUNT] = {0}; //This will be available until the next buffer is filled.

/* Variable storing the temporary average for CCD_average*/
static uint32_t CCD_average_temp[CCD_PIXEL_COUNT];


/*----------------------------------------------------------------------------*/
/* Interface with Linear CCD Sensor Array:  TSL1401CL                         */
/*----------------------------------------------------------------------------*/

/**
 * @brief Function to clear the TSL1401CL CCD Sensor for values that do not need to
 *  be read. The outputs will not be read.
 *  IMPORTANT: THe function expects CCD_State to be IDLE_SETUP prior to starting
 * @return Not used at this time, but can be used for error checking
 */
uint32_t TSL1401CL_clear(void) {
    clk_count = 0; //Reset clock count for new cycle
    
    /* State check BEFORE timer resets, as in other states timer events will modify GPIO outputs.
        MUST surround any updates to timer EGR regs */
#ifdef DEBUG
    if (CCD_state != IDLE_SETUP) DEBUG_ASSERT(0); //If CCD state is not IDLE for some reason, breakpoint.
#endif
    
    /* Reset timer TIM2 which will count integration time */
    TIM2->EGR  |= TIM_EGR_UG;
    
    /* Setup and reset timer TIM4 */
    TIM4->ARR   = CCD_CLEAR_T1_ARR; //Set the auto reload register to the desired period
    TIM4->EGR  |= TIM_EGR_UG;   // Send an update event to reset the timer and apply settings.
    
    /* Setup and reset timer TIM6 */
    TIM6->ARR   = CCD_CLEAR_T2_ARR;//Set the auto reload register to the desired period
    TIM6->EGR  |= TIM_EGR_UG; // Send an update event to reset the timer and apply settings.
    
    /* Wait for TIM4 to reset, and apply any neccesary offsets for initial timing adjustments.*/
    while (TIM4->SR & TIM_SR_UIF); // Until the update event has been cleared, wait here.
    
    /* State Change needs to happen AFTER timer resets, as in new state timer 
        events will modify GPIO outputs.*/
    CCD_state = CLEARING;
    
    /* Start the Signal Output Logic Timers and the first SI High:
        The next two lines are timing critical must be consequitive. */
    TIM4->CR1  |= TIM_CR1_CEN; // Enable timer 4.
    CCD_SI_GPIO_Port->BSRR = CCD_SI_Pin; //Set the CCD SI high
    
    /* Wait until the clear is finished. Modified from TIM6 IRQ */
    while (CCD_state != CLEARING_COMPLETE);
    
    return 1;
}

/**
 * @brief Do a single reading of the TSL1401CL with integration time as argument. 
    Results are available from CCD_readout[] when DMA_buffer_copy_complete -> 1,
     and should be waited on until it goes HIGH prior to extrapolating the 
     reading data.
    The result of the reading will be accessible in the set DMA buffer at return,
     'ADC_DMA_buffer'
 * @param Integration time used for between clearing and reading, in microseconds
 * @return Not used at this time, but can be used for error checking
 */
uint32_t TSL1401CL_read_single(uint32_t t_integration_us) {
#ifndef TCL1401CL_READ_ONLY_TESTING
    /*  Clear the CCD Sensor prior to reading, as this is needed for a one-shot image 
         to clear accumulated junk pixel charge. */
    TSL1401CL_clear();
#else
    //Setup ish values expected after CCD clear should have completed
    TIM2->CR1  |= TIM_CR1_CEN; // Enable timer 2
    TIM2->CNT = 2;
#endif
    //Set Control Variables
    clk_count = 0, DMA_buffer_copy_complete = 0;
    
    /* State Change needs to happen BEFORE timer resets, as in prev state as timer 
        events will modify GPIO outputs. MUST surround any updates to timer EGR regs */
    CCD_state = IDLE_SETUP;
    
    /* Reset timer TIM3 which will trigger ADC */
    TIM3->EGR  |= TIM_EGR_UG;   // Send an update event to reset the timer
    
    /* Setup and reset timer TIM4 */
    TIM4->ARR   = CCD_READ_T1_ARR; //Set the auto reload register to the desired period
    TIM4->EGR  |= TIM_EGR_UG;   // Send an update event to reset the timer and apply settings.
    
    /* Setup and reset timer TIM6 */
    TIM6->ARR   = CCD_READ_T2_ARR;//Set the auto reload register to the desired period
    TIM6->EGR  |= TIM_EGR_UG; // Send an update event to reset the timer and apply settings.
    
    
    /* Apply Offsets for initial timing adjustments. */
    while (TIM3->SR & TIM_SR_UIF); // Until the update event has been cleared, wait here.
    /* Offset TIM3 so that the ADC read event is generated at the apropriate time 
        in respect to pulse trains generated by TIM4/6, and so that ADC has 
        enough time to sample/read that.*/
    TIM3->CNT -= CCD_READ_TIM3_OFFSET; //Offset initial count to improve timing. 
    while (TIM4->SR & TIM_SR_UIF); // Until the update event has been cleared, wait here.
    //TIM4->CNT = CCD_READ_TIM4_OFFSET; //Offset initial count to improve timing. 
    //while (TIM6->SR & TIM_SR_UIF); // Until the update event has been cleared, wait here.
    //TIM6->CNT = CCD_READ_TIM6_OFFSET; //Offset initial count to improve timing. 
    
    /* State Change needs to happen AFTER timer resets, as in new state: timer 
        events will modify GPIO outputs.*/
    CCD_state = INTEGRATING;
    
    /* Enable/Reset ADC/DMA, and Clear ADC flags, DMA Flags should be clear.
        Should happen after Timer Resets */
    LL_ADC_Enable(ADC1);
    LL_DMA_EnableStream(DMA2,LL_DMA_STREAM_0);
    LL_ADC_ClearFlag_EOCS(ADC1);
    LL_ADC_ClearFlag_OVR(ADC1);
    
    /* Calculate the number of cycles required to make the desired integration offset
        in ns for use in below while loop. Could be moved into preprocessor if clock 
        frequency is decided on. */
    uint32_t t_offset_clk_cycles = (CCD_INTEGRATION_OFFSET_NS * (SystemCoreClock/1000000) / 1000 );
    t_offset_clk_cycles = t_offset_clk_cycles / 3; // Div by 3 to account for while loop unwrapping (machine code)
    /* Wait until the integration time has been reached to start the reading. */
    while ( TIM2->CNT < (t_integration_us - CCD_INTEGRATION_OFFSET_US) );
    TIM2->CR1 &= ~TIM_CR1_CEN; //Disable TIM2, not use after this until new read cycle
    /* A nanosecond delay, made by a while loop and decrementing counter. Will unwrap 
        to 3 machine code statements, \/__SUBS=->CMP->BNE__^ Until finished.
        This is used with the TIM2 comparison which is comparing to a rounded up
        value of microseconds, and this while loop will wait for the delta between
        waited microseconds and desired delay, in nanoseconds */
    while (t_offset_clk_cycles) t_offset_clk_cycles--; 
    
    /* State Change needs to happen AFTER timer resets, as in next state any timer 
        events will modify GPIO outputs. MUST surround any updates to timer EGR regs */
    CCD_state = READING_SINGLE;
    
    /* Start the Signal Output Logic Timer and the first SI High:
        The next three lines are timing critical must be consequitive. */
    TIM3->CR1 |= TIM_CR1_CEN; //Enable TIM3, will trigger first ADC reading in CCD_READ_ADC_ARR
    TIM4->CR1 |= TIM_CR1_CEN;
    CCD_SI_GPIO_Port->BSRR = CCD_SI_Pin; //Set the CCD SI high
    /*  If we change TIM4 and TIM6 uses, we can start TIM6 also from here but with an increased offset.
        This can be done by setting TIM4 as a down counter, which tim6 cannot be.
        Hence you can initially set CNT to be more than ARR 
        Should be considered to fine tuned timing.*/
    //TIM6->CR1 |= TIM_CR1_CEN; // Enable timer 6 --if uncommented, flow must be modified.
        
    /* Wait until the reading is finished. Modified from  TIM6 IRQ */
    while (CCD_state != READING_SINGLE_COMPLETE);
    
    /* Disable the DMA transfer */
    LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_0);
    LL_ADC_Disable(ADC1);
    CCD_state = IDLE_SETUP; //Return state to IDLE_SETUP
    return 1;
}

/**
 * @brief Function to calibrate the integration time of the TSL1401CL CCD Sensor reading.
 *  The calibration modified the integration time between 
 *   CCD_SIGNAL_LEVEL_LIMIT_UPPER and CCD_SIGNAL_LEVEL_LIMIT_LOWER set in this files header file.
 *   If the signal cannot be placed within the boundaries by going to either the minimum or 
 *    maximum integration time, 0 will be returned and the calibration will be unsuccessful. *   
 * @return Returns '1' for successful calibration, and '0' for unsuccessful calibration.
 *          Unsuccessful calibration will occur if LED is too bright or damaged/off/faulty.
 */
uint32_t CCD_calibrate (void) {
    uint32_t maximum_value, calibration_increment = CCD_CALIBRATION_MINOR;
    
    while(CCD_NOT_CALIBRATED) {
        CCD_read(t_integration_us); //read with current integration time
        while (!DMA_buffer_copy_complete);
        
        /*  Analyse the CCD_readout array to make sure there is no saturation,
            and that the signal is sufficiently large*/
        maximum_value = utils_find_maximum_value(CCD_readout);
        
        /*  Make sure the peak signal is between LOWER and UPPER limits set.
            If the signal is within these constraints, calibration completes. */
        if (maximum_value > CCD_SIGNAL_LEVEL_LIMIT_UPPER) {
#ifdef CCD_CALIBRATION_USE_STEPS
            if (  maximum_value - CCD_SIGNAL_LEVEL_LIMIT_UPPER >= CCD_SIGNAL_LEVEL_DELTA) calibration_increment = CCD_CALIBRATION_MAJOR;
            else calibration_increment = CCD_CALIBRATION_MINOR;
#endif
            /*  Make sure we dont go below minimum integration time - decrement if 
                still valid */
            if (t_integration_us - calibration_increment >= CCD_INTEGRATION_MINIMUM_TIME) {
                t_integration_us -= calibration_increment;
            } else {
                DPRINT("Warning: Calibration unsuccessful: Sensor is saturated, the LED is too bright!\r\n");
                return 0; // Failsafe if we cannot reach the desired level - return failure 
            }
        } else if (maximum_value < CCD_SIGNAL_LEVEL_LIMIT_LOWER) {
#ifdef CCD_CALIBRATION_USE_STEPS
            if ( CCD_SIGNAL_LEVEL_LIMIT_LOWER - maximum_value > CCD_SIGNAL_LEVEL_DELTA) calibration_increment = CCD_CALIBRATION_MAJOR;
            else calibration_increment = CCD_CALIBRATION_MINOR;
#endif
            /*  Make sure we dont go above maximum integration time - increment if 
                still valid */
            if (t_integration_us + calibration_increment <= CCD_INTEGRATION_MAXIMUM_TIME) {
                t_integration_us += calibration_increment;
            } else {
                DPRINT("Warning: Calibration unsuccessful: Maximum integration time %d reached!\r\n", (uint32_t)CCD_INTEGRATION_MAXIMUM_TIME);
                return 0; // Failsafe if we cannot reach the desired level - return failure
            }
        } else break; /*  If we are within limits, break the calibration */
    }
    DPRINT("Calibration successful: Initial time was %dus, current time is %dus\r\n", (uint32_t)CCD_INTEGRATION_INITIAL_TIME, t_integration_us);
    return 1;
}


/**
 * @brief Function to take an average reading of the CCD sensor, the number of averages set in
 *  main.h via CCD_AVERAGE_SAMPLES.
 *  Calibration should be performed at least once before performing this read.
 * @return Not used at this time, but can be used for error checking
 */
uint32_t CCD_read_average (void) {
    /*  Clear the average before starting new average accumulation */
    for(uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        CCD_average_temp[i] = 0; 
    }
    /*  Take CCD_AVERAGE_SAMPLES readings of the CCD and add 
        them to CCD_average_temp */
    for(uint_fast16_t i = 0; i < CCD_AVERAGE_SAMPLES; i++) {
        /*  Take one reading and wait until it is available in CCD_Readout */
        //DMA_buffer_copy_complete = 0; //Reset the DMA buffer status
        CCD_read(t_integration_us); //read with current integration time
        while (!DMA_buffer_copy_complete); //wait for completion

        /* Add the current contents of the CCD_readout to our average array.*/
        for(uint_fast16_t j = 0; j < CCD_PIXEL_COUNT; j++) {
            CCD_average_temp[j] += CCD_readout[j];
        }
    }

    /* Calculate average readings for each row of pixels and return it 
    to CCD_readout */
    for(uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        CCD_readout[i] = CCD_average_temp[i] / CCD_AVERAGE_SAMPLES;
    }
    return 1;
}


#ifdef TCL1401CL_CLEAR_TESTING
/**
 * @brief Sets and resets some parameters and runs clear to test CCD_clear() only,
 *  without needing to run read/others as well.
 */
void TSL1401CL_clear_test (void) {
    CCD_state = IDLE_SETUP; //for testing clear 
    CCD_clear();
    while (TIM2->CNT < 100000);
    TIM2->CR1 &= ~TIM_CR1_CEN; //Disable TIM2
}
#endif

#if defined (TCL1401CL_READ_ONLY_TESTING) || defined (TCL1401CL_READ_SINGLE_TESTING)
/**
 * @brief Test for single reading without dependency on clearing. Values will be rubbish,
 *  but functionality can be verified.
 */
void TSL1401CL_read_single_test (void) {
    DMA_buffer_copy_complete = 0;
    uint32_t t_integration_us;
    t_integration_us = CCD_INTEGRATION_MINIMUM_TIME+20;
    CCD_read(t_integration_us);
    while (!DMA_buffer_copy_complete);
    
    for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        printf("%d; ", CCD_readout[i]); //single value print
    }
    puts("\r");
}
#endif


/*----------------------------------------------------------------------------*/
/* Interface with Linear CCD Sensor Array:  TCD1103GFG                        */
/*----------------------------------------------------------------------------*/
/* These functions were not developed throughout the project.*/
// uint32_t TCD1103GFG_clear(void) {
//     return 1;
// }
// uint32_t TCD1103GFG_read_single(uint32_t t_integration_us) {
//     return 1;
// }
// uint32_t TCD1103GFG_read_multiple(uint32_t n, uint32_t t_integration_us) {
//     return 1;
// }


/**
  * @brief  Regular conversion complete callback in non blocking mode, 
  *  called by the end of conversion from DMA interrupt.
  * Callback for when the ADC/DMA transfer is complete and the values will be 
  *  transferred from ADC_DMA_buffer to CCD_readout, 
  *  after which DMA_buffer_copy_complete will be set to '1'. 
  */
void ADC_ConvFullCpltCallback(void) {
    TIM3->CR1 &= ~TIM_CR1_CEN; //Disable TIM3
    
    //Copy the DMA array into a external buffer that can be processed, then exit
    for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        CCD_readout[i] = ADC_DMA_buffer[i];
    }
#ifdef TCL1401CL_READ_ONLY_TESTING
    CCD_SI_GPIO_Port->BSRR = CCD_SI_Pin; //Set the CCD SI high for timing tests
#endif
    /*  Let the remaining program know reading is complete and that data available in the array */
    DMA_buffer_copy_complete = 1;
#ifdef TCL1401CL_READ_ONLY_TESTING
    CCD_SI_GPIO_Port->BSRR = (uint32_t)CCD_SI_Pin << 16U; //Set the CCD SI low for timing tests
#endif
}
