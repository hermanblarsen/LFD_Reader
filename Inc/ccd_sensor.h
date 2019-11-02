#ifndef _CCD_SENSOR_H_
#define _CCD_SENSOR_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Definitions ---------------------------------------------------------------*/
//Define the ONE sensor in use. Defining both will lead to undefined behaviour and errors.
#define TSL1401CL
//#define TCD1103GFG

  /**
   * Enumeration for the CCD state, allowing for flow control with CCD sensor.
   */
typedef enum {
    IDLE_SETUP=0,
    CLEARING,
    CLEARING_COMPLETE,
    INTEGRATING,
    READING_SINGLE,
    READING_SINGLE_COMPLETE,
    READING_CONTINEOUS,
} CCD_Cycle_t;

/* If the TSL1401CL is used, define some specific variables and the specific 
    functions developed for this sensor.*/
#ifdef TSL1401CL
# define CCD_PIXEL_COUNT 128
# define CCD_INTEGRATION_START 19
# define CCD_INTEGRATION_MINIMUM_TIME (110 * 1.33 + 20) //((128-18)*(clear period)+20) us
# define CCD_INTEGRATION_INITIAL_TIME 300 //(500 + CCD_INTEGRATION_MINIMUM_TIME)
# define CCD_INTEGRATION_MAXIMUM_TIME 10000 //10ms  (actual max from CCD datasheet is 100ms)
/*Redefine functions for this sensor to the generic CCD sensor prototypes.*/
# define TSL1401CL_clear CCD_clear
# define TSL1401CL_read_single CCD_read
# define TSL1401CL_read_multiple CCD_read_multiple
#endif

/* If the TCD1103GFG is used, define some specific variables and the specific 
    functions developed for this sensor. This sensor was not implemented in this
    project. */
#ifdef TCD1103GFG
# define CCD_PIXEL_COUNT 1500
# define TCD1103GFG_clear CCD_clear
# define TCD1103GFG_read_single CCD_read
# define TCD1103GFG_read_multiple CCD_read_multiple
#endif

/*  Define some variables relating to timer offsets in the clear and read functions*/
#define CCD_CLEAR_TIM4_OFFSET 0
#define CCD_CLEAR_TIM6_OFFSET 0
#define CCD_READ_TIM4_OFFSET 0
#define CCD_READ_TIM6_OFFSET 0
#define CCD_READ_TIM3_OFFSET 96 //In TIM CNTs, imperically determined. Positive moves forward in time. Corresponding to 120ns earlier.
#define CCD_INTEGRATION_OFFSET_US 2 //In microseconds, imperically determined. Positive moves forward in time. Corresponding to 2us earlier. Nearest to actual needed time of 1.6-1.7us
#define CCD_INTEGRATION_OFFSET_NS +250 //ns, required delay remainder after timer has finished.

/* Set the lower and upper boundaries for calibration. The closer this is to 
    2^12=4096, the higher the resolution will be of the reading. */
#define CCD_SIGNAL_LEVEL_LIMIT_UPPER 4000
#define CCD_SIGNAL_LEVEL_LIMIT_LOWER 3600


/* Strategic positions on the sensor defined, used when reading two lines 
    to set boundaries of peak finding. */
#define CCD_PIXEL_COUNT_HALF CCD_PIXEL_COUNT/2
#define CCD_PIXEL_COUNT_THREE_QUARTER CCD_PIXEL_COUNT*3/4
#define CCD_PIXEL_COUNT_ONE_QUARTER CCD_PIXEL_COUNT/4
/*  Defined so that bleeding of test or control line cannot disturb the others measurement.
    This does require the peaks to be set at a location, and that this scan covers that 
     and some more, but not all the way to half.*/
#define CCD_MAX_PIXEL_SCAN_TEST_LINE CCD_PIXEL_COUNT_HALF-15
#define CCD_MAX_PIXEL_SCAN_CONTROL_LINE CCD_PIXEL_COUNT_HALF+25

/* Prototypes ----------------------------------------------------------------*/

/**
 * @brief Function to clear the TSL1401CL CCD Sensor for values that do not need to
 *  be read. The outputs will not be read.
 *  IMPORTANT: THe function expects CCD_State to be IDLE_SETUP prior to starting
 * @return Not used at this time, but can be used for error checking
 */
uint32_t CCD_clear(void);

/**
 * @brief Do a single reading of the TSL1401CL with integration time as argument. 
    Results are available from CCD_readout[] when DMA_buffer_copy_complete -> 1,
     and should be waited on until it goes HIGH prior to extrapolating the 
     reading data.
    The result of the reading will be accessible in the set DMA buffer at return,
     'CCD_readout'
 * @param Integration time used for between clearing and reading, in microseconds
 * @return Not used at this time, but can be used for error checking
 */
uint32_t CCD_read(uint32_t t_integration_us);

/**
 * @brief Function to calibrate the integration time of the TSL1401CL CCD Sensor reading.
 *  The calibration modified the integration time between 
 *   CCD_SIGNAL_LEVEL_LIMIT_UPPER and CCD_SIGNAL_LEVEL_LIMIT_LOWER set in this files header file.
 *   If the signal cannot be placed within the boundaries by going to either the minimum or 
 *    maximum integration time, 0 will be returned and the calibration will be unsuccessful. *   
 * @return Returns '1' for successful calibration, and '0' for unsuccessful calibration.
 *          Unsuccessful calibration will occur if LED is too bright or damaged/off/faulty.
 */
uint32_t CCD_calibrate(void);

/**
 * @brief Function to take an average reading of the CCD sensor, the number of averages set in
 *  main.h via CCD_AVERAGE_SAMPLES.
 *  Calibration should be performed at least once before performing this read.
 *  Once read, the result is available in the CCD_readout array.
 * @return Not used at this time, but can be used for error checking
 */
uint32_t CCD_read_average(void);


/**
 * @brief Sets and resets some parameters and runs clear to test CCD_clear() only,
 *  without needing to run read/others as well.
 */
void TSL1401CL_clear_test (void);
/**
 * @brief Test for single reading without dependency on clearing. Values will be rubbish,
 *  but functionality can be verified.
 */
void TSL1401CL_read_single_test (void);


/**
 * @brief Callback for when the ADC/DMA transfer is complete and the values will be 
 *  transferred from ADC_DMA_buffer to CCD_readout, 
 *  after which DMA_buffer_copy_complete will be set to '1'. 
 *  DMA_buffer_copy_complete must be reset after every manual read as performed 
 *   internally in CCD_read.
 */
void ADC_ConvFullCpltCallback(void);


#endif /* _CCD_SENSOR_H_ */
