/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include <stdio.h>
#include "stm32f4xx_lp_modes.h"
#include "ccd_sensor.h"
#include "utilities.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define SUPER_DEBUG
#ifdef SUPER_DEBUG
# define static   // Define static to nothing, allows watching variables in watch
#endif

/* Define to use preprogrammed reading values useful for debugging */
//#define DEBUG_USE_STATIC_VALUES

/* Define to set the device to perform multiple readings with a 
    common calibration, instead of returning to calibrate after each finished reading. */
#define REPEAT_MEASURENT_READING_WHILE

/*Define what to print!*/
/* Define to print the outputs needed to generate the calibration curve. */
//#define EXTRACT_CALIBRATION_CURVE_DATA

/* Define to print  (and allow graphing of) the individual calibration and test CCD readouts (integers readings).
    This will also result in a sleep at those prints, to make sure the output stays graphed.*/
//#define PRINT_RAW_CCD_READOUT

/* Define to print (and allow graphing of) the resulting ratio/delta array.*/
#define PRINT_RATIO_CCD_READOUT

/* Print the user feedback. Disable to allow for longer term logging etc. with minimal noise. */
#define PRINT_USER

/*  Toggleable Test Definitions.
    Still requires modification the the main flow for testing. */
//#define TCL1401CL_CLEAR_TESTING
//#define TCL1401CL_READ_ONLY_TESTING
//#define TCL1401CL_READ_SINGLE_TESTING

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#ifdef DEBUG_USE_STATIC_VALUES
//Define some test arrays we can use for quicker/simpler debugging
uint32_t CCD_calib_test_arr[CCD_PIXEL_COUNT] = {
    2095,2129,2153,2152,2166,2168,2194,2206,2235,2256,2270,2287,2305,2326,2344,2371,2388,2404,2445,2456,2488,2513,2545,2584,2595,2589,2647,2669,2709,2744,2785,2824,2887,2964,3078,3125,3196,3197,3256,3283,3299,3314,3320,3263,3282,3415,3414,3412,3396,3395,3410,3412,3421,
    3444,3450,3460,3475,3481,3512,3540,3554,3577,3588,3600,3591,3581,3581,3568,3554,3556,3545,3531,3512,3493,3480,3470,3454,3460,3459,3465,3445,3429,3404,3369,3327,3326,3327,3336,3340,3274,3243,3219,3211,3136,3022,2958,2921,2880,2853,2848,2821,2799,2788,2758,2714,2661,
    2627,2618,2610,2564,2516,2484,2454,2421,2397,2378,2390,2359,2343,2336,2314,2299,2259,2249,2225,2215,2185,2168
};
uint32_t CCD_test_test_arr[CCD_PIXEL_COUNT] = {
    1951,1976,1989,1981,1988,1977,1992,1995,2025,2041,2046,2070,2078,2085,2068,2067,2064,2061,2097,2104,2134,2152,2181,2225,2239,2231,2309,2326,2375,2414,2465,2505,2573,2653,2765,2818,2898,2910,2994,3018,3038,3044,3049,3015,2996,3158,3182,3192,3167,3159,3182,3185,3191,
    3210,3216,3222,3225,3230,3258,3290,3304,3320,3328,3326,3318,3308,3307,3292,3262,3248,3229,3207,3178,3141,3113,3080,3053,3052,3054,3049,3008,2975,2925,2872,2814,2798,2820,2819,2812,2720,2687,2687,2689,2616,2512,2467,2461,2461,2478,2512,2533,2516,2501,2460,2423,2371,
    2337,2343,2335,2281,2231,2206,2188,2158,2145,2133,2154,2132,2129,2124,2112,2107,2078,2070,2058,2052,2027,2015
};
float32_t CCD_ratio_test_arr[CCD_PIXEL_COUNT] = {
    -0.030892,-0.031883,-0.028724,-0.028724,-0.024880,-0.024240,-0.023730,-0.020427,-0.020389,-0.018929,-0.017874,-0.014825,-0.014376,-0.011062,-0.009190,-0.006001,-0.001704,0.001258,0.004534,0.007777,0.011798,0.017179,0.021468,0.023709,0.026286,0.032687,0.036185,
    0.042537,0.047426,0.050693,0.054064,0.058989,0.061071,0.063921,0.064074,0.065183,0.064506,0.071450,0.068283,0.066242,0.063022,0.063460,0.063510,0.063181,0.059722,0.052415,0.049986,0.045306,0.044760,0.041950,0.041015,0.039740,0.037463,0.031983,0.029989,0.027608,
    0.025379,0.022899,0.020587,0.018810,0.017517,0.013991,0.012629,0.010325,0.009574,0.009569,0.009531,0.008317,0.007353,0.007053,0.008787,0.010878,0.013211,0.013713,0.012804,0.011311,0.010098,0.010049,0.012734,0.014840,0.019608,0.021978,0.025098,0.026427,0.028857,
    0.031772,0.034877,0.036785,0.037411,0.040033,0.042482,0.047926,0.052408,0.056884,0.062855,0.064852,0.067876,0.068606,0.070836,0.069602,0.071567,0.072473,0.072039,0.073171,0.073652,0.072567,0.071627,0.066875,0.062238,0.061834,0.062387,0.058456,0.054034,0.049565,
    0.046947,0.044220,0.039201,0.035840,0.030495,0.028583,0.025030,0.022142,0.018040,0.016196,0.013721,0.010187,0.009833,0.004795 
};
#endif

/*  Define parameters that could be vairable between LFDs. */
const char * UNIT_OF_DETECTION = {"'ng/mL'"}; //Detection unit of LFD
const char * BIOMARKER_NAME = {"'pathogenic bacteria'"}; //Name of detected pathogen
#define TEST_TIME 5 //Time between using the LFD and it supposedly being ready. CURRENTLY UNUSED!
const char * TEST_TIME_UNIT = {"'minutes'"}; //Functioning time unit
#define CONTROL_STRIP_THRESHOLD 0.01f //Threshold of control strip before measurment is considred not working. NOT OPTIMISED

/* Define whether to use curve fitting or linear interpolation to determine the biomarker concentration. */
#define USE_INTERPOLATION_CURVE_FITTING
//#define USE_INTERPOLATION_LINEAR
#if defined (USE_INTERPOLATION_CURVE_FITTING) && defined (USE_INTERPOLATION_LINEAR)
# error Only one interpolation mode can be active at a time.
#endif

#ifdef USE_INTERPOLATION_CURVE_FITTING
/*  The equation of the calibration curve hsould be placed here. Make sure that the argument,n, is always
     surrounded by parenthesis, and that the entire expression is additionally surrounded by parenthesis:
        - (a*( + b)
        - (a*(n)*(n) + b*(n) + c)  ie: ((-0.003* (n)*(n) +0.03 (n) + 0.02))
    If more natural equations with exponentials and squareroots are to be used, math.h needs to included,
     but this will bloat the software and should be avoided.
    This function f(x) is the concentration, where x is the intensity response from the LFDR. */
# define CALIBRATION_FUNCTION(n) ( ((-233100.0f)*(n)*(n)*(n)) + ((30560.0f)*(n)*(n)) + ((42.58f)*(n)) - (1.706f) ) // -2.331e+05, 3.056e+04 , 42.58 , -1.706
# define LIMIT_OF_DETECTION 1.5f //ng/mL , will not be used in calculations, only to display to user that its less that LOD in sample
# define LOWER_BOUNDS_OF_DETECTION_RATIO 0.0f //The minimum possible output
# define UPPER_BOUNDS_OF_DETECTION_RATIO 0.1f //The maximum possible output
# define LOWER_BOUNDS_OF_DETECTION LIMIT_OF_DETECTION
# define UPPER_BOUNDS_OF_DETECTION CALIBRATION_FUNCTION(UPPER_BOUNDS_OF_DETECTION_RATIO)
#endif

#ifdef USE_INTERPOLATION_LINEAR
/* This setup was used int the start due to lacking a calibration curve, and is not verified in the current software state.*/
# define CALIBRATION_CURVE_DATA_POINTS 21
# define CALIBRATION_CURVE_AMPLITUDE 0
# define CALIBRATION_CURVE_CONCENTRATION 1
static const float32_t calibration_curve[2][CALIBRATION_CURVE_DATA_POINTS] = 
{{
    0.0251201558564095,0.0252763893184618,0.0256355081315407,0.0264598748961960,0.0283464254621424,0.0326335769401872,0.0422225978331055,0.0629290900106217,0.104434552440458,0.176470358017296,0.275000000000000,0.373529641982704,0.445565447559542,0.487070909989378,0.507777402166894,0.517366423059813,0.521653574537858,0.523540125103804,0.524364491868459,0.524723610681538,0.524879844143591
},{
    0,500,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,8000,8500,9000,9500,10000
}};
#define LOWER_BOUNDS_OF_DETECTION_RATIO calibration_curve[1][0]
#define UPPER_BOUNDS_OF_DETECTION_RATIO calibration_curve[1][CALIBRATION_CURVE_DATA_POINTS-1]
#endif


/* Array to hold the latest CCD reading. This will be written to by DMA,
    and must be stored prior to the next CCD sensor reading.*/
extern uint32_t CCD_readout[CCD_PIXEL_COUNT]; //
/* The state of the CCD sensor. Used for internal flow control, and setup.*/
extern volatile CCD_Cycle_t CCD_state;
/*  Variables to store the initial (calibration) reading and the later 
     test strip reading, as well as arrays for provessing. */
uint32_t    CCD_read_calibration[CCD_PIXEL_COUNT],
            CCD_read_measurement[CCD_PIXEL_COUNT];
int32_t     CCD_delta_cal_meas[CCD_PIXEL_COUNT];//; //Difference between cal and meas, which can be negative
float32_t       CCD_ratio_delta_cal[CCD_PIXEL_COUNT]; //Ratio of change from calibration. 

/*  Variables used to process the measurements.
    - Pixel Indices for Test and Control Lines
    - Average value of Test and Control Lines Maxima
    - Average value of Minima Surrounding Maxima on either side 
    - Difference between test and control maxima and minima. */
uint32_t    maxima_test_pix_idx, 
            maxima_control_pix_idx;
float32_t   maxima_test_average_val, 
        maxima_control_average_val,
        minima_test_average, 
        minima_control_average,
        delta_test, 
        delta_control;

/* Variables to store concentration and whether within bounds..*/
uint32_t    concentration, 
            measurement_within_bounds;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
#ifdef PRINT_USER
    printf("\r\n\nLFDR Active...\r\n\tSetup Complete...\r\n");
#endif
    
    // Perform Final Setup for timers and ADC/DMA, and set the CCD sensor into a HighZ known stat..
    LL_TIM_EnableIT_UPDATE(TIM3); 
    LL_TIM_EnableIT_UPDATE(TIM4);
    LL_TIM_EnableIT_UPDATE(TIM6);
    DMA_ADC_Init();
    CCD_clear(); //Initialise the CCD sensor by clearing it, leaving it in a known state.
    CCD_state = IDLE_SETUP; //Force this state as clearing again next time will reqire this state to continue.
    
    DPRINT("\r\nDEBUG is active! Initialisation Complete... System Clock: %dMHz...\r\n", SystemCoreClock/1000000);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)    {
#ifdef PRINT_USER
        printf("\nReady for a new measurement...\
                \r\n\tPress 'button' to calibrate device with UNUSED LFD...\
                \r\n\tGoing to sleep...\r\n");
#endif
        /*Debug mode does not like the low_power_stop() func as it looses connectivity with the board.
            Comment out or define SUPER_DEBUG to negate this. */
#ifndef SUPER_DEBUG
        /*  Go into Deep Sleep Mode + sleep peripherals apart from ones needed, until button press of user_input
            WakeUp once starting */
        LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
        LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);
        low_power_stop(); //Will be gone out of via interrupt, so no timers etc. can be running unless you want them to!
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
#endif



        
#ifdef PRINT_USER
        printf("Button press received!  Calibrating UNUSED LFD...\r\n");
#endif
        /*  Calibration Measurement!
            Turn LED ON and only do the necessary stuff when its on, then turn off and process data.
            First turn the LED on and allow for settling of the intensity.
            Then calibrate the sensor input and take an average reading.
            turn the LED off again, and finally move the calibration reading into CCD_read_calibration. */
        LED_MEASUREMENT_GPIO_Port->BSRR = LED_MEASUREMENT_Pin; 
        HAL_Delay(T_STABILISE);
        /*Calibrate the CCD sensor. If not posible, print error. Currently nothing is done with error apart 
            from printing, as it should never happen, but it means that the upper or lower integration time 
            has been reached, corresponding to the LED being too bright or not bright/on at all. */
        if (CCD_calibrate() == 0) {
            /*  Error handling here*/
            printf("\tERROR in calibration! Continuing despite error..\r\n");
        }
        CCD_read_average();
        LED_MEASUREMENT_GPIO_Port->BSRR = (uint32_t)LED_MEASUREMENT_Pin << 16U; //Turn LED OFF
        move_CCD_data(CCD_readout, CCD_read_calibration);

#ifdef PRINT_RAW_CCD_READOUT
        /* Prints the integer readings from the sensor if this is desired, and sleeps the device 
            to allow interpretation of the output before it is cleared by the next terminal output */
        DPRINT("CCD_read_calibration Reading:\r\n");
        print_CCD_array(CCD_read_calibration);
        low_power_sleep(); //Sleep to leave the trace on the screen
#endif        





#ifdef REPEAT_MEASURENT_READING_WHILE
# ifdef PRINT_USER
        printf("\tCalibration Complete...\r\nExtract LFD -> Apply sample -> Re-insert USED LFD:\r\n\tPress 'button' once complete... Going to sleep...\r\n");
# endif
        while(1) {
#endif
#ifndef SUPER_DEBUG
            /*  Go To Sleep until ready - wait until button is pressed or a potential timer is ready (not implemented) */
            low_power_stop(); //Deep sleep //low_power_sleep(); //Light sleep for debug if necessary
#endif
# ifdef PRINT_USER
            printf("Button press received! Measurement of the LFD will commence in %d %s...\r\n", TEST_TIME, (char*)TEST_TIME_UNIT);
# endif
            /*  A wait is to be implemented here equal to the TEST_TIME  
                Currently, the measurement will happen immediately. */
            
            
#ifdef PRINT_USER
            printf("\tMeasuring USED LFD...\r\n");
#endif
            /*  Measurement of Used LFD*/
            LED_MEASUREMENT_GPIO_Port->BSRR = LED_MEASUREMENT_Pin; //Turn LED ON
            HAL_Delay(T_STABILISE);
            CCD_read_average();
            LED_MEASUREMENT_GPIO_Port->BSRR = (uint32_t)LED_MEASUREMENT_Pin << 16U; //Turn LED OFF
            move_CCD_data(CCD_readout, CCD_read_measurement);
            
#ifdef PRINT_RAW_CCD_READOUT
            DPRINT("CCD_read_measurement Reading:\r\n");
            print_CCD_array(CCD_read_measurement);
            low_power_sleep(); //Sleep to leave the trace on the screen
#endif
            







#ifdef PRINT_USER
            printf("\tProcessing...\r\n");
#endif
#ifdef DEBUG_USE_STATIC_VALUES
            //Testing by overwriting current data with previ captured CCD data!
            DPRINT("Using previously captured data! Comment if wanting actual readings!\r\n");
            move_CCD_data(CCD_calib_test_arr, CCD_read_calibration);
            move_CCD_data(CCD_test_test_arr , CCD_read_measurement);
#endif

            /* Calculate the difference between cal and meas, and then find the percentage change*/
            for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
                CCD_delta_cal_meas[i] = CCD_read_calibration[i] - CCD_read_measurement[i];
                CCD_ratio_delta_cal[i] = (float32_t)(CCD_delta_cal_meas[i]) / CCD_read_calibration[i]; //TODO WRONG! Facking weasley
            }
            
#ifdef DEBUG_USE_STATIC_VALUES
            /* Set the ratio array to a predetermined value */
            for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
                CCD_ratio_delta_cal[i] = CCD_ratio_test_arr[i];
            }
#endif
            
            /* DO the processing to find the concentration */
            //find maxima and their average value
            maxima_test_pix_idx = find_maxima_range(CCD_ratio_delta_cal, 0, CCD_MAX_PIXEL_SCAN_TEST_LINE);
            maxima_control_pix_idx = find_maxima_range(CCD_ratio_delta_cal, CCD_MAX_PIXEL_SCAN_CONTROL_LINE, CCD_PIXEL_COUNT);
            maxima_test_average_val = calculate_average_extrema_value(CCD_ratio_delta_cal, maxima_test_pix_idx, NUM_PIX_AVERAGE_MAX);
            maxima_control_average_val = calculate_average_extrema_value(CCD_ratio_delta_cal, maxima_control_pix_idx, NUM_PIX_AVERAGE_MAX);
            
            //find minima surrounding the maxima, find average value
            minima_test_average = calculate_minima_average_value(CCD_ratio_delta_cal, maxima_test_pix_idx, 
                0, CCD_PIXEL_COUNT_THREE_QUARTER, NUM_PIX_AVERAGE_MIN);
            minima_control_average = calculate_minima_average_value(CCD_ratio_delta_cal, maxima_control_pix_idx,
                CCD_PIXEL_COUNT_ONE_QUARTER, CCD_PIXEL_COUNT, NUM_PIX_AVERAGE_MIN);
            
            //Find difference between maxima and minima
            delta_test = maxima_test_average_val - minima_test_average;
            delta_control = maxima_control_average_val - minima_control_average;

            /* Print some values useful when developing and debugging the software! */
            DPRINT("Debug Variables:\r\n\tTmax_pixel %d, Tmax: %f \tCmax_pixel %d, Cmax: %f \r\n\tTmin %f, Cmin %f \r\n\tTmax/Tmin %f, Cmax/Cmin %f\r\n",
                maxima_test_pix_idx, maxima_test_average_val, maxima_control_pix_idx, maxima_control_average_val,
                minima_test_average, minima_control_average, delta_test, delta_control);



            /* If a calibratin curve is being developed, this is the output to be used for generatic this. */
#ifdef EXTRACT_CALIBRATION_CURVE_DATA
            /* Print the values encesssary for generating the caliration curve!  */
            printf("Calibration Curve:\r\n\tTmax_pixel %d, Tmax: %f \tCmax_pixel %d, Cmax: %f \r\n\tTmin %f, Cmin %f \r\n\tTmax/Tmin %f, Cmax/Cmin %f\r\n",
                maxima_test_pix_idx, maxima_test_average_val, maxima_control_pix_idx, maxima_control_average_val,
                minima_test_average, minima_control_average, delta_test, delta_control);
#endif
            

            /*  Check if control strip is above threshold to make sure the test can be trusted,
                and then do the necessary interpolation and user feedback depending on the interpolation
                method used/defined. */
            if ( delta_control >= CONTROL_STRIP_THRESHOLD ) {
#ifdef USE_INTERPOLATION_LINEAR
                /*  Check if the value is within bounds of the calibration array */
                if (delta_test < calibration_curve[CALIBRATION_CURVE_AMPLITUDE][0]) {
                    //check if the value is less than the minimum concetration, set to 0 if it is.
                    measurement_within_bounds = 2; //Set to something neq 1 so we can catch the underfow if we want.
                } else if (delta_test > calibration_curve[CALIBRATION_CURVE_AMPLITUDE][CALIBRATION_CURVE_DATA_POINTS-1]) {
                    measurement_within_bounds = 0; //set to 0 se we can display out of bounds error.
                } else {
                    measurement_within_bounds = 1;
                    /* Find the concentration through linear interpolation of the calibration array and our test data*/
                    //cycle through the calib array and find which linear segment the test strip delta is between.
                    for (uint_fast16_t i = 0; i < CALIBRATION_CURVE_DATA_POINTS-1; i++) {
                        /*  If delta is bigger than the next measurement, go to the measurement */
                        if (delta_test >= calibration_curve[CALIBRATION_CURVE_AMPLITUDE][i+1]) continue;
                        /*  
                        If code reaches here, test data lies within this segment: 
                            Perform linear interpolation:
                            With point A and B being the points of a straight line, and point C (delta_test) is on that line, 
                              can be expressed in X value by the same ratio of the Y values of the points
                            Since the ratio between the X and Y delta is equal we can write:
                                (yB-yA)/(yC-yA) = (xB-xA)/(xC-xA)
                            We can solve this for xC, so that xC = ( ( ( (yC-yA)(xB-xA) ) / (yB-yA) ) + xA ) */
                        /* The term (yC-yA):*/
                        float32_t delta_test_y = delta_test 
                            - calibration_curve[CALIBRATION_CURVE_AMPLITUDE][i];
                        /* The term (xB-xA):*/
                        float32_t delta_segment_x = calibration_curve[CALIBRATION_CURVE_CONCENTRATION][i+1] 
                            - calibration_curve[CALIBRATION_CURVE_CONCENTRATION][i];
                        /* The term ((yB-yA):*/
                        float32_t delta_segment_y = calibration_curve[CALIBRATION_CURVE_AMPLITUDE][i+1] 
                            - calibration_curve[CALIBRATION_CURVE_AMPLITUDE][i];
                        
                        /*  Use the above equation to solve for the concentration */
                        concentration = ( ( ( delta_test_y*delta_segment_x ) / delta_segment_y ) \
                                            + calibration_curve[CALIBRATION_CURVE_CONCENTRATION][i] );
                        break; // Exit for loop and finish measurement by printing information.
                    }
                }
#endif
#ifdef USE_INTERPOLATION_CURVE_FITTING
                /*  Check if the value is within bounds of the calibration array */
                if (delta_test < LOWER_BOUNDS_OF_DETECTION_RATIO) {
                    //check if the value is less than the minimum concetration, set to 0 if it is.
                    measurement_within_bounds = 2; //Set to something neq 1 so we can catch the underfow if we want.
                } else if (delta_test > UPPER_BOUNDS_OF_DETECTION_RATIO) {
                    measurement_within_bounds = 0; //set to 0 se we can display out of bounds error.
                } else {
                    measurement_within_bounds = 1;
                    /* Find the concentration through curve fitting.
                        Assuming the concentration is positive, 0.5f is added to round the
                        integer correctly, as integers only take the floor of the float.*/
                    concentration = (CALIBRATION_FUNCTION(delta_test)+0.5f);
                }
#endif     
#ifdef PRINT_USER
                /* Do the final user reporting prior to going back to sleep waithing for a new LFD to measure. */
                if (measurement_within_bounds == 1) {
                    printf("Measurement Successful: The concentration of %s is %d %s.\r\n", 
                        (char*)BIOMARKER_NAME, concentration, (char*)UNIT_OF_DETECTION);
                } else if (measurement_within_bounds == 2) {
                    printf("Measurement Successful: The concentration of '%s' is less than can be measured: < '%.2f' %s.\r\n", 
                        (char*)BIOMARKER_NAME, (float32_t)LOWER_BOUNDS_OF_DETECTION, (char*)UNIT_OF_DETECTION);
                } else if (measurement_within_bounds == 0) {
                    printf("Measurement Successful: The concentration of '%s' is more than can be measured: > '%.2f' %s.\r\n", 
                        (char*)BIOMARKER_NAME, (float32_t)UPPER_BOUNDS_OF_DETECTION, (char*)UNIT_OF_DETECTION);
                }
#endif
            } else { //If the control strip did not appear and the test was not valid, print some useful user feedback
#ifdef PRINT_USER
                printf("Measurement UNsuccessful: The control line did not appear!\r\n\
\tPossible Causes can be:\r\n\
\t - Insufficient sample volume\r\n\
\t - Malfunctioning LFD\r\n\
\t - Measurement Error\r\n");
#endif
            }
            DPRINT("Non-rounded value for testing: %.3f\r\n", CALIBRATION_FUNCTION(delta_test));
#ifdef PRINT_RATIO_CCD_READOUT
            DPRINT("Delta_test , delta_control, CCD_ratio_delta_cal array:%f;%f;\r\n",delta_test, delta_control);
            print_CCD_array_float32_t(CCD_ratio_delta_cal); //For testing and calibration curve generation
#endif
#ifdef REPEAT_MEASURENT_READING_WHILE
        } //Repeated measurement reading while Bracket
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
