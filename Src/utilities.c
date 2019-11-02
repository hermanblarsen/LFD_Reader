/**
  ******************************************************************************
  * @file           : utilities.c
  * @brief          : Utilities used for DSP and CCD readout analysis
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "utilities.h"
#include "ccd_sensor.h"

#include "debug.h"
#ifdef DEBUG
#include <stdio.h>
#endif

/* Definitions ---------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/**
 * @brief Prints the source array  "r0; r1;...;r127\r\n"
 * @param Array to print, uint32_t
 */
void print_CCD_array(const volatile uint32_t * arr_source) {
    for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        if (i < CCD_PIXEL_COUNT-1) {
            printf("%d;", arr_source[i]); //single value print
        } else{
            printf("%d\r\n", arr_source[i]); //Final value print
        }
    }
}

/**
 * @brief Prints the source array  "r0; r1;...;r127\r\n"
 * @param Array to print, int32_t
 */
void print_CCD_array_int(const volatile int32_t * arr_source) {
    for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        if (i < CCD_PIXEL_COUNT-1) {
            printf("%d;", arr_source[i]); //single value print
        } else{
            printf("%d\r\n", arr_source[i]); //Final value print
        }
    }
}

/**
 * @brief Prints the source array  "r0; r1;...;r127\r\n"
 * @param Array to print, float32_t
 */
void print_CCD_array_float32_t(const volatile float32_t * arr_source) {
    for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        if (i < CCD_PIXEL_COUNT-1) {
            printf("%f;", arr_source[i]); //single value print
        } else{
            printf("%f\r\n", arr_source[i]); //Final value print
        }
    }
}

/**
 * @brief Copies the content of a source array into a destination array.
 *   Source and destination must both be sized equal to the CCD_PIXEL_COUNT
 * @param Pointer to source array to copy from
 * @param Pointer to destination array to copy to
 */
void move_CCD_data(const volatile uint32_t * arr_source, uint32_t * arr_dest) {
    for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        *(arr_dest+i) = *(arr_source+i);
    }
}

/**
 * @brief Finds the absolute maximum pixel value of any pixel and returns this value
 * @param Pointer to source array to process
 * @return Maximum pixel value found
 */
uint32_t utils_find_maximum_value(volatile uint32_t * array) {
    uint32_t max_val = 0; //set maximum value to minimum possible value
    
    for (uint_fast16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        if ( *(array+i) > max_val) max_val = *(array+i);
    }
    return max_val;
}

/**
 * @brief Find the index of the pixel with maximum value within a certain range. Will return the first pixel if 
 *  several pixels of equal maximum value are present
 * @param Pointer to source array to process
 * @param Minimum pixel index to search from
 * @param Maximum pixel index to search to
 * @return The index of the maximum pixel
 */
uint32_t find_maxima_range(const float32_t * arr_source, uint32_t range_min, uint32_t range_max) {
    uint32_t max_pix = 0; 
    float32_t max_val = -1; //Set the max value to something that is definitely less than any possible point.
    
    //Make sure min and max range are within bounds.
    if ((int32_t)range_min < 0) range_min = 0;
    if (range_max > CCD_PIXEL_COUNT) range_max = CCD_PIXEL_COUNT;
    
    for (uint_fast16_t i = range_min; i < range_max; i++) {
        if ( *(arr_source+i) > max_val){
            max_pix = i;
            max_val = *(arr_source+i);
        }
    }
    
    return max_pix;
}

/**
 * @brief Find the average value of the maximum pixel and some surrounding pixels.
 * @param Pointer to source array to process
 * @param Maximum pixel index
 * @param Pixels to average on each side of the maximum pixel
 * @return The average value of the pixels surrounding the maximum
 */
float32_t calculate_average_extrema_value(const float32_t * arr_source, uint32_t pixel_idx, uint32_t number_of_pixels) {
    float32_t average_max = 0;
    uint32_t range_min = pixel_idx - number_of_pixels;
    uint32_t range_max = pixel_idx + number_of_pixels;
    uint32_t pixels_averaged = 0;
    
    //Make sure min and max range are within bounds.
    if ((int32_t)range_min < 0) range_min = 0;
    if (range_max >= CCD_PIXEL_COUNT) range_max = CCD_PIXEL_COUNT;
    else range_max += 1; // Add one so the range includes 'number_of_pixels' on either side of the pixel
    
    
    for (uint_fast16_t i = range_min; i < range_max; i++) {
        average_max += *(arr_source+i);
        pixels_averaged++;
    }
    average_max /= pixels_averaged; //Divide by number of pixels added
    
    return average_max;
}

/**
 * @brief Finds the minimum in a given range and calculates its average value over surrounding pixels near the minimum.
 *  Essentially operates as find_maxima_range and calculate_average_extrema_value combined into one function for minima.
 * @param Source array to process
 * @param Maximum pixel index used to set an initial 'minimum' value that must be higher than any other value
 * @param Minimum pixel index to search from
 * @param Maximum pixel index to search to
 * @param Number of pixels to average on each side of the found minimum.
 * @return The average value of the pixels surrounding the minimum
 */
float32_t calculate_minima_average_value(float32_t * arr_source, uint32_t max_pix_idx, 
uint32_t range_min, uint32_t range_max, uint32_t number_of_pixels) {
    float32_t avg_tot, minimum;
    
    //Make sure min and max range are within bounds.
    if ((int32_t)range_min < 0) range_min = 0;
    if (range_max > CCD_PIXEL_COUNT) range_max = CCD_PIXEL_COUNT;
    if (max_pix_idx > range_max || (int32_t)max_pix_idx < (int32_t)range_min) return -1; //Error, should never happen.
    

    uint32_t min_pix;
    minimum = *(arr_source+max_pix_idx); //Set the minimum to the max pixel value
    for (uint_fast16_t i = range_min; i < range_max; i++) {
        if ( *(arr_source+i) < minimum){
            min_pix = i;
            minimum = *(arr_source+i);
        }
    }
    avg_tot = calculate_average_extrema_value(arr_source, min_pix, number_of_pixels);

    return avg_tot;
}


/**
 * @brief Performs laplacian smoothing on the input array, smoothing each pixel 
 *  influenced by its closest neighbouring pixels
 * @param Pointer to source array to process
 * @param Number of smoothing iterations.
 * @return Not used at this time, but can be used for error checking
 */
uint32_t utils_smooth_results_laplacian(volatile uint32_t * array, uint32_t num_of_smoothing_runs) {
    uint32_t smoothing_array[CCD_PIXEL_COUNT] = {0};
    
    /* Run this for the number of smoothing cycles desired */
    for (uint_fast16_t j = 0; j < num_of_smoothing_runs-1; j++) {

        //Average the first and the ;ast point with itself and its onely neighbour
        smoothing_array[0] = (*(array+0) + *(array+1))/2;
        smoothing_array[CCD_PIXEL_COUNT-1] = (*(array+(CCD_PIXEL_COUNT-1)) + *(array+(CCD_PIXEL_COUNT-2)))/2;
        
        //For the rest of the values, average each value 
        for (uint_fast16_t i = 1; i < CCD_PIXEL_COUNT-1; i++) {
            smoothing_array[i] = (*(array+i-1) + *(array+i+1))/2;
        }
        
        //Copy data back to array
        for (uint_fast16_t i = 1; i < CCD_PIXEL_COUNT-1; i++) {
            *(array+i) = smoothing_array[i];
        }
    }
    
    return 1;
}

