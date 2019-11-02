#ifndef _UTILITIES_H_
#define _UTILITIES_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include <stdint.h>

/* Definitions ---------------------------------------------------------------*/


/* Prototypes ----------------------------------------------------------------*/
/**
 * @brief Prints the source array  "r0; r1;...;r127\r\n" for differnt arrays of
 *  varying type
 *  The different names are for different variable tpes in use in the arrays
 * @param Array to print
 */
void print_CCD_array(const volatile uint32_t * arr_source);
void print_CCD_array_int(const volatile int32_t * arr_source);
void print_CCD_array_float32_t(const volatile float32_t * arr_source);

/**
 * @brief Copies the content of a source array into a destination array.
 * 	 Source and destination must both be sized equal to the CCD_PIXEL_COUNT
 * @param Pointer to source array to copy from
 * @param Pointer to destination array to copy to
 */
void move_CCD_data(const volatile uint32_t * arr_source, uint32_t * arr_dest);

/**
 * @brief Finds the absolute maximum pixel value of any pixel and returns this value
 * @param Pointer to source array to process
 * @return Maximum pixel value found
 */
uint32_t utils_find_maximum_value(volatile uint32_t * array);


/**
 * @brief Find the index of the pixel with maximum value within a certain range. Will return the first pixel if 
 *  several pixels of equal maximum value are present
 * @param Pointer to source array to process
 * @param Minimum pixel index to search from
 * @param Maximum pixel index to search to
 * @return The index of the maximum pixel
 */
uint32_t find_maxima_range(const float32_t * arr_source, uint32_t range_min, uint32_t range_max);

/**
 * @brief Find the average value of the maximum pixel and some surrounding pixels.
 * @param Pointer to source array to process
 * @param Maximum pixel index
 * @param Pixels to average on each side of the maximum pixel
 * @return The average value of the pixels surrounding the maximum
 */
float32_t calculate_average_extrema_value(const float32_t * arr_source, uint32_t pixel_idx, uint32_t number_of_pixels);

/**
 * @brief Finds the minimum in a given range and calculates its average value over surrounding pixels near the minimum.
 *  Essentially operates as find_maxima_range and calculate_average_extrema_value combined into one function for minima.
 * @param Pointer to source array to process
 * @param Maximum pixel index
 * @param Minimum pixel index to search from
 * @param Maximum pixel index to search to
 * @param Number of pixels to average on each side of the found minimum.
 * @return The average value of the pixels surrounding the minimum
 */
float32_t calculate_minima_average_value(float32_t * arr_source, uint32_t max_pix_idx, 
    uint32_t range_min, uint32_t range_max, uint32_t number_of_pixels);

/**
 * @brief Performs laplacian smoothing on the input array, smoothing each pixel 
 *  influenced by its closest neighbouring pixels
 * @param Pointer to source array to process
 * @param Number of smoothing iterations.
 * @return Not used at this time, but can be used for error checking
 */
uint32_t utils_smooth_results_laplacian(volatile uint32_t * array, uint32_t num_of_smoothing_runs);


#endif /* _UTILITIES_H_ */
