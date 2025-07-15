#ifndef _FILTERING_H_
#define _FILTERING_H_

#include <stdint.h>

#include "circular-buffer.h"

/* Median filter - rejects sudden spikes in readings better than an averaging.
 * This type of filter is best kept small because it requires sorting
 */
struct median_filter {
    int size;                        /* The number of elements in the sorted array */
    float *sorted;                   /* The elements in the circular buffer, sorted */
    struct circ_buffer time_ordered; /* Circular buffer of floating point values */
};

/* Moving average - smooths data to get rid of random noise.
 * This type of filter is best kept larger, but causes a lag in the output of
 * appprox half the size of the filter
 */
struct average_filter {
    struct circ_buffer buffer; /* Circular buffer of floating point values */
    float sum;                 /* The sum of the elements in the circular buffer */
};

/* Moving window - makes sure that minimum and maximum values stay within a certain range
 * for a certain amount of time, such as altitude at landing
 */
struct window_criteria {
    float min;         /* The current minimum we've seen */
    float max;         /* The current maximum we've seen */
    uint64_t duration; /* The time since the window was last reset*/

    float target_size;        /* The maximum size of the window, before resetting it */
    uint64_t target_duration; /* The minimum duration the window must be within target_size */
};

void median_filter_init(struct median_filter *filter, float *sorted, float *time_ordered, int size);
float median_filter_add(struct median_filter *filter, float new_value);
void average_filter_init(struct average_filter *filter, float *buffer, int size);
float average_filter_add(struct average_filter *filter, float new_value);

void window_criteria_init(struct window_criteria *window, float target_size, uint64_t target_duration);
void window_criteria_add(struct window_criteria *window, float new_value, uint64_t since_update);
int window_criteria_satisfied(struct window_criteria *window);

#endif /* _FILTERING_H_ */
