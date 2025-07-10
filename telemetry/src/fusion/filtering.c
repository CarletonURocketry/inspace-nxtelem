#include "filtering.h"
#include <float.h>

/**
 * Remove a value from a sorted array by shifting all elements after the removed one to the left
 *
 * @param to_remove The value to remove
 * @param sorted The sorted array to remove the value from
 * @param size The number of elements in the sorted array
 */
static void remove_from_sorted(float to_remove, float *sorted, int size) {
  int remove_index = 0;
  for (; remove_index < size; remove_index++) {
    if (sorted[remove_index] == to_remove) {
      break;
    }
  }
  // Remove the oldest value by shifting to the left
  for (; remove_index < size - 1; remove_index++) {
    sorted[remove_index] = sorted[remove_index + 1];
  }
}

/**
 * Insert a value into a sorted array, shifting elements right after the insertion point
 *
 * @param value The value to insert
 * @param sorted The sorted array to insert a value into
 * @param size The number of elements in the sorted array
 */
static void insert_into_sorted(float value, float *sorted, int size) {
  // Find position to insert the new value
  int insert_index = 0;
  for (; insert_index < size; insert_index++) {
    if (sorted[insert_index] > value) {
      break;
    }
  }
  // Shift elements to the right after the insertion point
  for (int i = size - 1; i >= insert_index; i--) {
    sorted[i + 1] = sorted[i];
  }
  sorted[insert_index] = value;
}

/**
 * Initialize a median filter, providing backing arrays for the filter
 * 
 * @param filter The filter to initialize
 * @param sorted The memory to use as backing for the median filter's sorted array
 * @param time_ordered The memory to use as backing for the median filter's time ordered array
 * @param size The size of the sorted and time_ordered arrays in bytes. They must be the same size
 */
void median_filter_init(struct median_filter *filter, float *sorted, float *time_ordered, int size) {
  filter->size = 0;
  filter->sorted = sorted;
  circ_buffer_init(&filter->time_ordered, time_ordered, size, sizeof(float));
}

/**
 * Add a value to the median filter get the current result of the filter
 *
 * @param filter The filter to add the value to
 * @param new_value The new value to add
 * @return The current result of the median filter
 */
float median_filter_add(struct median_filter *filter, float new_value) {
  float to_remove = 0.0f;
  if (circ_buffer_push_out(&filter->time_ordered, &new_value, &to_remove)) {
    remove_from_sorted(to_remove, filter->sorted, filter->size);
    filter->size--;
  }
  insert_into_sorted(new_value, filter->sorted, filter->size);
  filter->size++;
  // Return the median (assume odd size)
  return filter->sorted[filter->size / 2];
}

/**
 * Initialize a averaging filter
 *
 * @param filter The averaging filter to initialize
 * @param buffer The memory to use as the backing for the averaging filter
 * @param size The size of buffer in bytes
 */
void average_filter_init(struct average_filter *filter, float *buffer, int size) {
  circ_buffer_init(&filter->buffer, buffer, size, sizeof(float));
  filter->sum = 0.0f;
}

/**
 * Add a value to an averaging filter, then get the current result of the averaging
 *
 * @param filter The averaging filter to add to
 * @param new_value The value to add to the filter
 * @return The current result of averaging
 */
float average_filter_add(struct average_filter *filter, float new_value) {
  float old_value = 0.0f;
  if (circ_buffer_push_out(&filter->buffer, &new_value, &old_value)) {
    filter->sum -= old_value;
  }
  filter->sum += new_value;
  return filter->sum / circ_buffer_size(&filter->buffer);
}

/**
 * Initialize a moving window
 * 
 * @param window The window to initialize
 * @param target_size The maximum size of the window. If the minimum and maximum values are greater than this, they and the duration will be reset
 * @param target_duration How long the window must be smaller than the target_size before window_criteria_satisfied returns 1
 */
void window_criteria_init(struct window_criteria *window, float target_size, uint64_t target_duration) {
    window->target_size = target_size;
    window->target_duration = target_duration;
    // No matter target_size, will be refreshed on the first sample
    window->min = -FLT_MAX;
    window->max = FLT_MAX;
    window->duration = 0;
}

/**
 * Update the window a new value, resetting it if necessary
 *
 * @param window The window to update
 * @param new_value The value to add to the window
 * @param since_update The time since the last update to the altitude window, in microseconds
 */
void window_criteria_add(struct window_criteria *window, float new_value, uint64_t since_update) {
    // If the window is too large, reset it
    if (window->max - window->min > window->target_size) {
        window->max = new_value;
        window->min = new_value;
        window->duration = 0;
        return;
    } else if (new_value > window->max) {
        // If we exceed the previous maximum or minimum, reset the time the window has been valid for
        window->max = new_value;
    } else if (new_value < window->min) {
        window->min = new_value;
    }
    window->duration += since_update;
}

/**
 * Check if the window's targets are satisfied
 *
 * @param window The window to check
 * @return 1 if the target size and duration are satisfied, 0 otherwise
 */
int window_criteria_satisfied(struct window_criteria *window) {
    /* The window makes sure that the maximum variation in altitude over a certain amount of time is
     * within a certain range. This is the same strategy for detecting landing that AltOS uses, and will prevent
     * landing detection during ascent and descent. This should be reliable enough to use at any time, provided we
     * aren't at transonic speeds
     */
    return (window->max - window->min) <= window->target_size && window->duration >= window->target_duration;
}
