#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "detector.h"

/* The that measurements should be valid for */
#define STALE_MEASUREMENT_TIME 1000000 /* 1 second in microseconds */

/* A measurement hasn't been updated in a long time and shouldn't be used */
#define IS_STALE(last_update, now) \
  (now - last_update > STALE_MEASUREMENT_TIME)

/* The maximum amount the altitude can change by in meters to consider us landed */
#define LANDED_ALT_WINDOW_SIZE 10.0f

/* The time in seconds that altitude variation must be within LANDED_ALT_WINDOW_SIZE */
#define LANDED_ALT_WINDOW_DURATION 5.0f

/* The change in altitude when idle to enter the airborne state */
#define LIFTOFF_ALT_THRESHOLD 20.0f

/* The acceleration above which we consider the rocket to be flying */
#define LIFTOFF_ACCEL_THRESHOLD 15.0f

void median_filter_init(struct median_filter *filter, float *sorted, float *time_ordered, int size) {
  filter->size = 0;
  filter->sorted = sorted;
  circ_buffer_init(&filter->time_ordered, time_ordered, size, sizeof(float));
}

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

static void insert_into_sorted(float value, float *sorted, int size) {
  // Find position to insert the new value
  int insert_index = 0;
  for (; insert_index < size; insert_index++) {
    if (sorted[insert_index] > value) {
      break;
    }
  }
  // Shift elements to the right after the insertion point
  for (int i = size; i >= insert_index; i--) {
    sorted[i + 1] = sorted[i];
  }
  sorted[insert_index] = value;
}

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

void average_filter_init(struct average_filter *filter, float *buffer, int size) {
  circ_buffer_init(&filter->buffer, buffer, size, sizeof(float));
  filter->sum = 0.0f;
}

float average_filter_add(struct average_filter *filter, float new_value) {
  float old_value = 0.0f;
  if (circ_buffer_push_out(&filter->buffer, &new_value, &old_value)) {
    filter->sum -= old_value;
  }
  filter->sum += new_value;
  return filter->sum / circ_buffer_size(&filter->buffer);
}

void detector_init(struct detector *detector) {
  detector->abs_max_altitude = -FLT_MAX;
  detector->alt_window_max = -FLT_MAX;
  detector->alt_window_min = FLT_MAX;
  detector->alt_window_duration = 0.0f;
  detector->landed_alt = 0.0f; // Figure out how to set properly later

  detector->current_time = 0;
  detector->last_alt_update = 0;
  detector->last_accel_update = 0;
  detector->current_alt = 0.0f;
  detector->current_accel = 0.0f;

  median_filter_init(&detector->alts.median, detector->alts.median_backing_sorted, detector->alts.median_backing_time_ordered, ALTITUDE_MEDIAN_FILTER_SIZE);
  average_filter_init(&detector->alts.average, detector->alts.average_backing, ALTITUDE_AVERAGE_FILTER_SIZE);

  median_filter_init(&detector->accels.median, detector->accels.median_backing_sorted, detector->accels.median_backing_time_ordered, ACCEL_MEDIAN_FILTER_SIZE);
  average_filter_init(&detector->accels.average, detector->accels.average_backing, ACCEL_AVERAGE_FILTER_SIZE);
}

void detector_add_alt(struct detector *detector, struct altitude_sample *sample) {
  float median = median_filter_add(&detector->alts.median, sample->altitude);
  detector->current_alt = average_filter_add(&detector->alts.average, sample->altitude);

  if (detector->current_alt > detector->abs_max_altitude) {
    detector->abs_max_altitude = detector->current_alt;
  }

  // Check for landed state here
  if (detector->alt_window_max - detector->alt_window_min > LANDED_ALT_WINDOW_SIZE) {
    // If the window is too large, reset it
    detector->alt_window_max = detector->current_alt;
    detector->alt_window_min = detector->current_alt;
    detector->alt_window_duration = 0;
  }
  else if (detector->current_alt > detector->alt_window_max) {
    detector->alt_window_max = detector->current_alt;
    detector->alt_window_duration = 0;
  }
  else if (detector->current_alt < detector->alt_window_min) {
    detector->alt_window_min = detector->current_alt;
    detector->alt_window_duration = 0;
  }
  else {
    detector->alt_window_duration += sample->time - detector->last_alt_update;
  }

  detector->last_alt_update = sample->time;
}

float detector_get_alt(struct detector *detector) {
  return detector->current_alt;
}

void detector_add_accel(struct detector *detector, struct accel_sample *sample) {
  float median = median_filter_add(&detector->accels.median, sample->acceleration);
  detector->current_accel = average_filter_add(&detector->accels.average, sample->acceleration);
  detector->last_accel_update = sample->time;
}

float detector_get_accel(struct detector *detector) {
  return detector->current_accel;
}

/**
 * Detects the occurence of events based on the current flight state and dynamics
 * @param detector The detector instance, currently unused (kept around in case state information is needed for events)
 * @param state The current flight state of the rocket
 * @param dynamics The current dynamics of the rocket, including altitude and velocity
 * @return The detected event, or DETECTOR_NO_EVENT if none was detected
 */
enum detector_event detector_detect(struct detector *detector, enum flight_state_e state) {
  switch (state) {
    case STATE_IDLE:
      if (!IS_STALE(detector->last_alt_update, detector->current_time)
          && detector->current_alt > (detector->landed_alt + LIFTOFF_ALT_THRESHOLD)) {
        return DETECTOR_AIRBORNE_EVENT;
      }
      else if (!IS_STALE(detector->last_accel_update, detector->current_time)
               && fabsf(detector->current_accel) > LIFTOFF_ACCEL_THRESHOLD) {
        return DETECTOR_AIRBORNE_EVENT;
      }
      break;

    case STATE_AIRBORNE: {
      break;
    }

    case STATE_LANDED: {
      break;
    }

    default:
      return DETECTOR_NO_EVENT;
  }
}
