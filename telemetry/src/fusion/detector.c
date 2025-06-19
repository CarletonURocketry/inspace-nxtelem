#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "detector.h"

/* The that measurements should be valid for */
#define STALE_MEASUREMENT_TIME 1000000 /* 1 second in microseconds */

/* The maximum amount the altitude can change by in meters to consider us landed */
#define LANDED_ALT_WINDOW_SIZE 10.0f

/* The time in seconds that altitude variation must be within LANDED_ALT_WINDOW_SIZE */
#define LANDED_ALT_WINDOW_DURATION 5.0f

/* The change in altitude when idle to enter the airborne state */
#define AIRBORNE_ALT_THRESHOLD 20.0f

/* The acceleration above which we consider the rocket to be flying */
#define AIRBORNE_ACCEL_THRESHOLD 15.0f

/* The amount of time to wait after reaching apogee before detecting it as having happened */
#define APOGEE_DETECTION_DELAY 100000 /* 0.1 seconds in microseconds */

static void median_filter_init(struct median_filter *filter, float *sorted, float *time_ordered, int size) {
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

static float median_filter_add(struct median_filter *filter, float new_value) {
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

static void average_filter_init(struct average_filter *filter, float *buffer, int size) {
  circ_buffer_init(&filter->buffer, buffer, size, sizeof(float));
  filter->sum = 0.0f;
}

static float average_filter_add(struct average_filter *filter, float new_value) {
  float old_value = 0.0f;
  if (circ_buffer_push_out(&filter->buffer, &new_value, &old_value)) {
    filter->sum -= old_value;
  }
  filter->sum += new_value;
  return filter->sum / circ_buffer_size(&filter->buffer);
}

/* Update the altitude window criteria for landing */
static void detector_update_alt_window(struct detector *detector, float new_altitude, uint64_t time_since_update) {
  // If the window is too large, reset it
  if (detector->alt_window_max - detector->alt_window_min > LANDED_ALT_WINDOW_SIZE) {
    detector->alt_window_max = new_altitude;
    detector->alt_window_min = new_altitude;
    detector->alt_window_duration = 0;
  }
  // If we exceed the previous maximum or minimum, reset the time the window has been valid for
  else if (new_altitude > detector->alt_window_max) {
    detector->alt_window_max = new_altitude;
    detector->alt_window_duration = 0;
  }
  else if (new_altitude < detector->alt_window_min) {
    detector->alt_window_min = new_altitude;
    detector->alt_window_duration = 0;
  }
  else {
    detector->alt_window_duration += time_since_update;
  }
}

static int detector_check_alt_window_landed(struct detector *detector) {
  return detector->alt_window_max - detector->alt_window_min < LANDED_ALT_WINDOW_SIZE &&
         detector->alt_window_duration > LANDED_ALT_WINDOW_DURATION;
}

static int detector_alt_valid(struct detector *detector) {
  return detector->current_time - detector->last_alt_update < STALE_MEASUREMENT_TIME;
}

static int detector_accel_valid(struct detector *detector) {
  return detector->current_time - detector->last_accel_update < STALE_MEASUREMENT_TIME;
}

static int detector_is_airborne(struct detector *detector) {
  return (detector_alt_valid(detector) && fabs(detector->current_alt - detector->landed_alt) > AIRBORNE_ALT_THRESHOLD) ||
         (detector_accel_valid(detector) && fabs(detector->current_accel) > AIRBORNE_ACCEL_THRESHOLD);
}

static int detector_is_landed(struct detector *detector) {
  return detector_alt_valid(detector) && detector_check_alt_window_landed(detector) && 
         detector_accel_valid(detector) && fabs(detector->current_accel) < AIRBORNE_ACCEL_THRESHOLD;
}

static int detector_is_apogee(struct detector *detector) {
  /* Note - at transonic speeds the barometer is unreliable, so require 
   * acceleration to be less than what we get during the burning of the motor
   */
  return detector_alt_valid(detector) && detector->current_alt < detector->apogee &&
         (detector->apogee_time - detector->current_time) > APOGEE_DETECTION_DELAY &&
         detector_accel_valid(detector) && fabs(detector->current_accel) < AIRBORNE_ACCEL_THRESHOLD;
}

void detector_init(struct detector *detector) {
  detector->apogee = -FLT_MAX;
  detector->apogee_time = 0;
  detector->alt_window_max = -FLT_MAX;
  detector->alt_window_min = FLT_MAX;
  detector->alt_window_duration = 0.0f;
  detector->landed_alt = 0.0f; // TODO - Figure out how to set properly later

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
  // Filtering step to prevent false readings
  float median = median_filter_add(&detector->alts.median, sample->altitude);
  detector->current_alt = average_filter_add(&detector->alts.average, sample->altitude);

  // Keep track of apogee
  if (detector->current_alt > detector->apogee) {
    detector->apogee = detector->current_alt;
    detector->apogee_time = sample->time;
  }

  if (detector->rocket_state.state == STATE_AIRBORNE) {
    detector_update_alt_window(detector, detector->current_alt, sample->time - detector->last_alt_update);
  }

  detector->last_alt_update = sample->time;
  if (sample->time > detector->current_time) {
    detector->current_time = sample->time;
  }
}

float detector_get_alt(struct detector *detector) {
  return detector->current_alt;
}

void detector_add_accel(struct detector *detector, struct accel_sample *sample) {
  float median = median_filter_add(&detector->accels.median, sample->acceleration);
  detector->current_accel = average_filter_add(&detector->accels.average, sample->acceleration);

  detector->last_accel_update = sample->time;
  if (sample->time > detector->current_time) {
    detector->current_time = sample->time;
  }
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
enum detector_event detector_detect(struct detector *detector) {
  switch (detector->rocket_state.state) {
    case STATE_IDLE:
      // Add a way to set the landed height before we try to 
      if (detector_is_airborne(detector)) {
        return DETECTOR_AIRBORNE_EVENT;
      }
    case STATE_AIRBORNE: {
      switch (detector->rocket_state.substate) {
        case SUBSTATE_UNKNOWN:
          /* If we aren't sure what state we're really in, make sure we haven't landed */
          if (detector_is_landed(detector)) {
            return DETECTOR_LANDING_EVENT;
          }
          /* Fall through */
        case SUBSTATE_ASCENT:
          if (detector_is_apogee(detector)) {
            return DETECTOR_APOGEE_EVENT;
          }
          break;
        case SUBSTATE_DESCENT:
          if (detector_is_landed(detector)) {
            return DETECTOR_LANDING_EVENT;
          }
          break;
      }
      break;
    }

    default:
      return DETECTOR_NO_EVENT;
  }
}
