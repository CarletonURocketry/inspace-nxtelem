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

/* The change in altitude when idle to enter the airborne state in meters */
#define AIRBORNE_ALT_THRESHOLD 20.0f

/* The acceleration above which we consider the rocket to be flying in m/s^2 */
#define AIRBORNE_ACCEL_THRESHOLD 15.0f

/* The altitude below our maximum altitude while airborne we consider apogee to have been reached in meters */
#define APOGEE_ALT_THRESHOLD 20.0f

/* The accel above which we will not detect an apogee event in m/s^2 */
#define APOGEE_ACCEL_THRESHOLD 15.0f

/* The time on after initialization to take an altitude reading and use as the maximum landing altitude, in microseconds */
#define INIT_ELEVATION_DELAY 100000 /* 0.1 seconds in microseconds */

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/**
 * Initialize a median filter, providing backing arrays for the filter
 * 
 * @param filter The filter to initialize
 * @param sorted The memory to use as backing for the median filter's sorted array
 * @param time_ordered The memory to use as backing for the median filter's time ordered array
 * @param size The size of the sorted and time_ordered arrays in bytes. They must be the same size
 */
static void median_filter_init(struct median_filter *filter, float *sorted, float *time_ordered, int size) {
  filter->size = 0;
  filter->sorted = sorted;
  circ_buffer_init(&filter->time_ordered, time_ordered, size, sizeof(float));
}

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
 * @param sorted The sorted array to insert a value into, which has space for size + 1 elements
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
  for (int i = size; i >= insert_index; i--) {
    sorted[i + 1] = sorted[i];
  }
  sorted[insert_index] = value;
}

/**
 * Add a value to the median filter get the current result of the filter
 *
 * @param filter The filter to add the value to
 * @param new_value The new value to add
 * @return The current result of the median filter
 */
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

/**
 * Initialize a averaging filter
 *
 * @param filter The averaging filter to initialize
 * @param buffer The memory to use as the backing for the averaging filter
 * @param size The size of buffer in bytes
 */
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

/**
 * Update the altitude window which can be used to detect landing
 *
 * @param detector The detector with the altitude window to update
 * @param new_altitude The altitude measurement to add to the window
 * @param time_since_update The time since the last update to the altitude window
 */
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

/**
 * Check if the altitude window satisfies the conditions for landing
 *
 * @param detector The detector with the altitude window to check
 * @return 1 if the altitude window satisfies landing conditions, 0 otherwise
 */
static int detector_check_alt_window_landed(struct detector *detector) {
  /* The altitude window makes sure that the maximum variation in altitude over a certain amount of time is
   * within a certain range. This is the same strategy for detecting landing that AltOS uses, and will prevent
   * landing detection during ascent and descent. This should be reliable enough to use at any time, provided we
   * aren't at transonic speeds
   */
  return detector->alt_window_max - detector->alt_window_min < LANDED_ALT_WINDOW_SIZE &&
         detector->alt_window_duration > LANDED_ALT_WINDOW_DURATION;
}

/**
 * Check if the current altitude is valid and can be used for detection purposes
 *
 * @param detector The detector to use
 * @return 1 if the detector's current altitude measurement can be used in detection
 */
static int detector_alt_valid(struct detector *detector) {
  /* Currently, only check if the measurement as generated too long ago. Could also so sanity checks on its value */
  return detector->current_time - detector->last_alt_update < STALE_MEASUREMENT_TIME;
}

/**
 * Check if the current acceleration is valid and can be used for detection purposes
 *
 * @param detector The detector to use
 * @return 1 if the detector's current acceleration measurement can be used in detection
 */
static int detector_accel_valid(struct detector *detector) {
  /* Only check if the measurement is too old */
  return detector->current_time - detector->last_accel_update < STALE_MEASUREMENT_TIME;
}

/**
 * Check if the conditions for being airborne are satisfied
 *
 * @param detector The detector to use
 * @return 1 if the rocket satisfies the conditions to be airborne, 0 otherwise
 */
static int detector_is_airborne(struct detector *detector) {
  /* Check for an absolute change in altitude from landing, or a high acceleration. If the elevation is set wrong,
   * we may detect being airborne when set on the pad. Ideally, set the elevation correctly before starting to detect events,
   * but if we do detect based on an incorrect elevation, allow landing detections in airborne so that the consequences of 
   * the false detection aren't severe. As long as the rocket is on for a reasonable amount of time (1 min), going through
   * idle -> airborne -> landed -> idle shouldn't cause us to miss the real launch
   * 
   * Alternatively, we can also require the altitude criteria to be set AND the acceleration criteria, but we should check
   * the reliability of the accelerometer first
   */
  return (detector_alt_valid(detector) && fabs(detector_get_alt(detector) - detector->elevation) > AIRBORNE_ALT_THRESHOLD) ||
         (detector_accel_valid(detector) && detector_get_accel(detector) > AIRBORNE_ACCEL_THRESHOLD);
}

/**
 * Check if the conditions for being landed are satisfied
 *
 * @param detector The detector to use
 * @return 1 if the rocket satisfies the conditions to be landed, 0 otherwise
 */
static int detector_is_landed(struct detector *detector) {
  /* Use an altitude window to make sure there isn't too much variation in the altitude
   * and then check that acceleration is below launch levels. Acceleration check is necessary in case
   * the barometer's readings are unreliable when airborne
   */
  return detector_alt_valid(detector) && detector_check_alt_window_landed(detector) && 
         detector_accel_valid(detector) && detector_get_accel(detector) < AIRBORNE_ACCEL_THRESHOLD;
}

/**
 * Check if the conditions for apogee are satisfied
 *
 * @param detector The detector to use
 * @return 1 if the rocket satisfies the conditions for having passed apogee, 0 otherwise
 */
static int detector_is_apogee(struct detector *detector) {
  /* At transonic speeds the barometer is unreliable, so require acceleration to be less than what we
   * get during the burning of the motor. With okay filtering, we should be able to trust our barometer
   * at non-transonic speeds enough to compare apogee against the current height directly
   */
  return detector_alt_valid(detector) && detector->apogee - detector_get_alt(detector) > APOGEE_ALT_THRESHOLD &&
         detector_accel_valid(detector) && detector_get_accel(detector) < APOGEE_ACCEL_THRESHOLD;
}

/**
 * Initialize a detector with a number of defaults. Its best not to use the detector before setting some of these values
 * and providng a number of samples to populate the internal buffers with
 *
 * @param detector The detector to initialize
 */
void detector_init(struct detector *detector, uint64_t time) {
  median_filter_init(&detector->alts.median, detector->alts.median_backing_sorted, detector->alts.median_backing_time_ordered, ALTITUDE_MEDIAN_FILTER_SIZE);
  average_filter_init(&detector->alts.average, detector->alts.average_backing, ALTITUDE_AVERAGE_FILTER_SIZE);

  median_filter_init(&detector->accels.median, detector->accels.median_backing_sorted, detector->accels.median_backing_time_ordered, ACCEL_MEDIAN_FILTER_SIZE);
  average_filter_init(&detector->accels.average, detector->accels.average_backing, ACCEL_AVERAGE_FILTER_SIZE);

  detector->init_time = time;
  detector->current_time = time;
  detector->last_alt_update = 0;
  detector->last_accel_update = 0;
  detector->current_alt = 0.0f;
  detector->current_accel = 0.0f;

  detector->apogee = -FLT_MAX;
  detector->apogee_time = 0;

  detector->alt_window_max = -FLT_MAX;
  detector->alt_window_min = FLT_MAX;
  detector->alt_window_duration = 0.0f;

  /* This can be set manually, or will be set by the detector automatically */
  detector->elevation_set = 0;
  detector->elevation = 0.0f;

  /* These should ideally be set manually before the detector is used, but these defaults may work */
  detector->state = STATE_AIRBORNE;
  detector->substate = SUBSTATE_UNKNOWN;
}

/**
 * Add an altitude sample to the detector to perform filtering on, and then to update internal state with
 *
 * @param detector The detector to use
 * @param sample The altitude sample to use
 */
void detector_add_alt(struct detector *detector, struct altitude_sample *sample) {
  /* Time decreasing would not make sense in the detector, but possible with order of provided samples */
  if (sample->time > detector->current_time) {
    detector->current_time = sample->time;
  }

  // Filtering step to prevent false readings
  float median = median_filter_add(&detector->alts.median, sample->altitude);
  detector->current_alt = average_filter_add(&detector->alts.average, median);

  /* Keep track of apogee, could also add other validation to make sure we don't keep a bad apogee */
  if (detector->current_alt > detector->apogee) {
    detector->apogee = detector->current_alt;
    detector->apogee_time = sample->time;
  }

  /* Could limit use of the altitude window to states that need it */
  detector_update_alt_window(detector, detector->current_alt, sample->time - detector->last_alt_update);

  /* If we just powered on and elevation hasn't been set */
  if (!detector->elevation_set) {
    if (detector->current_time - detector->init_time > INIT_ELEVATION_DELAY) {
      /* Hopefully, the filters should be full and this should be a very sensible value */
      detector->elevation = detector->current_alt;
      detector->elevation_set = 1;
    }
  }

  detector->last_alt_update = sample->time;
}

/**
 * Get the current altitude value being used by the detector, after filtering
 *
 * @param detector The detector to use
 * @return The current altitude that the detector is using
 */
float detector_get_alt(struct detector *detector) {
  return detector->current_alt;
}

/**
 * Add an acceleration value (magnitude) to the detector to perform filtering on, and then to update internal state with
 *
 * @param detector The detector to use
 * @param sample the altitude sample to use
 */
void detector_add_accel(struct detector *detector, struct accel_sample *sample) {
  if (sample->time > detector->current_time) {
    detector->current_time = sample->time;
  }
  /* Note - get rid of the sign on the provided acceleration value since in case it has one */
  float median = median_filter_add(&detector->accels.median, fabs(sample->acceleration));
  detector->current_accel = average_filter_add(&detector->accels.average, median);

  detector->last_accel_update = sample->time;
}

/**
 * Get the current acceleration value being used by the detector, after filtering. Note: this
 * doesn't make use of altitude data. Only direct acceleration measurements are used
 * 
 * @param detector The detector to use
 * @return The current acceleration that the detector is using
 */
float detector_get_accel(struct detector *detector) {
  return detector->current_accel;
}

/**
 * Detects events based on the internal state of the detector, like samples and rocket state information
 *
 * @param detector The detector to use
 * @return The detected event, or DETECTOR_NO_EVENT if none were detected
 */
enum detector_event detector_detect(struct detector *detector) {
  /* We are doing detection events based on state because the checks we perform otherwise
   * might not make sense
   */
  switch (detector->state) {
    case STATE_IDLE: {
      if (!detector->elevation_set) {
        /* Don't detect until we have a filtered altitude to use as our landed altitutde */
        return DETECTOR_NO_EVENT;
      }
      if (detector_is_airborne(detector)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        printf("Detected airborne event from the idle state\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
        return DETECTOR_AIRBORNE_EVENT;
      }
    } break;
    case STATE_AIRBORNE: {
      switch (detector->substate) {
        case SUBSTATE_UNKNOWN:
          /* If we aren't sure what state we're really in, make sure we haven't landed */
          if (detector_is_landed(detector)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
            printf("Detected a landing event from the airborne state, unknown substate\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
            return DETECTOR_LANDING_EVENT;
          }
          /* Fall through */
        case SUBSTATE_ASCENT:
          if (detector_is_apogee(detector)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
            printf("Detected apogee from the airborne state\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
            return DETECTOR_APOGEE_EVENT;
          }
          break;
        case SUBSTATE_DESCENT:
          if (detector_is_landed(detector)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
            printf("Detected a landing event from the descent state\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
            return DETECTOR_LANDING_EVENT;
          }
          break;
      }
    } break;
    default:
      /* Ignore states like landing */
      break;
  }
  return DETECTOR_NO_EVENT;
}

/**
 * Set the current flight state of the detector. This should be set whenever the rocket's flight state changes,
 * otherwise the detector can provide events that aren't relevant and miss others
 *
 * @param detector The detector to set the state for
 * @param state The flight state of the rocket
 * @param substate The flight substate of the rocket
 */
void detector_set_state(struct detector *detector, enum flight_state_e state, enum flight_substate_e substate) {
  detector->state = state;
  detector->substate = substate;
}

/**
 * Set the elevation (landing altitude) of the detector. The detector will take a reading after a certain amount 
 * of time if the elevation isn't set
 *
 * @param detector The detector to use
 * @param elevation The elevation to set, which should be collected anytime the rocket is in a new location
 */
void detector_set_elevation(struct detector *detector, float elevation) {
  detector->elevation_set = 1;
  detector->elevation = elevation;
}
