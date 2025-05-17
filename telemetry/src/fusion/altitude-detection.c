#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "altitude-detection.h"

#define DETECTION_DEBUG

#if defined(DETECTION_DEBUG)
#include <stdio.h>
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

/* The minimum number of samples to use in an average */
#define MIN_SAMPLES_FOR_AVERAGE 10

/* The period for averaging samples in milliseconds */
#define ALTITUDE_AVERAGE_PERIOD 500

struct altitude_thresholds {
  float max_roc;        /* Max abs rate of change of altitude for this event in m/s */
  float min_roc;        /* Min abs rate of change of altitude for this event in m/s */
  float time_threshold; /* The minimum time difference before an event can be detected in s*/
  float min_altitude;   /* The minimum altitude this event can be detected at in m*/
  float max_altitude;   /* The maximum altitude this event can be detected at in m*/
};

const struct altitude_thresholds airborne_thresholds = {
  .min_roc = 25.0,
  .max_roc = FLT_MAX,
  .time_threshold = 2.0,
  .min_altitude = -FLT_MAX,
  .max_altitude = FLT_MAX
};

const struct altitude_thresholds landed_thresholds = {
  .min_roc = -FLT_MAX,
  .max_roc = 0.5,
  .time_threshold = 2.0,
  .min_altitude = -FLT_MAX,
  .max_altitude = 100.0
};

/**
 * Returns the last event that was detected
 * @param records The altitude records structure to get the last event from
 * @return The last event that was detected, or FUSION_NO_EVENT if no event was detected
 */
enum fusion_events last_event(struct altitude_records *records) {
  enum fusion_events event = records->last_event;
  records->last_event = FUSION_NO_EVENT;
  return event;
}

/**
 * Checks if a new average should be created
 * @param records The altitude records structure to check
 * @param sample The most recent sample, not put in the structure yet
 * @return 1 if a new average should be created, 0 otherwise
 */
static int need_new_average(struct altitude_records *records, struct altitude_sample *sample) {
  struct altitude_average *last_average = circ_buffer_get(&records->averages_buffer);
  if (last_average == NULL) {
    DEBUG_PRINT("Need new average because we have none\n");
    return 1;
  }
  if ((sample->timestamp - last_average->timestamp) > (ALTITUDE_AVERAGE_PERIOD * 1000)) {
    return circ_buffer_size(&records->window_buffer) > MIN_SAMPLES_FOR_AVERAGE;
  }
  return 0;
}

/**
 * Creates a new average from the samples in the window buffer
 * @param records The altitude records structure to create the average from
 * @return The new average 
 */
static struct altitude_average new_average(struct altitude_records *records) {
  struct altitude_sample *sample;
  struct altitude_average output = {0, 0, 0};
  while ((sample = circ_buffer_pop(&records->window_buffer)) != NULL) {
    output.altitude += sample->altitude;
    output.timestamp += sample->timestamp;
    ++output.num_samples;
  }
  if (output.num_samples > 0) {
    output.altitude /= output.num_samples;
    output.timestamp /= output.num_samples;
  }
  return output;
}

/**
 * Checks if the records currently meet the criteria stored in the thresholds struct
 * @param records The altitude records structure to check
 * @param thresholds The thresholds to check against, which may include rate of change or time requirements
 * @return 1 if the thresholds are all met, 0 otherwise
 */
static int detect_event(struct altitude_records *records, struct altitude_thresholds const *thresholds) {
  struct circ_iterator it;
  circ_iterator_init(&it, &records->averages_buffer);
  struct altitude_average *last = circ_iterator_next(&it);
  if (last == NULL) {
    return 0;
  }
  struct altitude_average *cmp;
  while ((cmp = circ_iterator_next(&it)) != NULL) {
    float altitude_delta = fabsf(last->altitude - cmp->altitude);
    float time_delta = (last->timestamp - cmp->timestamp) / 1000000.0;
    DEBUG_PRINT("Time delta: %f, ROC: %f\n", time_delta, altitude_delta / time_delta); 
    int test = time_delta > thresholds->time_threshold;
    test &= (altitude_delta / time_delta) > thresholds->min_roc;
    test &= (altitude_delta / time_delta) < thresholds->max_roc;
    test &= last->altitude > thresholds->min_altitude;
    test &= last->altitude < thresholds->max_altitude;
    if (test) {
      DEBUG_PRINT("Detected event\n");
      return 1;
    }
  }
  return 0;
}

/**
 * Initalizes the altitude records structure
 * @param records The altitude records structure to initialize
 */
void init_records(struct altitude_records *records) {
  records->last_event = FUSION_NO_EVENT;
  circ_buffer_init(&records->window_buffer, records->window, ALTITUDE_WINDOW_SIZE, sizeof(struct altitude_sample));
  circ_buffer_init(&records->averages_buffer, records->averages, ALTITUDE_AVERAGES_SIZE, sizeof(struct altitude_average));
}

/**
 * Adds a sample to the altitude records structure, performs detection if necessary
 * @param records The altitude records structure to add the sample to
 * @param sample The sample to add
 */
void add_sample(struct altitude_records *records, struct altitude_sample *sample) {
  int need_avg = need_new_average(records, sample);
  circ_buffer_append(&records->window_buffer, sample);
  if (need_avg) {
    struct altitude_average avg = new_average(records);
    DEBUG_PRINT("Got a new average of %lu:%f, %d samples\n", avg.timestamp, avg.altitude, avg.num_samples);
    circ_buffer_append(&records->averages_buffer, &avg);
    // Only want to consider detecting landing if we definitely aren't airborne
    if (detect_event(records, &airborne_thresholds)) {
      records->last_event = FUSION_AIRBORNE_EVENT;
    } else if (detect_event(records, &landed_thresholds)) {
      records->last_event = FUSION_LANDING_EVENT;
    }
  }
}
