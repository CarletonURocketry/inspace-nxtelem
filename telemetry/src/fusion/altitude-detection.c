#include <stdint.h>
#include <stdlib.h>

#include "altitude-detection.h"

/* The minimum number of samples to use in an average */
#define MIN_SAMPLES_FOR_AVERAGE 10

/* The period for averaging samples in milliseconds */
#define ALTITUDE_AVERAGE_PERIOD 1000

/* Altitude change requirement for airborne event in meters */
#define AIRBORNE_ALTITUDE_DELTA_THRESHOLD 50

/* Time change requirement for airborne event in milliseconds */
#define AIRBORNE_TIME_DELTA_THRESHOLD 2000

/* Landed altitude change requirement (must be less than) in meters*/
#define LANDED_ALTITUDE_DELTA_THRESHOLD 2

/* Time change requirement for landed state in milliseconds */
#define LANDED_TIME_DELTA_THRESHOLD 2000

/* Altitude above which landing event won't be considered */
#define LANDED_ALTITUDE_FLOOR 100

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
  if (last_average != NULL) {
    if (sample->timestamp - last_average->timestamp < ALTITUDE_AVERAGE_PERIOD) {
      return records->window_buffer.size > MIN_SAMPLES_FOR_AVERAGE;
    }
  }
  return 1;
}

/**
 * Creates a new average from the samples in the window buffer
 * @param records The altitude records structure to create the average from
 * @return The new average 
 */
static struct altitude_average new_average(struct altitude_records *records) {
  struct circ_iterator it;
  circ_iterator_init(&it, &records->window_buffer);

  struct altitude_sample *sample;
  struct altitude_average output;
  output.num_samples = 0;
  while ((sample = circ_iterator_next(&it)) != NULL) {
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
 * Detects if the rocket is airborne, using the configured thresholds
 * @param records The altitude records structure to check
 * @return 1 if the rocket is airborne, 0 otherwise
 */
static int detect_airborne(struct altitude_records *records) {
  struct circ_iterator it;
  circ_iterator_init(&it, &records->averages_buffer);
  struct altitude_average *last = circ_iterator_next(&it);
  if (last == NULL) {
    return 0;
  }
  struct altitude_average *cmp;
  while ((cmp = circ_iterator_next(&it)) != NULL) {
    int altitude_criteria = (last->altitude - cmp->altitude) > AIRBORNE_ALTITUDE_DELTA_THRESHOLD;
    int time_criteria = (last->timestamp - cmp->timestamp) > AIRBORNE_TIME_DELTA_THRESHOLD;
    if (altitude_criteria && time_criteria) {
      return 1;
    }
  }
  return 0;
}

/**
 * Detects if the rocket is landed, using the configured thresholds
 * @param records The altitude records structure to check
 * @return 1 if the rocket is landed, 0 otherwise
 */
static int detect_landed(struct altitude_records *records) {
  struct circ_iterator it;
  circ_iterator_init(&it, &records->averages_buffer);
  struct altitude_average *last = circ_iterator_next(&it);
  if (last == NULL) {
    return 0;
  }
  struct altitude_average *cmp;
  while ((cmp = circ_iterator_next(&it)) != NULL) {
    int altitude_criteria = (last->altitude - cmp->altitude) < LANDED_ALTITUDE_DELTA_THRESHOLD;
    int time_criteria = (last->timestamp - cmp->timestamp) > LANDED_TIME_DELTA_THRESHOLD;
    int abs_altitude_criteria = cmp->altitude < LANDED_ALTITUDE_FLOOR;
    if (altitude_criteria && time_criteria && abs_altitude_criteria) {
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
    circ_buffer_append(&records->averages_buffer, &avg);
    // Only want to consider detecting landing if we definitely aren't airborne
    if (detect_airborne(records)) {
      records->last_event = FUSION_AIRBORNE_EVENT;
    } else if (detect_landed(records)) {
      records->last_event = FUSION_LANDING_EVENT;
    }
  }
}
