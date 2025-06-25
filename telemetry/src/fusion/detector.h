#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include "circular-buffer.h"
#include "../rocket-state/rocket-state.h"

#define ALTITUDE_MEDIAN_FILTER_SIZE 5
#define ALTITUDE_AVERAGE_FILTER_SIZE 10

#define ACCEL_MEDIAN_FILTER_SIZE 5
#define ACCEL_AVERAGE_FILTER_SIZE 10


/* Detection events */
enum detector_event {
  DETECTOR_NO_EVENT,       /* No event has occured */
  DETECTOR_AIRBORNE_EVENT, /* The rocket is airborne */
  DETECTOR_APOGEE_EVENT,   /* The rocket has reached its maximum height */
  DETECTOR_LANDING_EVENT   /* The rocket is landed*/
};

struct altitude_sample {
  float altitude; /* The altitude in meters */
  uint64_t time;  /* The time in microseconds */
};

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

struct alt_filter {
  float median_backing_sorted[ALTITUDE_MEDIAN_FILTER_SIZE];
  float median_backing_time_ordered[ALTITUDE_MEDIAN_FILTER_SIZE];
  float average_backing[ALTITUDE_AVERAGE_FILTER_SIZE];
  struct median_filter median;   /* Median filter for altitude */
  struct average_filter average; /* Average filter for altitude */
};

struct accel_sample {
  float acceleration; /* The acceleration in m/s^2 */
  uint64_t time;      /* The time in microseconds */
};

struct accel_filter {
  float median_backing_sorted[ACCEL_MEDIAN_FILTER_SIZE];
  float median_backing_time_ordered[ACCEL_MEDIAN_FILTER_SIZE];
  float average_backing[ACCEL_AVERAGE_FILTER_SIZE];
  struct median_filter median;   /* Median filter for acceleration */
  struct average_filter average; /* Average filter for acceleration */
};

/* Information related to detecting liftoff or landing */
struct detector {
  struct alt_filter alts;     /* Filtering for altitude data */
  struct accel_filter accels; /* Filtering for acceleration data */

  uint64_t init_time;          /* The time the detector was first used in microseconds*/
  uint64_t current_time;       /* The time of the most recent update to the detector in microseconds */
  uint64_t last_alt_update;    /* The time the guess of the current altitude was made in microseconds */
  uint64_t last_accel_update;  /* The time the guess of the current accleration was made in microseconds */
  float current_alt;           /* Best guess of the current altitude in meters */
  float current_accel;         /* Best guess of the current acceleration in meters */

  float apogee;               /* The maximum altitude ever recorded */
  uint64_t apogee_time;       /* The time the maximum altitude was recorded in microseconds */

  float alt_window_max;       /* The maximum altitude recently recorded */
  float alt_window_min;       /* The minimum altitude recently recorded */
  float alt_window_duration;  /* The minimum of the timestamps of recent_min/max_alt */

  int elevation_set;               /* If the elevation has been set or not */
  float elevation;                 /* The altitude at landing or a sensible default in meters */
  enum flight_state_e state;       /* The flight state */
  enum flight_substate_e substate; /* The flight substate */
};

void detector_init(struct detector *detector, uint64_t time);
void detector_add_alt(struct detector *detector, struct altitude_sample *sample);
void detector_add_accel(struct detector *detector, struct accel_sample *sample);
float detector_get_alt(struct detector *detector);
float detector_get_accel(struct detector *detector);
void detector_set_state(struct detector *detector, enum flight_state_e state, enum flight_substate_e substate);
void detector_set_elevation(struct detector *detector, float elevation);
enum detector_event detector_detect(struct detector *detector);


#endif /* _DETECTOR_H_ */
