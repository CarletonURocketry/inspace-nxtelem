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
  float time;     /* The time in seconds since mission start */
};

struct median_filter {
  int size;
  float *sorted;
  struct circ_buffer time_ordered;
};

struct average_filter {
  struct circ_buffer buffer; /* Circular buffer */
  float sum;
};

struct alt_filter {
  struct altitude_sample median_backing_sorted[ALTITUDE_MEDIAN_FILTER_SIZE];
  struct altitude_sample median_backing_time_ordered[ALTITUDE_MEDIAN_FILTER_SIZE];
  struct altitude_sample average_backing[ALTITUDE_AVERAGE_FILTER_SIZE];
  struct median_filter median;   /* Median filter for altitude */
  struct average_filter average; /* Average filter for altitude */

};

struct accel_sample {
  float acceleration; /* The acceleration in m/s^2 */
  float time;         /* The time in seconds since mission start */
};

struct accel_filter {
  struct accel_sample median_backing_sorted[ACCEL_MEDIAN_FILTER_SIZE];
  struct accel_sample median_backing_time_ordered[ACCEL_MEDIAN_FILTER_SIZE];
  struct accel_sample average_backing[ACCEL_AVERAGE_FILTER_SIZE];
  struct median_filter median;   /* Median filter for acceleration */
  struct average_filter average; /* Average filter for acceleration */
};

/* Information related to detecting liftoff or landing */
struct detector {
  struct alt_filter alts;     /* Filtering for altitude data */
  struct accel_filter accels; /* Filtering for acceleration data */

  uint64_t current_time;
  uint64_t last_alt_update;
  uint64_t last_accel_update;
  float current_alt;
  float current_accel;
  
  float landed_alt;           /* The altitude at landing or a sensible default */
  float abs_max_altitude;     /* The maximum altitude ever recorded */
  float alt_window_max;       /* The maximum altitude recently recorded */
  float alt_window_min;       /* The minimum altitude recently recorded */
  float alt_window_duration;  /* The minimum of the timestamps of recent_min/max_alt */
};

void detector_init(struct detector *detector);
void detector_add_alt(struct detector *detector, struct altitude_sample *sample);
float detector_get_alt(struct detector *detector);
void detector_add_accel(struct detector *detector, struct accel_sample *sample);
float detector_get_accel(struct detector *detector);
enum detector_event detector_detect(struct detector *detector, enum flight_state_e state);

void median_filter_init(struct median_filter *filter, float *sorted, float *time_ordered, int size);
float median_filter_add(struct median_filter *filter, float new_value);

void average_filter_init(struct average_filter *filter, float *buffer, int size);
float average_filter_add(struct average_filter *filter, float new_value);

#endif /* _DETECTOR_H_ */
