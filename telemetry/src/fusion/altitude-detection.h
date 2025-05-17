#ifndef _ALTITUDE_DETECTION_H_
#define _ALTITUDE_DETECTION_H_

#include <nuttx/sensors/sensor.h>

#include "circular-buffer.h"

/* Detection events */
enum fusion_events {
  FUSION_NO_EVENT,       /* No event has occured */
  FUSION_AIRBORNE_EVENT, /* The rocket is airborne */
  FUSION_LANDING_EVENT   /* The rocket is landed*/
};

/* The number of samples to take to use in averages */
#define ALTITUDE_WINDOW_SIZE 1000

/* The number of altitude averages to keep */
#define ALTITUDE_AVERAGES_SIZE 5

/* A single measurement of altitude */
struct altitude_sample {
  uint64_t timestamp; /* The timestamp in microseconds */
  float altitude;     /* The altitude in meters */
};

/* An average of altitude measurements */
struct altitude_average {
  uint64_t timestamp; /* The average timestamp in microseconds */
  float altitude;     /* The average altitude in meters */
  int num_samples;    /* The number of samples making up this average */
};

/* Information related to detecting liftoff or landing */
struct altitude_records {
  enum fusion_events last_event;                            /* The last event that occured when a record was added*/
  struct altitude_sample window[ALTITUDE_WINDOW_SIZE];      /* Backing array for window_buffer */ 
  struct altitude_average averages[ALTITUDE_AVERAGES_SIZE]; /* Backing array for averages_buffer */ 
  struct circ_buffer window_buffer;                         /* A moving window of altitude samples */
  struct circ_buffer averages_buffer;                       /* A moving window of altitude averages */
};

void init_records(struct altitude_records *records);
void add_sample(struct altitude_records *records, struct altitude_sample *sample);
enum fusion_events last_event(struct altitude_records *records);

#endif /* _ALTITUDE_DETECTION_H_ */
