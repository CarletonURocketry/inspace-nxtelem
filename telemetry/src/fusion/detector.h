#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include "../rocket-state/rocket-state.h"
#include "filtering.h"

/* Detection events */
enum detector_event {
    DETECTOR_NO_EVENT,       /* No event has occured */
    DETECTOR_AIRBORNE_EVENT, /* The rocket is airborne */
    DETECTOR_APOGEE_EVENT,   /* The rocket has reached its maximum height */
    DETECTOR_LANDING_EVENT   /* The rocket is landed*/
};

struct accel_sample {
    uint64_t time;      /* The time in microseconds */
    float acceleration; /* The acceleration in m/s^2 */
};

struct altitude_sample {
    uint64_t time;  /* The time in microseconds */
    float altitude; /* The altitude in meters */
};

struct alt_filter {
    float median_backing_sorted[CONFIG_INSPACE_TELEMETRY_ALT_MED_FILT_SIZE];
    float median_backing_time_ordered[CONFIG_INSPACE_TELEMETRY_ALT_MED_FILT_SIZE];
    float average_backing[CONFIG_INSPACE_TELEMETRY_ALT_AVG_FILT_SIZE];
    struct median_filter median;   /* Median filter for altitude */
    struct average_filter average; /* Average filter for altitude */
};

struct accel_filter {
    float median_backing_sorted[CONFIG_INSPACE_TELEMETRY_ACCEL_MED_FILT_SIZE];
    float median_backing_time_ordered[CONFIG_INSPACE_TELEMETRY_ACCEL_MED_FILT_SIZE];
    float average_backing[CONFIG_INSPACE_TELEMETRY_ACCEL_AVG_FILT_SIZE];
    struct median_filter median;   /* Median filter for acceleration */
    struct average_filter average; /* Average filter for acceleration */
};

/* Information related to detecting liftoff or landing */
struct detector {
    struct alt_filter alts;            /* Filtering for altitude data */
    struct accel_filter accels;        /* Filtering for acceleration data */

    struct window_criteria land_alt_window;   /* Window that tracks the variation in altitude */

    uint64_t init_time;         /* The time the detector was first used in microseconds*/
    uint64_t current_time;      /* The time of the most recent update to the detector in microseconds */
    uint64_t last_alt_update;   /* The time the guess of the current altitude was made in microseconds */
    uint64_t last_accel_update; /* The time the guess of the current accleration was made in microseconds */
    float current_alt;          /* Best guess of the current altitude in meters */
    float current_accel;        /* Best guess of the current acceleration in meters */

    float apogee;         /* The maximum altitude ever recorded */
    uint64_t apogee_time; /* The time the maximum altitude was recorded in microseconds */

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
