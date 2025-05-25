#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "detector.h"

#define DETECTION_DEBUG

#if defined(DETECTION_DEBUG)
#include <stdio.h>
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

struct detection_criteria {
  float max_vel;        /* Max abs rate of change of altitude for this event in m/s */
  float min_vel;        /* Min abs rate of change of altitude for this event in m/s */
  float min_altitude;   /* The minimum altitude this event can be detected at in m*/
  float max_altitude;   /* The maximum altitude this event can be detected at in m*/
};

const struct detection_criteria airborne_thresholds = {
  .min_vel = 25.0,
  .max_vel = FLT_MAX,
  .min_altitude = -FLT_MAX,
  .max_altitude = FLT_MAX
};

const struct detection_criteria landed_thresholds = {
  .min_vel = -FLT_MAX,
  .max_vel = 0.5,
  .min_altitude = -FLT_MAX,
  .max_altitude = 100.0
};

void detector_init(struct detector *detector) {
  return;
}

static int criteria_met(struct detection_criteria *criteria, struct rocket_dynamics *dynamics) {
  int test = dynamics->velocity > criteria->min_vel && dynamics->velocity < criteria->max_vel;
  test &= dynamics->altitude > criteria->min_altitude && dynamics->altitude < criteria->max_altitude;
  return test;
}

/**
 * Detects the occurence of events based on the current flight state and dynamics
 * @param detector The detector instance, currently unused (kept around in case state information is needed for events)
 * @param state The current flight state of the rocket
 * @param dynamics The current dynamics of the rocket, including altitude and velocity
 * @return The detected event, or DETECTOR_NO_EVENT if none was detected
 */
enum detector_event detector_detect(struct detector *detector, enum flight_state_e state, struct rocket_dynamics *dynamics) {
  // Don't use data that has not been updated at all
  if (dynamics->last_update == 0) {
    return DETECTOR_NO_EVENT;
  }
  if (state == STATE_IDLE) {
    if (criteria_met(&airborne_thresholds, dynamics)) {
      DEBUG_PRINT("Detected airborne event at altitude %f m and velocity %f m/s\n", dynamics->altitude, dynamics->velocity);
      return DETECTOR_AIRBORNE_EVENT;
    }
  } else if (state == STATE_AIRBORNE) {
    if (criteria_met(&landed_thresholds, dynamics)) {
      DEBUG_PRINT("Detected landing event at altitude %f m and velocity %f m/s\n", dynamics->altitude, dynamics->velocity);
      return DETECTOR_LANDING_EVENT;
    }
  }
  return DETECTOR_NO_EVENT;
}
