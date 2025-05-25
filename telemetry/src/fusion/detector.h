#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include "filter.h"
#include "../rocket-state/rocket-state.h"

/* Detection events */
enum detector_event {
  DETECTOR_NO_EVENT,       /* No event has occured */
  DETECTOR_AIRBORNE_EVENT, /* The rocket is airborne */
  DETECTOR_LANDING_EVENT   /* The rocket is landed*/
};

/* Information related to detecting liftoff or landing */
struct detector {};

void detector_init(struct detector *detector);
enum detector_event detector_detect(struct detector *detector, enum flight_state_e state, struct rocket_dynamics *dynamics); 

#endif /* _DETECTOR_H_ */
