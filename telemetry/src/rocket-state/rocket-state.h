#ifndef _ROCKET_STATE_H_
#define _ROCKET_STATE_H_

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

/* Enum representing the current flight state. */

enum flight_state_e {
  STATE_IDLE,     /* The rocket is idle on the pad. */
  STATE_AIRBORNE, /* The rocket is in the air. */
  STATE_LANDED,   /* The rocket is landed. */
};

enum flight_substate_e {
  SUBSTATE_UNKNOWN,  /* The flight substate is unknown. */
  SUBSTATE_ASCENT,   /* The rocket is ascending. */
  SUBSTATE_DESCENT,  /* The rocket is descending. */
};

/* The default elevation in millimeters to assume is ground level */
#define DEFAULT_ELEVATION_MM 1189000 /* The approximate elevation of Las Cruces, New Mexico */

/* State information about the rocket */

typedef struct {
  atomic_int state;               /* Flight state of the rocket. */
  atomic_int substate;            /* Flight substate of the rocket. */
  atomic_int_least32_t elevation; /* The measured altitude above sea level when at at ground level, in millimeters (signed in case of a constant error) */
} rocket_state_t;

int state_init(rocket_state_t *state);

int state_set_flightstate(rocket_state_t *state);
int state_get_flightstate(rocket_state_t *state);

int state_set_flightsubstate(rocket_state_t *state,
                             enum flight_substate_e flight_substate);
int state_get_flightsubstate(rocket_state_t *state,
                             enum flight_substate_e *flight_substate);

int state_set_elevation(rocket_state_t *state, int32_t elevation);
int state_get_elevation(rocket_state_t *state, int32_t *elevation);

// TODO - might need to add more mutual exclusion checks to stop writes with old information from happening
  // It could be possible that multiple state updates happen at the same time
  // causing multiple writes to happen, each with slightly different states.
  // Just check this isn't possible


#endif // _ROCKET_STATE_H_
