#ifndef _ROCKET_STATE_H_
#define _ROCKET_STATE_H_

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

/* Enum representing the current flight state. */

enum flight_state_e {
    STATE_IDLE,     /* The rocket is idle on the pad. */
    STATE_AIRBORNE, /* The rocket is in the air. */
    STATE_LANDED,   /* The rocket is landed. */
};

/* Enum representing the substate of the current flight state, if applicable */

enum flight_substate_e {
    SUBSTATE_UNKNOWN, /* The flight substate is unknown. */
    SUBSTATE_ASCENT,  /* The rocket is ascending. */
    SUBSTATE_DESCENT, /* The rocket is descending. */
};

/* State information about the rocket */

typedef struct {
    atomic_int state;    /* Flight state of the rocket. */
    atomic_int substate; /* Flight substate of the rocket. */
} rocket_state_t;

int state_init(rocket_state_t *state);

int state_set_flightstate(rocket_state_t *state, enum flight_state_e flight_state);
int state_get_flightstate(rocket_state_t *state, enum flight_state_e *flight_state);

int state_set_flightsubstate(rocket_state_t *state, enum flight_substate_e flight_substate);
int state_get_flightsubstate(rocket_state_t *state, enum flight_substate_e *flight_substate);

#endif // _ROCKET_STATE_H_
