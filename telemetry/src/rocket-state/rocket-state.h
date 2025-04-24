#ifndef _ROCKET_STATE_H_
#define _ROCKET_STATE_H_

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

/* Enum representing the current flight state. */

enum flight_state_e {
  STATE_IDLE,     /* Rocket is idle on the pad. */
  STATE_AIRBORNE, /* Rocket is in the air. */
  STATE_LANDED,   /* Rocket is landed. */
};

/* State information about the rocket */

typedef struct {
  atomic_int state;          /* Flight state of the rocket. */
} rocket_state_t;

int state_init(rocket_state_t *state);
int state_set_flightstate(rocket_state_t *state,
                          enum flight_state_e flight_state);
int state_get_flightstate(rocket_state_t *state,
                          enum flight_state_e *flight_state);

#endif // _ROCKET_STATE_H_
