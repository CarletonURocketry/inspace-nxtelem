#ifndef _ROCKET_STATE_H_
#define _ROCKET_STATE_H_

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

/* Enum representing the current flight state. */
enum flight_state_e {
  STATE_IDLE,     /* Rocket is idle on the pad. */
  STATE_AIRBORNE, /* Rocket is in the air. */
  STATE_LANDED,   /* Rocket is landed. */
};

/* GPS coordinates */
struct coordinate_t {
  int32_t lat; /* Latitude in microdegrees */
  int32_t lon; /* Longitude in microdegrees */
};

/* Describes the state of the rocket at any given moment */
struct rocket_t {
  uint32_t time;    /* Time of measurement in milliseconds since launch */
  int32_t temp;     /* Temperature in millidegrees Celsius */
  int32_t pressure; /* Pressure in Pascals */
  int32_t altitude; /* Altitude in millimetres */
  struct coordinate_t coords;
};

/* A monitor struct for accessing the rocket state safely between threads */
typedef struct {
  struct rocket_t data;      /* Rocket state data, being protected */
  enum flight_state_e state; /* Flight state of the rocket. */
  pthread_rwlock_t rw_lock;  /* Read-write lock for modifying state. */
  pthread_mutex_t wait_lock; /* Lock for blocking on condvar */
  pthread_cond_t change;     /* Condvar to detect state change */
  bool changed; /* Condition external to condvar to prevent spurious wakeups */
  uint8_t waiting; /* Number of threads waiting for change */
} rocket_state_t;

int state_init(rocket_state_t *state);
int state_signal_change(rocket_state_t *state);
int state_wait_for_change(rocket_state_t *state);
int state_unlock(rocket_state_t *state);
int state_read_lock(rocket_state_t *state);
int state_write_lock(rocket_state_t *state);
int state_set_flightstate(rocket_state_t *state,
                          enum flight_state_e flight_state);
int state_get_flightstate(rocket_state_t *state,
                          enum flight_state_e *flight_state);

#endif // _ROCKET_STATE_H_
