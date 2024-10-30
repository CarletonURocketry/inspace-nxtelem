#ifndef _ROCKET_STATE_H_
#define _ROCKET_STATE_H_

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

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
  struct rocket_t state;     /* State of the rocket, being protected */
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

#endif // _ROCKET_STATE_H_
