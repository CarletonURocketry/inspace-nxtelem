#include <pthread.h>
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "rocket-state.h"

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
static const char *FLIGHT_STATES[] = {
    [STATE_IDLE] = "STATE_IDLE",
    [STATE_LANDED] = "STATE_LANDED",
    [STATE_AIRBORNE] = "STATE_AIRBORNE",
};
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/*
 * Gets the current flight state stored in NV storage.
 * TODO: tell this where to get flight state
 */
static enum flight_state_e get_flight_state(void) {
  // TODO: real implementation
  return STATE_IDLE;
}

/* Initialize the rocket state monitor
 * @param state The rocket state to initialize
 * @param flight_state The flight state that the rocket is currently in.
 * @return 0 on success, error code on failure
 */
int state_init(rocket_state_t *state) {

  int err;

  state->waiting = 0;
  state->changed = false;
  state->state = get_flight_state();

  err = pthread_mutex_init(&state->wait_lock, NULL);
  if (err)
    return err;

  err = pthread_rwlock_init(&state->rw_lock, NULL);
  if (err)
    return err;

  err = pthread_cond_init(&state->change, NULL);
  return err;
}

/* Signal that the rocket state has changed to all waiting threads.
 * @param state The rocket state for which to broadcast a change
 * @return 0 on success, error code on failure
 */
int state_signal_change(rocket_state_t *state) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("State change signalled.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  state->changed = true;
  return pthread_cond_broadcast(&state->change);
}

/* Blocking wait until the rocket state has changed.
 * @param state The rocket state on which to wait for a change
 * @return 0 on success, error code on failure
 */
int state_wait_for_change(rocket_state_t *state) {

  int err;

  /* Lock mutex for wait_blocking */

  err = pthread_mutex_lock(&state->wait_lock);
  if (err)
    return err;

  /* Wait for condition */

  state->waiting++; /* Record that we are waiting */

  while (!state->changed) {
    pthread_cond_wait(&state->change, &state->wait_lock);
  }

  /* If we are here, the state has changed and condvar was signalled. */

  state->waiting--; /* We have stopped waiting */

  /* Mark state as unchanged once all threads have finished waiting. */
  if (state->waiting == 0) {
    state->changed = false;
  }

  /* Release immediately */
  return pthread_mutex_unlock(&state->wait_lock);
}

/* Lock the rocket state for writing.
 * @param state The rocket state to lock
 * @return 0 on success, error code on failure
 */
int state_write_lock(rocket_state_t *state) {
  return pthread_rwlock_wrlock(&state->rw_lock);
}

/* Lock the rocket state for reading.
 * @param state The rocket state to lock
 * @return 0 on success, error code on failure
 */
int state_read_lock(rocket_state_t *state) {
  return pthread_rwlock_rdlock(&state->rw_lock);
}

/* Unlock the rocket state from reading or writing.
 * @param state The rocket state to unlock
 * @return 0 on success, error code on failure
 */
int state_unlock(rocket_state_t *state) {
  return pthread_rwlock_unlock(&state->rw_lock);
}

/*
 * Set the flight state in NV storage and state object (write-through)
 * @param state The rocket state to modify
 * @param flight_state The rocket's new flight state to set.
 * @return 0 on success, error code on failure
 */
int state_set_flightstate(rocket_state_t *state,
                          enum flight_state_e flight_state) {
  // TODO: set for real in NV-storage
  int err;
  err = state_write_lock(state);
  state->state = flight_state;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Flight state changed to %s\n", FLIGHT_STATES[flight_state]);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  err = state_unlock(state);
  return err;
}

/*
 * Get the flight state atomically.
 * @param state The rocket state to get flight state from.
 * @param flight_state A pointer in which to hold the flight state.
 * @return 0 on success, error code on failure
 */
int state_get_flightstate(rocket_state_t *state,
                          enum flight_state_e *flight_state) {
  int err;
  err = state_read_lock(state);
  *flight_state = state->state;
  err = state_unlock(state);
  return err;
}
