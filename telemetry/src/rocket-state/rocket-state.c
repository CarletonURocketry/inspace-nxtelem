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

/* Initialize the barrier object.
 * @param barrier A reference to the barrier to initialize.
 * @return 0 on success, error code on failure.
 */
static int barrier_init(struct barrier_t *barrier) {
  int err;
  barrier->version = 0;
  err = pthread_mutex_init(&barrier->lock, NULL);
  if (err)
    return err;
  return pthread_cond_init(&barrier->change, NULL);
}

/* Unblock all threads currently waiting on the barrier.
 * @param barrier The barrier to broadcast on
 * @return 0 for success, error code on failure
 */
static int barrier_signal_change(struct barrier_t *barrier) {

  int err;

  /* Increment the version number of the data. */

  err = pthread_mutex_lock(&barrier->lock);
  if (err)
    return err;

  barrier->version++;

  err = pthread_mutex_unlock(&barrier->lock);
  if (err)
    return err;

    /* Allow all currently waiting threads to pass */

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("State change signalled.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  return pthread_cond_broadcast(&barrier->change);
}

/* Block on the barrier until a change is signalled.
 * @param barrier The barrier to block on.
 * @param version The last version of data the caller has seen
 * @return 0 for success, error code on failure.
 */
static int barrier_wait_for_change(struct barrier_t *barrier,
                                   uint32_t *version) {

  int err;

  /* Lock mutex for wait_blocking */

  err = pthread_mutex_lock(&barrier->lock);
  if (err)
    return err;

  /*
   * If the latest version of the data matches is less than or equal to the last
   * version we saw, we should block and wait for something new that we haven't
   * seen before.
   */

  while (barrier->version <= *version) {
    pthread_cond_wait(&barrier->change, &barrier->lock);
  }

  /* If we are here, the state has changed and we have passed the barrier.
   * Make sure we update our most recently seen version, then release the lock
   * so we can do something with the newly changed state.
   */

  *version = barrier->version;
  return pthread_mutex_unlock(&barrier->lock);
}

/* Initialize the rocket state monitor
 * @param state The rocket state to initialize
 * @param flight_state The flight state that the rocket is currently in.
 * @return 0 on success, error code on failure
 */
int state_init(rocket_state_t *state) {

  int err;

  atomic_store(&state->state, get_flight_state());

  err = barrier_init(&state->barrier);
  if (err)
    return err;

  err = pthread_rwlock_init(&state->rw_lock, NULL);
  return err;
}

/* Signal that the rocket state has changed to all waiting threads.
 * @param state The rocket state for which to broadcast a change
 * @return 0 on success, error code on failure
 */
int state_signal_change(rocket_state_t *state) {
  return barrier_signal_change(&state->barrier);
}

/* Blocking wait until the rocket state has changed.
 * @param state The rocket state on which to wait for a change
 * @param version The last version of the data seen by the caller
 * @return 0 on success, error code on failure
 */
int state_wait_for_change(rocket_state_t *state, uint32_t *version) {
  return barrier_wait_for_change(&state->barrier, version);
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
  atomic_store(&state->state, flight_state);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Flight state changed to %s\n", FLIGHT_STATES[flight_state]);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  return 0;
}

/*
 * Get the flight state atomically.
 * @param state The rocket state to get flight state from.
 * @param flight_state A pointer in which to hold the flight state.
 * @return 0 on success, error code on failure
 */
int state_get_flightstate(rocket_state_t *state,
                          enum flight_state_e *flight_state) {
  *flight_state = atomic_load(&state->state);
  return 0;
}
