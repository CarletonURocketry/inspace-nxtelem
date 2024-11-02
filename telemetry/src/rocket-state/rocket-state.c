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
  barrier->waiting = 0;
  barrier->signalled = 0;
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

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("State change signalled.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  /* Allow all currently waiting threads to pass */

  pthread_mutex_lock(&barrier->lock);
  barrier->signalled = barrier->waiting;
  pthread_mutex_unlock(&barrier->lock);

  return pthread_cond_broadcast(&barrier->change);
}

/* Block on the barrier until a change is signalled.
 * @param barrier The barrier to block on.
 * @return 0 for success, error code on failure.
 */
static int barrier_wait_for_change(struct barrier_t *barrier) {

  int err;

  /* Lock mutex for wait_blocking */

  err = pthread_mutex_lock(&barrier->lock);
  if (err)
    return err;

  barrier->waiting++; /* Record that we are waiting */

  /* Here we block inside a loop which prevents us from waking up spuriously.
   *
   * If the number of currently waiting threads is not equal to the number of
   * signalled threads, this is because one of the threads who was already
   * signalled came back around to wait again. It should block.
   *
   * If the number of waiting threads is equal to the number of signalled
   * threads, we know that any thread who was waiting here passed the barrier
   * and decremented the number of waiting and signalled threads to keep them
   * equal. When the condvar is first signalled, the producer sets `waiting`
   * equal to `signalled`, so these numbers start equal.
   *
   * That means that a thread who has come around to block twice in a row will
   * get trapped by this condition until all other waiting threads have passed
   * the barrier.
   *
   * This synchronization method allows consumers of different speeds to consume
   * the same data. If a really fast consumer is keeping up with the producer
   * while the slow consumer is still doing processing, `waiting` will only be
   * 1. This means the fast consumer does not have to wait for the slow one to
   * continue consuming new data. Whenever the slow one is done processing, it
   * will also increment wait. Now wait will be 2, both consumers will be
   * signalled when a change happens, and they will both get a chance to consume
   * the underlying data.
   */
  while (barrier->waiting != barrier->signalled) {
    pthread_cond_wait(&barrier->change, &barrier->lock);
  }

  /* If we are here, the state has changed and we have passed the barrier. */

  barrier->waiting--;
  barrier->signalled--;

  /* Release the lock so we can do something with the newly changed state */

  return pthread_mutex_unlock(&barrier->lock);
}

/* Initialize the rocket state monitor
 * @param state The rocket state to initialize
 * @param flight_state The flight state that the rocket is currently in.
 * @return 0 on success, error code on failure
 */
int state_init(rocket_state_t *state) {

  int err;

  state->state = get_flight_state();

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
 * @return 0 on success, error code on failure
 */
int state_wait_for_change(rocket_state_t *state) {
  return barrier_wait_for_change(&state->barrier);
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
