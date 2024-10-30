#include "rocket-state.h"
#include <pthread.h>

/* Initialize the rocket state monitor
 * @param state The rocket state to initialize
 * @return 0 on success, error code on failure
 */
int state_init(rocket_state_t *state) {

  int err;

  state->waiting = 0;
  state->changed = false;

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
