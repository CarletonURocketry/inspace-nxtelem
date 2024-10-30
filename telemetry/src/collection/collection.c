#include <pthread.h>
#include <time.h>

#include "../rocket-state/rocket-state.h"
#include "collection.h"

/*
 * Get the time difference from start until now in milliseconds.
 * @param start The start time.
 * @return The time elapsed since start in milliseconds.
 */
static uint32_t ms_since_on(struct timespec *start) {

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  struct timespec diff = {
      .tv_sec = (now.tv_sec - start->tv_sec),
      .tv_nsec = now.tv_nsec - start->tv_nsec,
  };

  if (diff.tv_nsec < 0) {
    diff.tv_nsec += 1e9;
    diff.tv_nsec--;
  }

  return diff.tv_sec * 1000 + (diff.tv_nsec / 1e6);
}

/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {

  int err;
  struct timespec start_time;
  rocket_state_t *state = (rocket_state_t *)(arg);

  /* Get startup time */
  clock_gettime(CLOCK_MONOTONIC, &start_time);

  for (;;) {

    /* Collect data; TODO */

    /* Put data in the state structure */
    err = state_write_lock(state);
    if (err)
      continue;

    state->state.temp++; // TODO: remove and replace with real data
    state->state.time = ms_since_on(&start_time); /* Measurement time */

    err = state_unlock(state);
    // TODO: handle error

    err = state_signal_change(state);
    // TODO: handle error
  }

  pthread_exit(0);
}
