#include "collection.h"
#include "../rocket-state/rocket-state.h"
#include <pthread.h>

/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {

  int err;
  rocket_state_t *state = (rocket_state_t *)(arg);

  for (;;) {

    /* Collect data; TODO */

    /* Put data in the state structure */
    err = state_write_lock(state);
    if (err)
      continue;

    state->state.temp++; // TODO: remove and replace with real data

    err = state_unlock(state);
    // TODO: handle error

    err = state_signal_change(state);
    // TODO: handle error
  }

  pthread_exit(0);
}
