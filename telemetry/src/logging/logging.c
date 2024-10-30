#include "logging.h"
#include <stdio.h>
#include <stdlib.h>

/*
 * Logging thread which runs to log data to the SD card.
 */
void *logging_main(void *arg) {

  size_t written;
  rocket_state_t *state = ((struct logging_args_t *)(arg))->state;
  char *loc = ((struct logging_args_t *)(arg))->storage_loc;
  int err;

  /* Open storage location in append mode */
  FILE *storage = fopen(loc, "ab");
  if (storage == NULL) {
    pthread_exit((void *)EXIT_FAILURE); // TODO: fail more gracefully
  }

  /* Infinite loop to log data */

  for (;;) {

    /* Wait for the data to have a change */

    err = state_wait_for_change(state); // TODO: handle error

    /* Log data */

    err = state_read_lock(state); // TODO: handle error

    written = fwrite(&state->data, sizeof(state->data), 1, storage);
    if (written == 0) {
      // TODO: Handle error (might happen if file got too large, start another
      // file)
    }

    err = state_unlock(state); // TODO: handle error
  }
}
