#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#include "../rocket-state/rocket-state.h"
#include "transmit.h"

void *transmit_main(void *arg) {

  int err;
  int radio; /* Radio device file descriptor */
  ssize_t written;
  rocket_state_t *state = ((struct transmit_args_t *)(arg))->state;
  const char *radio_dev = ((struct transmit_args_t *)(arg))->radio_dev;

  /* Get access to radio */
  radio = open(radio_dev, O_WRONLY);
  if (radio < 0) {
    pthread_exit(EXIT_FAILURE); // TODO: handle more gracefully
  }

  /* Transmit forever */

  for (;;) {
    err = state_wait_for_change(state); // TODO handle error

    err = state_read_lock(state); // TODO: handle error

    /* Encode radio data into packet */

    /* Transmit radio data; TODO right now just transmitting state struct */
    written = write(radio, &state->state, sizeof(state->state));
    if (written == -1) {
      // TODO: handle error in errno
    }

    err = state_unlock(state); // TODO: handle error
  }

  close(radio);
  pthread_exit(0);
}
