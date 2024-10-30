#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "rocket-state/rocket-state.h"

#include "collection/collection.h"
#include "logging/logging.h"
#include "transmission/transmit.h"

int main(int argc, char **argv) {

  int err = 0;
  rocket_state_t state; /* The shared rocket state. */

  /* Thread handles. */
  pthread_t transmit_thread;
  pthread_t log_thread;
  pthread_t collect_thread;

  puts("You are running the Carleton University InSpace telemetry system.");

  state_init(&state); /* Initialize shared state */

  /* Initialize thread arguments */

  struct logging_args_t log_args = {.state = &state,
                                    .storage_loc = "./test.bin"};

  struct transmit_args_t transmit_args = {.state = &state,
                                          .radio_dev = "./radio.bin"};

  /* Start all threads */

  // TODO: handle thread creation errors better
  err = pthread_create(&collect_thread, NULL, collection_main, &state);
  if (err)
    exit(EXIT_FAILURE);

  err = pthread_create(&log_thread, NULL, logging_main, &log_args);
  if (err)
    exit(EXIT_FAILURE);

  err = pthread_create(&transmit_thread, NULL, transmit_main, &transmit_thread);
  if (err)
    exit(EXIT_FAILURE);

  /* Join on all threads: TODO handle errors */

  err = pthread_join(transmit_thread, NULL);
  err = pthread_join(collect_thread, NULL);
  err = pthread_join(log_thread, NULL);

  return err;
}
