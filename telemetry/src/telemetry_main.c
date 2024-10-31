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

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  puts("You are running the Carleton University InSpace telemetry system.");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  state_init(&state); /* Initialize shared state */

  /* Initialize thread arguments */

  struct logging_args_t log_args = {
      .state = &state,
      .flight_storage = CONFIG_INSPACE_TELEMETRY_FLIGHT_FS,
      .landing_storage = CONFIG_INSPACE_TELEMETRY_LANDED_FS,
  };

  struct transmit_args_t transmit_args = {
      .state = &state,
      .radio_dev = "./radio.bin",
  };

  /* Start all threads */

  // TODO: handle thread creation errors better
  err = pthread_create(&collect_thread, NULL, collection_main, &state);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem starting collection thread: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    exit(EXIT_FAILURE);
  }

  err = pthread_create(&log_thread, NULL, logging_main, &log_args);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem starting logging thread: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    exit(EXIT_FAILURE);
  }

  err = pthread_create(&transmit_thread, NULL, transmit_main, &transmit_thread);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem starting transmission thread: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    exit(EXIT_FAILURE);
  }

  /* Join on all threads: TODO handle errors */

  err = pthread_join(transmit_thread, NULL);
  err = pthread_join(collect_thread, NULL);
  err = pthread_join(log_thread, NULL);

  return err;
}
