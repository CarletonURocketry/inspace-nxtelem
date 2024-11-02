#include <pthread.h>
#include <sched.h>
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

  /* Initialize shared state */

  state_init(&state);

  /* Start all threads */

  // TODO: handle thread creation errors better
  err = pthread_create(&collect_thread, NULL, collection_main, &state);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem starting collection thread: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    exit(EXIT_FAILURE);
  }

  err = pthread_create(&transmit_thread, NULL, transmit_main, &state);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem starting transmission thread: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    exit(EXIT_FAILURE);
  }

  int policy;
  struct sched_param param;
  err = pthread_getschedparam(transmit_thread, &policy, &param);

  err = pthread_setschedparam(transmit_thread, SCHED_FIFO, &param);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem setting transmit thread scheduling policy: %d\n",
            err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  err = pthread_create(&log_thread, NULL, logging_main, &state);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem starting logging thread: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    exit(EXIT_FAILURE);
  }
  err = pthread_setschedparam(log_thread, SCHED_FIFO, &param);
  if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Problem setting logging thread scheduling policy: %d\n",
            err);
#endif /* defined                                                              \
CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  /* Join on all threads: TODO handle errors */

  err = pthread_join(collect_thread, NULL);
  err = pthread_join(transmit_thread, NULL);
  err = pthread_join(log_thread, NULL);

  return err;
}
