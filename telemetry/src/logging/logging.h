#ifndef _INSPACE_LOGGING_H_
#define _INSPACE_LOGGING_H_

#include "../rocket-state/rocket-state.h"

struct logging_args_t {
  rocket_state_t *state; /* The rocket state to log */
  char *flight_storage;  /* Storage location to log to in flight */
  char *landing_storage; /* Storage location to copy to when landed */
};

void *logging_main(void *arg);

#endif // _INSPACE_LOGGING_H_
