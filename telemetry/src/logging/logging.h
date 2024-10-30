#ifndef _INSPACE_LOGGING_H_
#define _INSPACE_LOGGING_H_

#include "../rocket-state/rocket-state.h"

struct logging_args_t {
  rocket_state_t *state; /* The rocket state to log */
  char *storage_loc;     /* Storage location to log to */
};

void *logging_main(void *arg);

#endif // _INSPACE_LOGGING_H_
