#ifndef _INSPACE_TRANSMIT_H_
#define _INSPACE_TRANSMIT_H_

#include "../rocket-state/rocket-state.h"

/* Arguments to the radio transmission thread. */
struct transmit_args_t {
  rocket_state_t *state; /* The rocket state containing data to transmit */
  char *radio_dev;       /* The radio device file path to transmit on */
};

void *transmit_main(void *arg);

#endif // _INSPACE_TRANSMIT_H_
