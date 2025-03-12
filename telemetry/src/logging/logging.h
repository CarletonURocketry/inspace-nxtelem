#ifndef _INSPACE_LOGGING_H_
#define _INSPACE_LOGGING_H_

#include "../rocket-state/rocket-state.h"
#include "../packets/buffering.h"

struct logging_args {
    rocket_state_t *state;
    packet_buffer_t *buffer;
};

void *logging_main(void *arg);

#endif // _INSPACE_LOGGING_H_
