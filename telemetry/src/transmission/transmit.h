#ifndef _INSPACE_TRANSMIT_H_
#define _INSPACE_TRANSMIT_H_

#include "../rocket-state/rocket-state.h"
#include "../packets/buffering.h"

struct transmit_args {
    rocket_state_t *state;
    packet_buffer_t *buffer;
};

void *transmit_main(void *arg);

#endif // _INSPACE_TRANSMIT_H_
