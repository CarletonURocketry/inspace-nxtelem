#ifndef _INSPACE_TRANSMIT_H_
#define _INSPACE_TRANSMIT_H_

#include "../packets/buffering.h"
#include "../rocket-state/rocket-state.h"
#include "../radio-telem.h"

struct transmit_args {
    struct radio_options config;
    rocket_state_t *state;
    packet_buffer_t *buffer;
    radio_telem_t *radio_telem;
};

void *transmit_main(void *arg);

#endif // _INSPACE_TRANSMIT_H_
