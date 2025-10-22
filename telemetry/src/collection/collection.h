#ifndef _COLLECTION_H_
#define _COLLECTION_H_

#include "../packets/buffering.h"
#include "../rocket-state/rocket-state.h"
#include "../radio-telem.h"

struct collection_args {
    rocket_state_t *state;
    packet_buffer_t *logging_buffer;
    packet_buffer_t *transmit_buffer;
    radio_telem_t *radio_telem;
};

void *collection_main(void *arg);

#endif // _COLLECTION_H_
