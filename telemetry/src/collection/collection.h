#ifndef _COLLECTION_H_
#define _COLLECTION_H_

#include "../radio-telem.h"

struct collection_args {
    rocket_state_t *state;
    radio_telem_t *radio_telem;
};

void *collection_main(void *arg);

#endif // _COLLECTION_H_
