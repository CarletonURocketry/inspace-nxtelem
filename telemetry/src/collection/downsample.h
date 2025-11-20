#ifndef _INSPACE_DOWNSAMPLE_
#define _INSPACE_DOWNSAMPLE_

#include "../radio-telem.h"

struct downsample_args {
    rocket_state_t *state;
    radio_telem_t *radio_telem;
};

void *downsample_main(void *arg);

#endif // _INSPACE_DOWNSAMPLE_
