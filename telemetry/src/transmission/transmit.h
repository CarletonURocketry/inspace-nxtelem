#ifndef _INSPACE_TRANSMIT_H_
#define _INSPACE_TRANSMIT_H_

#include "../radio-telem.h"

struct transmit_args {
    struct radio_options config;
    radio_telem_t *radio_telem;
};

void *transmit_main(void *arg);

#endif // _INSPACE_TRANSMIT_H_
