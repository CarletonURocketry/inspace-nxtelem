#ifndef _INSPACE_LOGGING_H_
#define _INSPACE_LOGGING_H_

#include "../packets/buffering.h"
#include "../rocket-state/rocket-state.h"
#include <stdio.h>

struct logging_args {
    rocket_state_t *state;
    packet_buffer_t *buffer;
};

/* Exposed functions for testing */
int should_swap(struct timespec *last_swap, struct timespec *now);
int swap_files(FILE **active_file, FILE **standby_file);
int choose_flight_number(const char *dir, const char *format);
int open_log_file(FILE **opened_file, const char *format, int flight_number, int serial_number, const char *mode);
int copy_out(FILE *active_file, FILE *extract_file);

void *logging_main(void *arg);

#endif // _INSPACE_LOGGING_H_
