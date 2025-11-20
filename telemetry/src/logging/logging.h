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
unsigned int choose_mission_number(const char *flight_dir, const char *flight_fmt, const char *extr_dir, const char *extr_fmt);
int open_log_file(FILE **opened_file, const char *format, int mission_num, int serial_number, const char *mode);
int copy_file(const char *from, const char *to);
int sync_files(const char *flight_dir, const char *flight_fmt, const char *extr_path_fmt);
int clean_dir(const char *dir, const char *fname_fmt);

void *logging_main(void *arg);

#endif // _INSPACE_LOGGING_H_
