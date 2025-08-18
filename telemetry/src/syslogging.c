#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>

#include "syslogging.h"

#define SYSLOG_SYNC_FREQ 8

FILE *__syslogging_file;
atomic_int syslog_count;

/*
 * Sets up the syslogging to a file
 * @return Zero on success, or errno on failure
 */
int setup_syslogging(void) {
    atomic_set(&syslog_count, 0);
    __syslogging_file = fopen(CONFIG_INSPACE_SYSLOG_PATH, "a");
    if (__syslogging_file == NULL) {
        return errno;
    }
    return 0;
}

/*
 * Flushes the syslogging output (that is being sent to a file) periodically
 */
static void syslog_flush(void) {
    if (__syslogging_file) {
        int count = atomic_fetch_add(&syslog_count, 1);
        if ((count % SYSLOG_SYNC_FREQ) == 0) {
            fsync(fileno(__syslogging_file));
        }
    }
}

/*
 * Prints syslog output to the syslogging file (if set up), and to stdout
 * @param func The calling function
 * @param fmt The print format
 */
void syslog_tee(const char *fmt, ...) {
    va_list args;
    va_list args_copy;
    va_start(args, fmt);
    va_copy(args_copy, args);
    vfprintf(stdout, fmt, args);
    va_end(args);

    if (__syslogging_file) {
        vfprintf(__syslogging_file, fmt, args_copy);
        va_end(args_copy);

        syslog_flush();
    }
}
