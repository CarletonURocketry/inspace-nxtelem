#include "syslogging.h"
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#define SYSLOG_SYNC_FREQ 8

FILE *__syslogging_file = NULL;
atomic_int syslog_count;
static pthread_mutex_t syslog_mutex = PTHREAD_MUTEX_INITIALIZER;

/*
 * Sets up the syslogging to a file if syslog output is enabled.
 * @return Zero on success or skip, or errno on failure
 */
int setup_syslogging(void) {
#ifdef CONFIG_INSPACE_SYSLOG_PATH
    atomic_store(&syslog_count, 0);
    pthread_mutex_lock(&syslog_mutex);
    __syslogging_file = fopen(CONFIG_INSPACE_SYSLOG_PATH, "a");
    pthread_mutex_unlock(&syslog_mutex);
    if (__syslogging_file == NULL) {
        return errno;
    }
#endif
    return 0;
}

/*
 * Flushes the syslogging output (that is being sent to a file) periodically.
 * Caller must hold syslog_mutex.
 */
static void syslog_flush(void) {
    if (__syslogging_file) {
        int count = atomic_fetch_add(&syslog_count, 1);
        if ((count % SYSLOG_SYNC_FREQ) == 0) {
            if (fflush(__syslogging_file) < 0 || fsync(fileno(__syslogging_file)) < 0) {
                fprintf(stderr, "syslog_tee: flush failed: %s\n", strerror(errno));
                fclose(__syslogging_file);
                __syslogging_file = NULL;
            }
        }
    }
}

/*
 * Prints syslog output to the syslogging file (if set up), and to stdout
 * @param fmt The print format
 */
void syslog_tee(const char *fmt, ...) {
    va_list args;
    va_list args_copy;
    va_start(args, fmt);
    va_copy(args_copy, args);

    vfprintf(stdout, fmt, args);
    va_end(args);

    pthread_mutex_lock(&syslog_mutex);
    if (__syslogging_file != NULL) {
        if (vfprintf(__syslogging_file, fmt, args_copy) < 0) {
            fprintf(stderr, "syslog_tee: write failed, switching to stdout only\n");
            fclose(__syslogging_file);
            __syslogging_file = NULL;
        } else {
            syslog_flush();
        }
    }
    pthread_mutex_unlock(&syslog_mutex);

    va_end(args_copy);
}