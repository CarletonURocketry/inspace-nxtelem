#include <nuttx/config.h>

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "../fusion/fusion.h"
#include "../packets/packets.h"
#include "../syslogging.h"
#include "logging.h"

/* The format for flight log file names */

#define FLIGHT_FNAME_FMT CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "flog_boot%d_%d.bin"

/* The format for extraction log file names */

#define EXTR_FNAME_FMT CONFIG_INSPACE_TELEMETRY_LANDED_FS "elog_boot%d_%d.bin"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* The number of times opening a log file will be attempted */

#define NUM_TIMES_TRY_OPEN 3

/* The number of seconds of guaranteed data before liftoff */

// TODO - make configurable
#define PING_PONG_DURATION 1.0f

// Private Functions

static int should_swap(struct timespec *last_swap);
static int swap_files(FILE **active_file, FILE **standby_file, struct timespec *last_swap);
static int find_max_boot_number(const char *dir, const char *format);
static int choose_flight_number(const char *dir, const char *format);
static int open_log_file(FILE **opened_file, const char *format, int flight_number, int serial_number,
                         const char *mode);
static int get_last_modified_time(FILE *file, struct timespec *last_modified);
static double timespec_diff(struct timespec *new_time, struct timespec *old_time);
static int try_open_file(FILE **file_to_open, char *filename, char *open_option);
static size_t log_packet(FILE *storage, uint8_t *packet, size_t packet_size);
static int copy_out(FILE *active_file, FILE *extract_file);

/*
 * Logging thread which runs to log data to the SD card.
 */
void *logging_main(void *arg) {
    int err;
    enum flight_state_e flight_state;
    struct logging_args *unpacked_args = (struct logging_args *)(arg);
    rocket_state_t *state = unpacked_args->state;
    packet_buffer_t *buffer = unpacked_args->buffer;
    int packet_seq_num = 0;

    FILE *active_file = NULL;
    FILE *standby_file = NULL;
    struct timespec last_swap;

    indebug("Logging thread started.\n");

    /* Generate flight log file names using the boot number */
    int flight_number = choose_flight_number(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS, FLIGHT_FNAME_FMT);
    int flight_ser_num = 0;

    int extract_number = choose_flight_number(CONFIG_INSPACE_TELEMETRY_LANDED_FS, EXTR_FNAME_FMT);
    int extract_ser_num = 0;

    err = open_log_file(&active_file, FLIGHT_FNAME_FMT, flight_number, flight_ser_num++, "wb+");
    if (err < 0) {
        inerr("Error opening log file with flight number %d, serial number: %d: %d", flight_number, flight_ser_num,
              err);
        // As an error handling step, may want to retry with a random flight number
        goto err_cleanup;
    }

    err = open_log_file(&standby_file, FLIGHT_FNAME_FMT, flight_number, flight_ser_num++, "wb+");
    if (err < 0) {
        inerr("Error opening log file with flight number %d, serial number: %d: %d", flight_number, flight_ser_num,
              err);
        // As an error handling step, may want to retry with a random flight number
        goto err_cleanup;
    }

    // Since both files were just overwritten, neither has any data
    if (clock_gettime(CLOCK_REALTIME, &last_swap) < 0) {
        err = errno;
        inerr("Error during clock_gettime: %d\n", err);
    }

    /* Infinite loop to handle states */
    for (;;) {

        err = state_get_flightstate(state, &flight_state);
        if (err) {
            inerr("Error getting flight state: %d\n", err);
            goto err_cleanup;
        }

        switch (flight_state) {
        case STATE_IDLE: {
            if (should_swap(&last_swap)) {
                swap_files(&active_file, &standby_file, &last_swap);
                ininfo("Swapped logging files");
            }

        } /* Purposeful fall-through */

        case STATE_AIRBORNE: {
            packet_node_t *next_packet = packet_buffer_get_full(buffer);
            ((pkt_hdr_t *)next_packet->packet)->packet_num = packet_seq_num++;
            log_packet(active_file, next_packet->packet, next_packet->end - next_packet->packet);
            packet_buffer_put_empty(buffer, next_packet);
        } break;

        case STATE_LANDED: {

            FILE *extract_file = NULL;
            // Don't overwrite if there's anything in there already
            err = open_log_file(&extract_file, EXTR_FNAME_FMT, extract_number, extract_ser_num++, "a");
            if (err < 0) {
                inerr("Error opening log file with flight number %d, serial number: %d: %d", extract_number,
                      extract_ser_num, err);
                // TODO - add more error handling here, should try harder to get data
                break;
            }
            indebug("Copying files to extraction file system.\n");

            // Copy out both the standby and the active. If standby doesn't have data it'll be empty and this is a no-op
            if (copy_out(standby_file, extract_file) < 0) {
                // Ignoring any error here, since we'll be opening a new file anyways
                fclose(standby_file);
                err = open_log_file(&standby_file, FLIGHT_FNAME_FMT, flight_number, flight_ser_num++, "wb+");
                if (err < 0) {
                    inerr("Error opening new standby log file with flight number %d, serial number: %d: %d\n",
                          flight_number, flight_ser_num, err);
                    goto err_cleanup;
                }
            }

            if (copy_out(active_file, extract_file) < 0) {
                fclose(active_file);
                err = open_log_file(&active_file, FLIGHT_FNAME_FMT, flight_number, flight_ser_num++, "wb+");
                if (err < 0) {
                    inerr("Error opening new active log file with flight number %d, serial number: %d: %d\n",
                          flight_number, flight_ser_num, err);
                    goto err_cleanup;
                }
            }

            /* Once done copying, close extraction file */
            if (fclose(extract_file) != 0) {
                err = errno;
                inerr("Couldn't close extraction log file: %d\n", err);
            }

            /* Now that logs are copied to FAT partition, move back to the idle state for another go. */
            flight_state = STATE_IDLE;
            // TODO - delete old storage files or clear data in some way, to recover their space

            err = state_set_flightstate(state, STATE_IDLE);
            if (err < 0) {
                inerr("Error during state_set_flightstate: %d\n", err);
            }
        } break;
        }
    }

err_cleanup:
    /* Close files that may be open */
    if (active_file && fclose(active_file) != 0) {
        err = errno;
        inerr("Failed to close flight logfile 1 handle: %d\n", err);
    }

    if (standby_file && fclose(standby_file) != 0) {
        err = errno;
        inerr("Failed to close flight logfile 2 handle: %d\n", err);
    }

    pthread_exit(err_to_ptr(err));
}

static size_t log_packet(FILE *storage, uint8_t *packet, size_t packet_size) {
    size_t written;

    written = fwrite(packet, 1, packet_size, storage);
    indebug("Logged %zu bytes\n", written);
    if (written == 0) {
        // TODO: Handle error (might happen if file got too large, start
        // another file)
        inerr("Failted to write data to the logging file\n");
    }
    return written;
}

/****************************************************/
/*                Helper Functions                  */
/****************************************************/

/**
 * Checks if its been at least the amount of time specified by PING_PONG_DURATION since last_swap
 *
 * @param last_swap The last time the active and standby files were swapped
 * @return 1 if they should be swapped, 0 otherwise
 */
static int should_swap(struct timespec *last_swap) {
    int err = 0;
    struct timespec now;
    if (clock_gettime(CLOCK_REALTIME, &now) < 0) {
        err = errno;
        inerr("Error during clock_gettime: %d\n", err);
        return 0;
    }

    double time_diff = timespec_diff(&now, last_swap);
    if (time_diff < 0) {
        inerr("Time difference is negative\n");
    }

    ininfo("time diff is %f\n", time_diff);
    return time_diff > PING_PONG_DURATION;
}

/**
 * Swaps the active and standby files, resetting the new active file and setting the new last_swap time
 *
 * @param active_file The current active file, to be swapped with standby_file
 * @param standby_file The current standby file, to be swapped with active_file
 * @return 0 if successful, or a negative error code
 */
static int swap_files(FILE **active_file, FILE **standby_file, struct timespec *last_swap) {
    int err = 0;
    struct timespec now;
    if (clock_gettime(CLOCK_REALTIME, &now) < 0) {
        err = errno;
        inerr("Error during clock_gettime: %d\n", err);
    } else {
        *last_swap = now;
    }

    // Switch active log file
    FILE *tmp = *active_file;
    *active_file = *standby_file;
    *standby_file = tmp;

    err = fseek(*active_file, 0, SEEK_SET);
    if (err) {
        err = errno;
        inerr("Couldn't seek active file back to start: %d", err);
        // Not a huge deal
    }
    return -err;
}

/**
 * Gets the time that a file was last modified and stores it in a timespec struct
 *
 * @param file The file to check the modified time for
 * @param last_modified To be filled with information about when the file was last modified
 * @return 0 if successful, or a negative error code
 */
static int get_last_modified_time(FILE *file, struct timespec *last_modified) {
    int err = 0;

    // Need for fstat
    int fd = fileno(file);
    if (fd < 0) {
        err = errno;
        inerr("Error using fileno: %d\n", err);
        return -err;
    }

    struct stat file_info;
    err = fstat(fd, &file_info);
    if (err) {
        err = errno;
        inerr("Error using fstat: %d\n", err);
        return -err;
    }

    // Last modified time
    last_modified->tv_sec = file_info.st_mtime;
    last_modified->tv_nsec = 0;
    return -err;
}

/**
 * Open a file using the specified format and serial numbers for its name
 *
 * @param opened_file The pointer to the file to update
 * @param format A format string with two integer print codes
 * @param flight_number The integer to use with the first print code
 * @param serial_number The integer to use with the second print code
 * @param mode The open options for fopen
 */
static int open_log_file(FILE **opened_file, const char *format, int flight_number, int serial_number,
                         const char *mode) {
    static char filename[NAME_MAX]; // This should really be PATH_MAX, with a check on how large the filename part is
    snprintf(filename, sizeof(filename), format, flight_number, serial_number);
    return try_open_file(opened_file, filename, "wb+");
}

/**
 * Attempt to open file given filename and flag option multiple times. Once opened, update active file pointer.
 *
 * @param active_file_pointer The pointer to the file to update
 * @param filename The file name of the file to open
 * @param open_option An open option like fopen would expect
 * @return 0 if succesful, otherwise a negative error code
 */
static int try_open_file(FILE **file_pointer, char *filename, char *open_option) {
    int err = 0;
    for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) {
        *file_pointer = fopen(filename, open_option);
        if (*file_pointer == NULL) {
            err = errno;
            inerr("Error (Attempt %d) opening '%s' file: %d\n", i, filename, err);
            usleep(1 * 1000); // Sleep for 1 millisecond before trying again
        } else {
            indebug("Opened File: %s", filename);
            break;
        }
    }

    return -err;
}

/**
 * Find max boot number of logging files that already exist
 *
 * @param dir The directory to look for the boot number in
 * @param format The filename format to look using (which has two integer print codes in it)
 * @return The maximum boot number found of previous files, 0 if none are found, or a negative error code
 */
static int find_max_boot_number(const char *dir, const char *format) {
    DIR *directory_pointer = opendir(dir);
    if (directory_pointer == NULL) {
        int err = errno;
        inerr("Could not open the directory to read the boot number: %d", err);
        return -err;
    }

    int max_boot_number = 0, boot_number = 0, file_number = 0;

    struct dirent *entry;
    while ((entry = readdir(directory_pointer)) != NULL) {
        if (sscanf(entry->d_name, format, &boot_number, &file_number) == 2) {
            if (boot_number > max_boot_number) {
                max_boot_number = boot_number;
            }
        }
    }

    (void)closedir(directory_pointer);
    return max_boot_number;
}

/**
 * Pick a flight number using previous files in a directory to avoid naming conflicts
 *
 * @param dir The directory to look for previous files in
 * @param format The filename format to look using (which has two integer print codes in it)
 * @return A flight number to use with the format string as the first
 */
static int choose_flight_number(const char *dir, const char *format) {
    int flight_number = find_max_boot_number(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS, FLIGHT_FNAME_FMT);
    if (flight_number < 0) {
        inerr("Error finding max boot number: %d, picking a random number instead\n", flight_number);
        flight_number = rand();
    } else {
        indebug("Previous max boot number: %d\n", flight_number);
        flight_number += 1;
    }
    return flight_number;
}

/**
 * Returns double time difference between two timespec structs
 *
 * @param new_time The newer (larger) time
 * @param old_time The older (smaller) time
 * @return The difference between the two times
 */
static double timespec_diff(struct timespec *new_time, struct timespec *old_time) {
    return (new_time->tv_sec - old_time->tv_sec) + (new_time->tv_nsec - old_time->tv_nsec) / 1e9;
}

/**
 * Copies a file's contents to another using read and write calls
 *
 * @param active_file The file to copy from
 * @param extract_file The file to copy to
 * @return 0 on success or a negative error code
 */
static int copy_out(FILE *active_file, FILE *extract_file) {
    /* Roll power-safe log file pointer back to beginning */
    int err = fseek(active_file, 0, SEEK_SET);
    if (err) {
        err = errno;
        inerr("Couldn't seek active file back to start: %d", err);
        return -err;
    }

    /* Copy from one to the other using a buffer */
    uint8_t buf[BUFSIZ];
    size_t nbytes = 0;

    while (!feof(active_file)) {
        nbytes = sizeof(buf) * fread(buf, sizeof(buf), 1, active_file);
        if (nbytes == 0) {
            err = errno;
            inerr("Failed to read from the active file\n");
            return -err;
        }
        nbytes = sizeof(buf) * fwrite(buf, nbytes, 1, extract_file);
        if (nbytes == 0) {
            err = errno;
            inerr("Failed to write to the extraction file\n");
            // Don't break here, in case writing only fails once
        }
    }

    /* Don't truncate the file if the write out fails */
    if (err != 0) {
        return -err;
    }

    int fd = fileno(active_file);
    if (fd < 0) {
        err = errno;
        inerr("Failed to get file number (file stream invalid)\n");
        return -err;
    }

    if (ftruncate(fd, 0) < 0) {
        err = errno;
        inerr("Could not truncate file\n");
        return -err;
    }

    return err;
}
