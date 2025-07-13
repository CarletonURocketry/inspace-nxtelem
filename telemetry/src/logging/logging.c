#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../fusion/fusion.h"
#include "../packets/packets.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "../sensors/sensors.h"
#include "../syslogging.h"
#include "logging.h"
#include <dirent.h>
#include <time.h>

/* The maximum size of a log file's file name. */

#define MAX_FILENAME 32

/* The format for flight log file names */

#define FLIGHT_FNAME_FMT CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "flog_boot%d_%d.bin"

/* The format for extraction log file names */

#define EXTR_FNAME_FMT CONFIG_INSPACE_TELEMETRY_LANDED_FS "elog_boot%d_%d.bin"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* The number of times opening a log file will be attempted */

#define NUM_TIMES_TRY_OPEN 3

// Private Functions

static int find_max_boot_number(char *format_string);
static int switch_active_log_file(FILE **active_storage_file, FILE *storage_file_1, FILE **storage_file_2,
                                  char *flight_filename2);
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
    uint32_t seq_num = 0;

    char flight_filename1[sizeof(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS) + MAX_FILENAME];
    char flight_filename2[sizeof(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS) + MAX_FILENAME];
    char land_filename[sizeof(CONFIG_INSPACE_TELEMETRY_LANDED_FS) + MAX_FILENAME];

    FILE *storage_file_1 = NULL;      // File pointer for first logging file
    FILE *storage_file_2 = NULL;      // File pointer for second logging file
    FILE *active_storage_file = NULL; // File pointer for active logging file

    FILE *extract_storage_file; // File pointer for extraction logging file

    struct stat file_info;         // Stat struct to hold fstat return
    struct timespec base_timespec; // Time that file was last modified
    struct timespec new_timespec;  // Time to check against base_time to see if 30 seconds has passed

    indebug("Logging thread started.\n");

    /* Generate flight log file names using the boot number */
    int max_flight_log_boot_number = find_max_boot_number(FLIGHT_FNAME_FMT);
    indebug("Previous max boot number: %d\n", max_flight_log_boot_number);

    if (max_flight_log_boot_number < 0) {
        inerr("Error finding max boot number: return (%d)\n", max_flight_log_boot_number);
    }

    snprintf(flight_filename1, sizeof(flight_filename1), FLIGHT_FNAME_FMT, max_flight_log_boot_number + 1, 1);
    snprintf(flight_filename2, sizeof(flight_filename2), FLIGHT_FNAME_FMT, max_flight_log_boot_number + 1, 2);

    indebug("File 1 name: %s\n", flight_filename1);
    indebug("File 2 name: %s\n", flight_filename2);

    if (strlen(flight_filename1) > MAX_FILENAME) // Check log 1 filename length
    {
        inerr("Log file 1's name (%s) is longer than the maximum size of characters (%d)\n", flight_filename1,
              MAX_FILENAME);
    }

    if (strlen(flight_filename2) > MAX_FILENAME) // Check log 2 filename length
    {
        inerr("Log file 2's name (%s) is longer than the maximum size of characters (%d)\n", flight_filename2,
              MAX_FILENAME);
    }

    err = try_open_file(&storage_file_1, flight_filename1, "wb+");
    if (err < 0 || storage_file_1 == NULL) {
        inerr("Error opening log file 1 (%s): %d\n", flight_filename1, err);
        goto err_cleanup;
    }

    active_storage_file = storage_file_1;

    int fd = fileno(active_storage_file);
    if (fd < 0) {
        err = errno;
        inerr("Error using fileno: %d\n", err);
        goto err_cleanup;
    }

    err = fstat(fd, &file_info);
    if (err) {
        err = errno;
        inerr("Error using fstat: %d\n", err);
        goto err_cleanup;
    }

    // Save the time the file was last modified
    base_timespec = (struct timespec){file_info.st_mtime, 0};

    /* Infinite loop to handle states */
    for (;;) {
        err = state_get_flightstate(state, &flight_state);
        if (err) {
            inerr("Error getting flight state: %d\n", err);
            goto err_cleanup;
        }

        switch (flight_state) {
        case STATE_IDLE: {

            // Switch between files every 30 seconds

            /* Store current system time as new_timespec */
            if (clock_gettime(CLOCK_REALTIME, &new_timespec) < 0) {
                err = errno;
                inerr("Error during clock_gettime: %d\n", err);
                continue; // TODO - what to do when we fail here
            }
            indebug("Time: %d:%ld\n", new_timespec.tv_sec, new_timespec.tv_nsec);

            double time_diff = timespec_diff(&new_timespec, &base_timespec);
            if (time_diff < 0) {
                inerr("Error during timespec_diff (Negative difference returned): %f\n", time_diff);
            }

            if (time_diff >= 30.0) {
                indebug("30 seconds passed\n");

                // Switch active log file
                err = switch_active_log_file(&active_storage_file, storage_file_1, &storage_file_2, flight_filename2);
                if (err < 0) {
                    err = errno;
                    inerr("Error switching active log file: %d\n", err);
                    goto err_cleanup;
                }

                /* Set Base time to current time to reset*/
                base_timespec = new_timespec;
            }
        }
        /* Purposeful fall-through */

        case STATE_AIRBORNE: {
            packet_node_t *next_packet = packet_buffer_get_full(buffer);
            ((pkt_hdr_t *)next_packet->packet)->packet_num = seq_num++;
            log_packet(active_storage_file, next_packet->packet, next_packet->end - next_packet->packet);
            packet_buffer_put_empty(buffer, next_packet);
        } break;

        case STATE_LANDED: {
            indebug("Copying files to extraction file system.\n");

            /* Generate log file name for extraction file system */
            int max_extraction_log_file_number = find_max_boot_number(EXTR_FNAME_FMT);
            snprintf(land_filename, sizeof(land_filename), EXTR_FNAME_FMT, max_extraction_log_file_number + 1, 1);
            indebug("Extraction file name: %s\n", land_filename);

            err = try_open_file(&extract_storage_file, land_filename, "wb+");
            if (err < 0 || extract_storage_file == NULL) {
                // If reach here, all attempts to open log file have failed, fatal error
                err = errno;
                inerr("Couldn't open extraction log file '%s' with error: %d\n", land_filename, err);
                goto err_cleanup;
            }

            copy_out(active_storage_file, extract_storage_file);

            /* Once done copying, close extraction file */
            if (fclose(extract_storage_file) != 0) {
                err = errno;
                inerr("Couldn't close extraction log file '%s' with error: %d\n", land_filename, err);
            }

            /* Now that logs are copied to FAT partition, move back to the idle state for another go. */
            flight_state = STATE_IDLE;

            err = state_set_flightstate(state, STATE_IDLE);
            if (err < 0) {
                inerr("Error during state_set_flightstate: %d\n", err);
            }
        } break;
        }
    }

err_cleanup:
    /* Close files that may be open */
    if (storage_file_1 && fclose(storage_file_1) != 0) {
        err = errno;
        inerr("Failed to close flight logfile 1 handle: %d\n", err);
    }

    if (storage_file_2 && fclose(storage_file_2) != 0) {
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
 * Attempt to open file given filename and flag option multiple times. Once opened, update active file pointer.
 *
 * @param active_file_pointer The pointer to the file to update
 * @param filename The file name of the file to open
 * @param open_option The string option passed to fopen (r, rb, rb+, etc)
 * @return 0 if succesful, else error from errno
 */
static int try_open_file(FILE **file_pointer, char *filename, char *open_option) {
    int err = 0;
    // Matteo recomendeded trying multiple times, in case singular one fails.
    for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) {
        *file_pointer = fopen(filename, open_option);
        if (*file_pointer == NULL) {
            err = errno;
            inerr("Error (Attempt %d) opening '%s' file: %d\n", i, filename, err);
            usleep(1 * 1000); // Sleep for 1 millisecond before trying again
        } else {
            indebug("\nOpened File: %s\n\n", filename);
            break;
        }
    }

    return err;
}

/**
 * Find max boot number of logging files that already exist
 *
 * @param format_string The formatted string to sscanf to extract the boot number from. eg.) "flog_boot%d_%d.bin"
 * @return The maximum boot number found of previous files, -1 if error
 */
static int find_max_boot_number(char *format_string) {
    DIR *directory_pointer = opendir("/tmp");
    if (directory_pointer == NULL) {
        perror("Couldn't open the directory: ");
        return -1;
    }

    int max_boot_number = 0, boot_number = 0, file_number = 0;

    struct dirent *entry;
    while ((entry = readdir(directory_pointer)) != NULL) {
        if (sscanf(entry->d_name, format_string, &boot_number, &file_number) == 2) {
            if (boot_number > max_boot_number) {
                max_boot_number = boot_number;
            }
        }
    }

    (void)closedir(directory_pointer);
    return max_boot_number;
}

/**
 * Switch the active file pointer for the ping pong buffer
 *
 * @param active_storage_file The active log file pointer
 * @param storage_file_1 The pointer to the first log file
 * @param storage_file_2 The pointer to the second log file
 * @param flight_filename2 The name of the second log file
 * @return 0 if succesful, -1 on error
 */
static int switch_active_log_file(FILE **active_storage_file, FILE *storage_file_1, FILE **storage_file_2,
                                  char *flight_filename2) {
    int err = 0;

    /* Make sure we aren't dereferencing null pointer later */
    if (*active_storage_file == NULL) {
        inerr("Error in switch_active_log_file: active_storage_file is NULL");
        return -1;
    }

    if (*active_storage_file == storage_file_1) {
        if (*storage_file_2 != NULL) {
            *active_storage_file = *storage_file_2;
            return err;
        }

        /* If reach here, file 2 is not opened yet, so open it*/
        err = try_open_file(storage_file_2, flight_filename2, "wb+");
        if (err < 0 || *storage_file_2 == NULL) {
            inerr("Error (%d) trying to open %s", err, flight_filename2);
            return -1;
        }

        *active_storage_file = *storage_file_2;
    }

    // If file_2 is currently active, that means file_1 was opened successfully, so just switch the pointer
    else if (*active_storage_file == *storage_file_2) {
        if (storage_file_1 == NULL) {
            inerr("ERROR: storage_file_1 is NULL\n");
            return -1;
        }

        *active_storage_file = storage_file_1;
    }

    /* Roll active log file pointer back to beginning */
    err = fseek(*active_storage_file, 0, SEEK_SET);
    if (err) {
        inerr("Couldn't seek active file back to start: %d", err);
    }

    return err;
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
            inerr("Failed to read from the active file");
            return -err;
        }
        nbytes = sizeof(buf) * fwrite(buf, nbytes, 1, extract_file);
        if (nbytes == 0) {
            inerr("Failed to write to the extraction file");
            // Don't break here, in case writing only fails once
        }
    }
    return 0;
}
