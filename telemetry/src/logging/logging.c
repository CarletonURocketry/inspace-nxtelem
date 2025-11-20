#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "../collection/status-update.h"
#include "../packets/packets.h"
#include "../syslogging.h"
#include "logging.h"

/* The format for flight log file names. Note that integers can take up at most 11 bytes, and these must fit into
 * NAME_MAX */

#define FLIGHT_FNAME_FMT "flog_%d_%d.bin"
#define FLIGHT_FPATH_FMT CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "/" FLIGHT_FNAME_FMT

/* The format for extraction log file names */

#define EXTR_FNAME_FMT "elog_%d_%d.bin"
#define EXTR_FPATH_FMT CONFIG_INSPACE_TELEMETRY_LANDED_FS "/" EXTR_FNAME_FMT

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* The number of times opening a log file will be attempted */

#define NUM_TIMES_TRY_OPEN 10

/* The number of seconds of guaranteed data before liftoff */

#define PING_PONG_DURATION 30.0f

static int log_packet(FILE *storage, uint8_t *packet, size_t packet_size);
static int clear_file(FILE *to_clear);
static int try_open_file(FILE **file_to_open, const char *filename, const char *open_option);
static int find_max_mission_number(const char *dir, const char *format);
static double timespec_diff(struct timespec *new_time, struct timespec *old_time);
static int close_synced(FILE *to_close);
static int ejectled_set(bool on);

/*
 * Logging thread which runs to log data to the SD card.
 */
void *logging_main(void *arg) {
    int err;
    enum flight_state_e flight_state;
    struct logging_args *unpacked_args = (struct logging_args *)(arg);
    rocket_state_t *state = unpacked_args->state;
    packet_buffer_t *buffer = unpacked_args->buffer;
    unsigned int packet_seq_num = 0;
    bool ejectled_on = false;
    FILE *active_file = NULL;
    FILE *standby_file = NULL;
    struct timespec last_swap;

    ininfo("Logging thread started.\n");

    ejectled_set(false); /* Turn off LED on start */

    /* Choose the maximum of the flight numbers on the extraction and user filesystems. Should not change */
    const unsigned int mission_num = choose_mission_number(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS, FLIGHT_FNAME_FMT,
                                                           CONFIG_INSPACE_TELEMETRY_LANDED_FS, EXTR_FNAME_FMT);
    unsigned int flight_ser_num = 0;

    err = open_log_file(&active_file, FLIGHT_FPATH_FMT, mission_num, flight_ser_num++, "w+");
    if (err < 0) {
        inerr("Error opening log file with flight number %d, serial number: %d: %d\n", mission_num, flight_ser_num,
              err);
        goto err_cleanup;
    }

    err = open_log_file(&standby_file, FLIGHT_FPATH_FMT, mission_num, flight_ser_num++, "w+");
    if (err < 0) {
        inerr("Error opening log file with flight number %d, serial number: %d: %d\n", mission_num, flight_ser_num,
              err);
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
            flight_state = STATE_AIRBORNE;
        }

        switch (flight_state) {
        case STATE_IDLE: {
            struct timespec now;
            if (clock_gettime(CLOCK_REALTIME, &now) < 0) {
                err = errno;
                inerr("Error during clock_gettime: %d\n", err);
            }
            if (should_swap(&now, &last_swap)) {
                swap_files(&active_file, &standby_file);
                last_swap = now;
                ininfo("Swapped logging files\n");
            }

            if (!ejectled_on) {
                if (!ejectled_set(true)) {
                    ejectled_on = true;
                }
            }

        } /* Purposeful fall-through */

        case STATE_AIRBORNE: {
            /* Not safe to eject from now onward until files are copied. if-statement is to guard against idle state
             * fall-through.
             */
            if (ejectled_on && flight_state == STATE_AIRBORNE) {
                if (!ejectled_set(false)) {
                    ejectled_on = false;
                }
            }

            // Get the next packet, or wait if there isn't one yet
            packet_node_t *next_packet = packet_buffer_get_full(buffer);
            ((pkt_hdr_t *)next_packet->packet)->packet_num = packet_seq_num++;
            if (log_packet(active_file, next_packet->packet, next_packet->end - next_packet->packet) < 0) {
                inerr("Opening a new logging file because writing to the current one failed");
                fclose(active_file);
                err = open_log_file(&active_file, FLIGHT_FPATH_FMT, mission_num, flight_ser_num++, "w+");
                if (err < 0) {
                    inerr("Error opening log file with flight number %d, serial number: %d: %d\n", mission_num,
                          flight_ser_num, err);
                    goto err_cleanup;
                }
            }
            packet_buffer_put_empty(buffer, next_packet);

            // Have to flush and sync on the power safe file system to commit writes. Do this sparingly to save time
            if ((packet_seq_num % CONFIG_INSPACE_TELEMETRY_FS_SYNC_FREQ) == 0) {
                indebug("Syncing littlefs...\n");
                fsync(fileno(active_file));
                indebug("littlefs synced!\n");
            }

        } break;

        case STATE_LANDED: {
            /* Copy out files */

            /* Close to make sure the changes are committed, seperate this flight from next */
            close_synced(active_file);
            close_synced(standby_file);

            err = sync_files(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS, FLIGHT_FNAME_FMT, EXTR_FPATH_FMT);
            if (err == 0) {
                /* Only delete files if everything was synced successfully */

                err = clean_dir(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS, FLIGHT_FNAME_FMT);
                if (err < 0) {
                    inerr("Couldn't clear flight file directory after successful sync: %d\n", err);
                }

                /* It's now safe to take out the SD card */

                if (!ejectled_set(true)) {
                    ejectled_on = true;
                }
            } else {
                inerr("Couldn't sync all files to user system (skipped deletion): %d\n", err);
            }

            /* Open new active/standby files */

            err = open_log_file(&standby_file, FLIGHT_FPATH_FMT, mission_num, flight_ser_num++, "w+");
            if (err < 0) {
                inerr("Error opening new standby log file with flight number %d, serial number: %d: %d\n", mission_num,
                      flight_ser_num, err);
                goto err_cleanup;
            }

            err = open_log_file(&active_file, FLIGHT_FPATH_FMT, mission_num, flight_ser_num++, "w+");
            if (err < 0) {
                inerr("Error opening new active log file with flight number %d, serial number: %d: %d\n", mission_num,
                      flight_ser_num, err);
                goto err_cleanup;
            }

            /* If copying the files out failed, still set to IDLE so that we don't get stuck */

            err = state_set_flightstate(state, STATE_IDLE);
            if (err < 0) {
                inerr("Error during state_set_flightstate: %d\n", err);
            }
        } break;
        }
    }

err_cleanup:
    /* Close files that may be open */
    if (active_file && close_synced(active_file) != 0) {
        err = errno;
        inerr("Failed to close active file: %d\n", err);
    }

    if (standby_file && close_synced(standby_file) != 0) {
        err = errno;
        inerr("Failed to close standby file: %d\n", err);
    }

    publish_error(PROC_ID_LOGGING, ERROR_PROCESS_DEAD);
    pthread_exit(err_to_ptr(err));
}

/**
 * Logs a packet to the specified file
 *
 * @param storage The file to write to
 * @param packet The packet to log
 * @param packet_size The size of the packet being logged
 * @return The number of bytes written or a negative error code
 */
static int log_packet(FILE *storage, uint8_t *packet, size_t packet_size) {
    size_t written = fwrite(packet, 1, packet_size, storage);
    indebug("Logged %zu bytes\n", written);
    if (written != packet_size) {
        if (ferror(storage)) {
            int err = errno;
            inerr("Failed to write data to the logging file: %d\n", err);
            return -err;
        }
    }
    return written;
}

/**
 * Set the length of a file to zero, and reset its write position
 *
 * @param to_clear The file to clear the contents of
 * @return 0 if the file was cleared successfully, or a negative error code
 */
static int clear_file(FILE *to_clear) {
    int err = fseek(to_clear, 0, SEEK_SET);
    if (err) {
        err = errno;
        inerr("Couldn't seek active file back to start: %d\n", err);
        return -err;
    }

    int fd = fileno(to_clear);
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
    fsync(fd);
    return -err;
}

/**
 * Attempt to open file given filename and flag option multiple times. Once opened, update active file pointer.
 *
 * @param active_file_pointer The pointer to the file to update
 * @param filename The file name of the file to open
 * @param open_option An open option like fopen would expect
 * @return 0 if succesful, otherwise a negative error code
 */
static int try_open_file(FILE **file_pointer, const char *filename, const char *open_option) {
    int err = 0;
    for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) {
        *file_pointer = fopen(filename, open_option);
        if (*file_pointer == NULL) {
            err = errno;
            inerr("Error (Attempt %d) opening '%s' file: %d\n", i, filename, err);
            usleep(1 * 1000); // Sleep for 1 millisecond before trying again
        } else {
            err = 0;
            indebug("Opened File: %s\n", filename);
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
 * @return The maximum boot number found of previous files, or a random number if there was an error
 */
static int find_max_mission_number(const char *dir, const char *format) {
    int err;
    DIR *directory_pointer = opendir(dir);
    if (directory_pointer == NULL) {
        err = errno;
        inerr("Could not open the directory to read the boot number: %d\n", err);
        return rand() % (INT_MAX / 2);
    }

    int max_boot_number = -1;
    int boot_number = 0;
    int file_number = 0;

    // Following instructions for readdir, must set errno to be able to tell if error occured
    errno = 0;
    struct dirent *entry;
    while ((entry = readdir(directory_pointer)) != NULL) {
        if (sscanf(entry->d_name, format, &boot_number, &file_number) == 2) {
            if (boot_number > max_boot_number) {
                max_boot_number = boot_number;
            }
        }
    }
    err = errno;
    if (err) {
        inerr("Couldn't read all directory entries to find max boot number: %d\n", err);
        max_boot_number = rand() % (INT_MAX / 2);
    }

    (void)closedir(directory_pointer);
    return max_boot_number;
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
    static uint8_t buf[BUFSIZ];
    size_t bytes_read;
    size_t bytes_written;
    int err;

    /* Roll power-safe log file pointer back to beginning */

    err = fseek(active_file, 0, SEEK_SET);
    if (err) {
        err = errno;
        inerr("Couldn't seek active file back to start: %d\n", err);
        return -err;
    }

    /* Copy from one to the other using a buffer */

    while ((bytes_read = fread(buf, 1, sizeof(buf), active_file)) > 0) {
        bytes_written = fwrite(buf, 1, bytes_read, extract_file);
        if (bytes_written != bytes_read) {
            err = errno;
            inerr("Failed to write to the extraction file: %d\n", err);
            break;
        }
    }

    // If the last fread just failed, errno will still be set and isn't touched by ferror
    if (ferror(active_file)) {
        err = errno;
        inerr("Failed to read from the source file when copying to extraction: %d\n", err);
    }
    return -err;
}

/**
 * Close a file, but sync it and flush it first
 *
 * @param to_close The file to close
 * @return 0 on success, or -1 on failure (errno is set)
 */
static int close_synced(FILE *to_close) {
    fflush(to_close);
    fsync(fileno(to_close));
    return fclose(to_close);
}

/* Turns on/off the eject LED.
 *
 * @param on True to turn on the LED, false to turn off the LED.
 * @return 0 on success, errno code on failure.
 */
static int ejectled_set(bool on) {
    int fd;
    int err;

    int oflags = O_RDONLY;
#ifdef CONFIG_ARCH_SIM
    oflags |= O_CREAT;
#endif
    fd = open(CONFIG_INSPACE_TELEMETRY_EJECTLED, oflags);
    if (fd < 0) {
        inerr("Could not open %s: %d\n", CONFIG_INSPACE_TELEMETRY_EJECTLED, errno);
        return errno;
    }

// Need to check if the simulator has a way to mock the gpio driver
#ifndef CONFIG_ARCH_SIM
    err = ioctl(fd, GPIOC_WRITE, on);
    if (err < 0) {
        inerr("Could not turn on eject LED: %d\n", errno);
        return errno;
    }
#endif

    indebug("Eject LED %s.", on ? "on" : "off");

    err = close(fd);
    if (err < 0) {
        inerr("Couldn't close eject LED file: %d\n", errno);
        return errno;
    }
    return 0;
}

/**
 * Checks if its been at least the amount of time specified by PING_PONG_DURATION since last_swap
 *
 * @param last_swap The last time the active and standby files were swapped
 * @return 1 if they should be swapped, 0 otherwise
 */
int should_swap(struct timespec *now, struct timespec *last_swap) {
    double time_diff = timespec_diff(now, last_swap);
    if (time_diff < 0) {
        inerr("Time difference is negative\n");
    }
    return time_diff > PING_PONG_DURATION;
}

/**
 * Swaps the active and standby files, resetting the new active file
 *
 * @param active_file The current active file, to be swapped with standby_file
 * @param standby_file The current standby file, to be swapped with active_file
 * @return 0 if successful, or a negative error code
 */
int swap_files(FILE **active_file, FILE **standby_file) {
    FILE *tmp = *active_file;
    *active_file = *standby_file;
    *standby_file = tmp;
    int err = clear_file(*active_file);
    return err;
}

/**
 * Pick a flight number using previous files in a directory to avoid naming conflicts
 *
 * @param dir The directory to look for previous files in
 * @param format The filename format to look using (which has two integer print codes in it)
 * @return A flight number to use with the format string as the first
 */
unsigned int choose_mission_number(const char *flight_dir, const char *flight_fmt, const char *extr_dir,
                                   const char *extr_fmt) {
    int flight_num = find_max_mission_number(flight_dir, flight_fmt);
    int extr_num = find_max_mission_number(extr_dir, extr_fmt);

    if (flight_num > extr_num) {
        return flight_num + 1;
    } else {
        return extr_num + 1;
    }
}

/**
 * Open a file using the specified format and serial numbers for its name
 *
 * @param opened_file The pointer to the file to update
 * @param format A format string with two integer print codes
 * @param mission_num The integer to use with the first print code
 * @param serial_number The integer to use with the second print code
 * @param mode The open options for fopen
 */
int open_log_file(FILE **opened_file, const char *format, int mission_num, int serial_number, const char *mode) {
    static char filename[PATH_MAX];
    snprintf(filename, sizeof(filename), format, mission_num, serial_number);
    return try_open_file(opened_file, filename, mode);
}

/**
 * Copy the contents of a file at one path and append them to a file at another path, creating it if it doesn't exist
 *
 * @param from The file to copy from
 * @param to The file to copy to
 * @return 0 on success, or a negative error on failure
 */
int copy_file(const char *from, const char *to) {
    // This file hasn't been copied yet
    int err = 0;

    // Could also just use fopen here, since we might get another chance to sync files later
    FILE *from_file = NULL;
    try_open_file(&from_file, from, "r");
    if (from_file == NULL) {
        err = errno;
        inerr("Couldn't open file to copy from with path %s: %d\n", from, err);
        return -err;
    }
    FILE *to_file = NULL;
    try_open_file(&to_file, to, "a");
    if (to_file == NULL) {
        err = errno;
        inerr("Couldn't open file to copy to with path %s: %d\n", to, err);
        return -err;
    }

    err = copy_out(from_file, to_file);
    if (err < 0) {
        inerr("Couldn't copy file contents when syncing with path %s: %d\n", from, err);
        return err;
    }

    // If closing the files fails,
    if (fclose(from_file) < 0) {
        inerr("Couldn't close file to copy from\n");
    }
    // Make sure the write has happened for this file
    if (close_synced(to_file) < 0) {
        inerr("Couldn't close file to copy to with sync\n");
    }
    return 0;
}

/**
 * Sync files in directories, copying everything matching flight_fmt to extr_dir with filenames following extr_fmt
 *
 * @param flight_dir The directory to sync from
 * @param flight_fmt The format of filenames in flight_dir to copy
 * @param extr_path_fmt The format for paths on the extraction filesystem
 */
int sync_files(const char *flight_dir, const char *flight_fmt, const char *extr_path_fmt) {
    // sync_err represents the last unexpected error that sync_files had
    int sync_err = 0;
    int err = 0;
    DIR *directory_pointer = opendir(flight_dir);
    if (directory_pointer == NULL) {
        err = errno;
        inerr("Could not open the directory to sync files: %d\n", err);
        return -err;
    }

    // Following instructions for readdir, must set errno to be able to tell if error occured
    errno = 0;
    struct dirent *entry;
    while ((entry = readdir(directory_pointer)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        int boot_number = 0, file_number = 0;
        if (sscanf(entry->d_name, flight_fmt, &boot_number, &file_number) == 2) {
            static char extr_path[PATH_MAX];
            snprintf(extr_path, sizeof(extr_path), extr_path_fmt, boot_number, file_number);

            // Failing access check means the file hasn't been copied out yet (as long as error is ENOENT)
            if (access(extr_path, F_OK) < 0) {
                err = errno;
                if (err == ENOENT) {
                    static char flight_path[PATH_MAX];
                    snprintf(flight_path, sizeof(flight_path), "%s/%s", flight_dir, entry->d_name);

                    err = -copy_file(flight_path, extr_path);
                    if (err) {
                        sync_err = err;
                    }
                } else {
                    sync_err = err;
                    inerr("Failed to use access() to check file existence: %d\n", err);
                }
            }
        }
        // Reset errno, so that we can determine if it was the readdir call that failed
        errno = 0;
    }
    // Check if we got an error on readdir, and that the error isn't caused by something inside the loop
    err = errno;
    if (err) {
        inerr("Couldn't read all directory entries to find max boot number: %d\n", err);
        sync_err = err;
    }

    closedir(directory_pointer);
    return -sync_err;
}

/**
 * Delete all files in a directory using unlink
 *
 * @param dir The directory to delete all files in
 * @return 0 if successful and all files were deleted, or a negative error code
 */
int clean_dir(const char *dir, const char *fname_fmt) {
    int err = 0;
    DIR *directory_pointer = opendir(dir);
    if (directory_pointer == NULL) {
        inerr("Could not open the directory to sync files: %d\n", errno);
        return -errno;
    }

    int fd = dirfd(directory_pointer);
    if (fd < 0) {
        inerr("Could not get directory file descriptor: %d\n", fd);
        closedir(directory_pointer);
        return -fd;
    }

    // Following instructions for readdir, must set errno to be able to tell if error occured
    errno = 0;
    struct dirent *entry;
    while ((entry = readdir(directory_pointer)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        int boot_number = 0;
        int file_number = 0;
        if (sscanf(entry->d_name, fname_fmt, &boot_number, &file_number) != 2) {
            continue;
        }

        if (unlinkat(fd, entry->d_name, 0) < 0) {
            err = errno;
            if (err == ENOENT) {
                err = 0;
            } else {
                inerr("Could not delete file at path %s: %d\n", entry->d_name, err);
            }
            // Continue. Not a problem to leave a file around
        }
        errno = 0;
    }
    int dir_error = errno;
    if (dir_error) {
        inerr("Failed to iterate over all files in directory when deleting: %d\n", dir_error);
        return -dir_error;
    }

    closedir(directory_pointer);
    return -err;
}
