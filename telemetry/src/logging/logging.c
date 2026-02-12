#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
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

enum uorb_sensors {
    SENSOR_ACCEL,   /* Accelerometer */
    SENSOR_GYRO,    /* Gyroscope */
    SENSOR_MAG,     /* Magnetometer */
    SENSOR_GNSS,    /* GNSS */
    SENSOR_ALT,     /* Altitude fusion */
    SENSOR_BARO,    /* Barometer */
    STATUS_MESSAGE, /* Status message */
    ERROR_MESSAGE,  /* Error message */
};

/* A buffer that can hold any of the types of data created by the sensors in uorb_inputs */
union uorb_data {
    struct sensor_accel accel;
    struct sensor_gyro gyro;
    struct sensor_mag mag;
    struct sensor_gnss gnss;
    struct fusion_altitude alt;
    struct sensor_baro baro;
    struct status_message status;
    struct error_message error;
};

typedef struct {
    enum uorb_sensors type;
    union uorb_data data;
} uorb_block_t TIGHTLY_PACKED;

/* uORB polling file descriptors */

static struct pollfd uorb_fds[] = {
    [SENSOR_ACCEL] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GYRO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [STATUS_MESSAGE] = {.fd = -1, .events = POLLIN, .revents = 0},
    [ERROR_MESSAGE] = {.fd = -1, .events = POLLIN, .revents = 0},
};

/* uORB sensor metadatas */

ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_gnss);
ORB_DECLARE(fusion_altitude);
ORB_DECLARE(sensor_baro);
ORB_DECLARE(status_message);
ORB_DECLARE(error_message);

static struct orb_metadata const *uorb_metas[] = {
    [SENSOR_ACCEL] = ORB_ID(sensor_accel),     [SENSOR_GYRO] = ORB_ID(sensor_gyro),
    [SENSOR_MAG] = ORB_ID(sensor_mag),         [SENSOR_GNSS] = ORB_ID(sensor_gnss),
    [SENSOR_ALT] = ORB_ID(fusion_altitude),    [SENSOR_BARO] = ORB_ID(sensor_baro),
    [STATUS_MESSAGE] = ORB_ID(status_message), [ERROR_MESSAGE] = ORB_ID(error_message),
};

/* The numbers of sensors that are available to be polled */

#define NUM_SENSORS (sizeof(uorb_fds) / sizeof(uorb_fds[0]))

static int log_buffer(FILE *storage, uint8_t *buffer, size_t buffer_size);
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
    unsigned int packet_seq_num = 0;
    bool ejectled_on = false;
    FILE *active_file = NULL;

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

    /* subscribe to topics */
    for (int i = 0; i < NUM_SENSORS; i++) {

        /* Skip metadata that couldn't be found */
        if (uorb_metas[i] == NULL) {
            inerr("Missing metadata for sensor %d\n", i);
            continue;
        }

        ininfo("Subscribing to '%s'\n", uorb_metas[i]->o_name);
        uorb_fds[i].fd = orb_subscribe(uorb_metas[i]);
        if (uorb_fds[i].fd < 0) {
            inerr("Failed to subscribe to '%s': %d\n", uorb_metas[i]->o_name, errno);
        }
    }

    union uorb_data data_buf[10];

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);
    uint32_t mission_time_ms = current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;

    int log_err = 0;

    for (;;) {
        poll(uorb_fds, NUM_SENSORS, -1);

        for (int i = 0; i < NUM_SENSORS; i++) {

            /* Skip invalid sensors and sensors without new data */

            if (uorb_fds[i].fd < 0 || !(uorb_fds[i].revents & POLLIN)) {
                continue;
            }

            uorb_fds[i].revents = 0; /* Mark the event as handled */

            err = orb_copy_multi(uorb_fds[i].fd, &data_buf, sizeof(data_buf));
            if (err < 0) {
                inerr("Error reading data from %s: %d\n", uorb_metas[i]->o_name, err);
                continue;
            }

            for (int j = 0; j < (err / uorb_metas[i]->o_size); j++) {
                switch (i) {

                case SENSOR_ACCEL: {
                    uint8_t sensor_type = SENSOR_ACCEL;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct sensor_accel *accel = &((struct sensor_accel *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)accel, sizeof(struct sensor_accel));
                    break;
                }
                case SENSOR_GYRO: {
                    uint8_t sensor_type = SENSOR_GYRO;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct sensor_gyro *gyro = &((struct sensor_gyro *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)gyro, sizeof(struct sensor_gyro));
                    break;
                }
                case SENSOR_MAG: {
                    uint8_t sensor_type = SENSOR_MAG;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct sensor_mag *mag = &((struct sensor_mag *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)mag, sizeof(struct sensor_mag));
                    break;
                }
                case SENSOR_GNSS: {
                    uint8_t sensor_type = SENSOR_GNSS;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct sensor_gnss *gnss = &((struct sensor_gnss *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)gnss, sizeof(struct sensor_gnss));
                    break;
                }
                case SENSOR_ALT: {
                    uint8_t sensor_type = SENSOR_ALT;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct fusion_altitude *alt = &((struct fusion_altitude *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)alt, sizeof(struct fusion_altitude));
                    break;
                }
                case SENSOR_BARO: {
                    uint8_t sensor_type = SENSOR_BARO;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct sensor_baro *baro = &((struct sensor_baro *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)baro, sizeof(struct sensor_baro));
                    break;
                }
                case STATUS_MESSAGE: {
                    uint8_t sensor_type = STATUS_MESSAGE;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct status_message *status = &((struct status_message *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)status, sizeof(struct status_message));
                    break;
                }
                case ERROR_MESSAGE: {
                    uint8_t sensor_type = ERROR_MESSAGE;
                    log_err = log_buffer(active_file, (uint8_t *)&sensor_type, sizeof(sensor_type));
                    if (log_err < 0) break;

                    struct error_message *error = &((struct error_message *)data_buf)[j];
                    log_err = log_buffer(active_file, (uint8_t *)error, sizeof(struct error_message));
                    break;
                }
                }

                if (log_err < 0) {
                    if (log_err == -EFBIG) {
                        inerr("File is too big, attempting to recover\n");
                        close_synced(active_file);

                        err = open_log_file(&active_file, FLIGHT_FPATH_FMT, mission_num, flight_ser_num++, "w+");
                        if (err < 0) {
                            inerr("Error opening new log file: %d\n", err);
                            goto err_cleanup;
                        }

                        /* reattempt the same block */
                        j -= 1;
                        continue;
                    } else {
                        inerr("Error logging data: %d\n", log_err);
                        goto err_cleanup;
                    }
                }

                packet_seq_num++;

                if (packet_seq_num % CONFIG_INSPACE_TELEMETRY_FS_SYNC_FREQ == 0) {
                    indebug("Syncing file\n");
                    fsync(fileno(active_file));
                }
            }
        }
    }

err_cleanup:
    /* Close files that may be open */
    if (active_file && close_synced(active_file) != 0) {
        err = errno;
        inerr("Failed to close active file: %d\n", err);
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
static int log_buffer(FILE *storage, uint8_t *buffer, size_t buffer_size) {
    size_t written = fwrite(buffer, 1, buffer_size, storage);
    if (written != buffer_size) {
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
