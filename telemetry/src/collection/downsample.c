
#include "downsample.h"
#include "../fusion/fusion.h"
#include "../syslogging.h"
#include "status-update.h"
#include "uORB/uORB.h"
#include <fcntl.h>
#include <math.h>
#include <nuttx/sensors/sensor.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

enum uorb_sensors {
    SENSOR_ACCEL, /* Accelerometer */
    SENSOR_GYRO,  /* Gyroscope */
    SENSOR_MAG,   /* Magnetometer */
    SENSOR_GNSS,  /* GNSS */
    SENSOR_ALT,   /* Altitude fusion */
    SENSOR_BARO,
};

/* A buffer that can hold any of the types of data created by the sensors in uorb_inputs */
union uorb_data {
    struct sensor_accel accel;
    struct sensor_gyro gyro;
    struct sensor_mag mag;
    struct sensor_gnss gnss;
    struct fusion_altitude alt;
    struct sensor_baro baro;
};

/* uORB polling file descriptors */

static struct pollfd uorb_fds[] = {
    [SENSOR_ACCEL] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GYRO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
};

/* uORB sensor metadatas */

ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_gnss);
ORB_DECLARE(fusion_altitude);
ORB_DECLARE(sensor_baro);

static struct orb_metadata const *uorb_metas[] = {
    [SENSOR_ACCEL] = ORB_ID(sensor_accel), [SENSOR_GYRO] = ORB_ID(sensor_gyro),    [SENSOR_MAG] = ORB_ID(sensor_mag),
    [SENSOR_GNSS] = ORB_ID(sensor_gnss),   [SENSOR_ALT] = ORB_ID(fusion_altitude), [SENSOR_BARO] = ORB_ID(sensor_baro),
};

/* The default sampling rate for low-sample sensors or topics */

#define LOW_SAMPLE_RATE_DEFAULT 10

static const uint32_t sample_freqs[] = {
    [SENSOR_ACCEL] = CONFIG_INSPACE_TELEMETRY_ACCEL_SF, [SENSOR_GYRO] = CONFIG_INSPACE_TELEMETRY_GYRO_SF,
    [SENSOR_MAG] = CONFIG_INSPACE_TELEMETRY_MAG_SF,     [SENSOR_GNSS] = CONFIG_INSPACE_TELEMETRY_GPS_SF,
    [SENSOR_BARO] = CONFIG_INSPACE_TELEMETRY_BARO_SF,
};

/* Data buffer for copying uORB data */

static uint8_t data_buf[sizeof(union uorb_data) * 10];

/* The numbers of sensors that are available to be polled */

#define NUM_SENSORS (sizeof(uorb_fds) / sizeof(uorb_fds[0]))

typedef struct {
    float out[3];             /* downsampled output, 3 fields for sensors with 3 axes */
    uint16_t window_n;        /* number of samples accumulated in the current downsampling window */
    uint16_t target_window_n; /* target number of input samples per downsampled output */
    uint16_t total_n;         /* total samples since last buffer swap, used for dynamic rate adjustment */
    uint16_t dropped_n;       /* output blocks dropped since last buffer swap */
    uint16_t output_n;        /* output blocks written since last swap */
} sensor_downsampling_t;

static sensor_downsampling_t sensor_downsamples[] = {
    [SENSOR_ACCEL] = {.target_window_n = CONFIG_INSPACE_TELEMETRY_ACCEL_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_GYRO] = {.target_window_n = CONFIG_INSPACE_TELEMETRY_GYRO_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_MAG] = {.target_window_n = CONFIG_INSPACE_TELEMETRY_MAG_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_GNSS] = {.target_window_n = CONFIG_INSPACE_TELEMETRY_GPS_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_ALT] = {.target_window_n = CONFIG_INSPACE_TELEMETRY_ALT_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
};

/*
 * Downsample thread, takes data in from uorb topics and downsamples it to the target frequency
 */
void *downsample_main(void *arg) {
    int err;
    struct downsample_args *unpacked_args = (struct downsample_args *)(arg);
    radio_telem_t *radio_telem = unpacked_args->radio_telem;

    ininfo("Downsample thread started.\n");

    /* Subscribe to all sensors */

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

    ininfo("Sensors subscribed.\n");

    /* Set sensor specific requirements */
    /* TODO: move this to the main or init thread */

    ininfo("Configuring sensors with their specific requirements.\n");

    if (uorb_fds[SENSOR_ACCEL].fd >= 0) {
        ininfo("Configuring accelerometer FSR to +/-32g.\n");
        err = orb_ioctl(uorb_fds[SENSOR_ACCEL].fd, SNIOC_SETFULLSCALE, 32);
        if (err < 0) {
            inerr("Couldn't set FSR of sensor_accel: %d\n", errno);
        }
    }

    if (uorb_fds[SENSOR_GYRO].fd >= 0) {
        ininfo("Configuring gyro FSR to +/-2000dps.\n");
        err = orb_ioctl(uorb_fds[SENSOR_GYRO].fd, SNIOC_SETFULLSCALE, 2000);
        if (err < 0) {
            inerr("Couldn't set FSR of sensor_gyro: %d\n", errno);
        }
    }

#ifndef CONFIG_INSPACE_MOCKING
    /* Set sample frequencies for all sensors
     *
     * NOTE: setting frequencies when mocking will break some flight records
     * since all measurements in the CSV are synced line to line, so if we set
     * accel to be faster than barometer, the measurements will be out of sync
     * chronologically. Hence, disable frequency configurations when mocking.
     */

    ininfo("Setting sensor sample frequencies.\n");

    for (int i = 0; i < sizeof(sample_freqs) / sizeof(sample_freqs[0]); i++) {

        /* Skip invalid sensors */

        if (uorb_fds[i].fd < 0) {
            continue;
        }

        ininfo("Setting frequency of '%s' to %luHz\n", uorb_metas[i]->o_name, sample_freqs[i]);
        err = orb_set_frequency(uorb_fds[i].fd, sample_freqs[i]);
        if (err < 0) {
            inerr("Failed to set frequency of '%s' to %luHz: %d\n", uorb_metas[i]->o_name, sample_freqs[i], errno);
        }
    }

    ininfo("Sensor frequencies set.\n");
#endif

    /* keepint track of sampling rates */
    uint32_t rate_counts[NUM_SENSORS] = {0};
    struct timespec rate_report_start;
    clock_gettime(CLOCK_MONOTONIC, &rate_report_start);

    /* Measure data forever */
    for (;;) {
        poll(uorb_fds, NUM_SENSORS, -1);

        if (sem_trywait(&radio_telem->swapped) == 0) {
            for (int k = 0; k < NUM_SENSORS; k++) {
                if (k == SENSOR_BARO) continue;
                int new_target_window_n = sensor_downsamples[k].total_n / (CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ - 1);
                if (sensor_downsamples[k].dropped_n > 0) {
                    new_target_window_n = new_target_window_n *
                                          (CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ + sensor_downsamples[k].dropped_n) /
                                          CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ;
                }
                if (new_target_window_n > 0 && new_target_window_n != sensor_downsamples[k].target_window_n) {
                    sensor_downsamples[k].target_window_n = new_target_window_n;
                }
                sensor_downsamples[k].total_n = 0;
                sensor_downsamples[k].dropped_n = 0;
                sensor_downsamples[k].output_n = 0;
            }
        }

        for (int i = 0; i < NUM_SENSORS; i++) {

            /* Skip invalid sensors and sensors without new data */

            if (uorb_fds[i].fd < 0 || !(uorb_fds[i].revents & POLL_IN)) {
                continue;
            }

            /* A valid sensor has some data ready to be read. Add it to a packet. */

            uorb_fds[i].revents = 0; /* Mark the event as handled */

            err = orb_copy_multi(uorb_fds[i].fd, &data_buf, sizeof(data_buf));
            if (err < 0) {
                inerr("Error reading data from %s: %d\n", uorb_metas[i]->o_name, errno);
                continue;
            }

            for (int j = 0; j < (err / uorb_metas[i]->o_size); j++) {
                sensor_downsamples[i].total_n++;
                sensor_downsamples[i].window_n++;
                rate_counts[i]++;

                switch (i) {
                case SENSOR_ACCEL: {
                    if (radio_telem->empty_buff->accel_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                        sensor_downsamples[SENSOR_ACCEL].dropped_n++;
                        sensor_downsamples[SENSOR_ACCEL].window_n = 0;
                        break;
                    }

                    struct sensor_accel accel_input = ((struct sensor_accel *)data_buf)[j];

                    /*
                    using the Welford formula to calculate the mean, one pass for each axis
                    mean = mean_(n-1) + (x - mean_(n-1)) / n
                    */
                    sensor_downsamples[SENSOR_ACCEL].out[0] +=
                        (accel_input.x - sensor_downsamples[SENSOR_ACCEL].out[0]) /
                        sensor_downsamples[SENSOR_ACCEL].window_n;
                    sensor_downsamples[SENSOR_ACCEL].out[1] +=
                        (accel_input.y - sensor_downsamples[SENSOR_ACCEL].out[1]) /
                        sensor_downsamples[SENSOR_ACCEL].window_n;
                    sensor_downsamples[SENSOR_ACCEL].out[2] +=
                        (accel_input.z - sensor_downsamples[SENSOR_ACCEL].out[2]) /
                        sensor_downsamples[SENSOR_ACCEL].window_n;

                    if (sensor_downsamples[SENSOR_ACCEL].window_n >= sensor_downsamples[SENSOR_ACCEL].target_window_n) {

                        struct sensor_accel sensor_accel = {
                            .timestamp = accel_input.timestamp,
                            .x = sensor_downsamples[SENSOR_ACCEL].out[0],
                            .y = sensor_downsamples[SENSOR_ACCEL].out[1],
                            .z = sensor_downsamples[SENSOR_ACCEL].out[2],
                        };

                        radio_telem->empty_buff->accel[radio_telem->empty_buff->accel_n++] = sensor_accel;
                        sensor_downsamples[SENSOR_ACCEL].output_n++;
                        sensor_downsamples[SENSOR_ACCEL].out[0] = 0;
                        sensor_downsamples[SENSOR_ACCEL].out[1] = 0;
                        sensor_downsamples[SENSOR_ACCEL].out[2] = 0;
                        sensor_downsamples[SENSOR_ACCEL].window_n = 0;
                    }

                    break;
                }
                case SENSOR_GYRO: {
                    if (radio_telem->empty_buff->gyro_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                        sensor_downsamples[SENSOR_GYRO].dropped_n++;
                        sensor_downsamples[SENSOR_GYRO].window_n = 0;
                        break;
                    }

                    struct sensor_gyro gyro_input = ((struct sensor_gyro *)data_buf)[j];

                    /*
                    using the Welford formula to calculate the mean, one pass for each axis
                    mean = mean_(n-1) + (x - mean_(n-1)) / n
                    */
                    sensor_downsamples[SENSOR_GYRO].out[0] += (gyro_input.x - sensor_downsamples[SENSOR_GYRO].out[0]) /
                                                              sensor_downsamples[SENSOR_GYRO].window_n;
                    sensor_downsamples[SENSOR_GYRO].out[1] += (gyro_input.y - sensor_downsamples[SENSOR_GYRO].out[1]) /
                                                              sensor_downsamples[SENSOR_GYRO].window_n;
                    sensor_downsamples[SENSOR_GYRO].out[2] += (gyro_input.z - sensor_downsamples[SENSOR_GYRO].out[2]) /
                                                              sensor_downsamples[SENSOR_GYRO].window_n;

                    if (sensor_downsamples[SENSOR_GYRO].window_n >= sensor_downsamples[SENSOR_GYRO].target_window_n) {
                        struct sensor_gyro sensor_gyro = {
                            .timestamp = gyro_input.timestamp,
                            .x = sensor_downsamples[SENSOR_GYRO].out[0],
                            .y = sensor_downsamples[SENSOR_GYRO].out[1],
                            .z = sensor_downsamples[SENSOR_GYRO].out[2],
                        };

                        radio_telem->empty_buff->gyro[radio_telem->empty_buff->gyro_n++] = sensor_gyro;
                        sensor_downsamples[SENSOR_GYRO].output_n++;
                        sensor_downsamples[SENSOR_GYRO].out[0] = 0;
                        sensor_downsamples[SENSOR_GYRO].out[1] = 0;
                        sensor_downsamples[SENSOR_GYRO].out[2] = 0;
                        sensor_downsamples[SENSOR_GYRO].window_n = 0;
                    }

                    break;
                }
                case SENSOR_MAG: {
                    if (radio_telem->empty_buff->mag_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                        sensor_downsamples[SENSOR_MAG].dropped_n++;
                        sensor_downsamples[SENSOR_MAG].window_n = 0;
                        break;
                    }

                    struct sensor_mag mag_input = ((struct sensor_mag *)data_buf)[j];

                    /*
                    using the Welford formula to calculate the mean, one pass for each axis
                    mean = mean_(n-1) + (x - mean_(n-1)) / n
                    */
                    sensor_downsamples[SENSOR_MAG].out[0] +=
                        (mag_input.x - sensor_downsamples[SENSOR_MAG].out[0]) / sensor_downsamples[SENSOR_MAG].window_n;
                    sensor_downsamples[SENSOR_MAG].out[1] +=
                        (mag_input.y - sensor_downsamples[SENSOR_MAG].out[1]) / sensor_downsamples[SENSOR_MAG].window_n;
                    sensor_downsamples[SENSOR_MAG].out[2] +=
                        (mag_input.z - sensor_downsamples[SENSOR_MAG].out[2]) / sensor_downsamples[SENSOR_MAG].window_n;

                    if (sensor_downsamples[SENSOR_MAG].window_n >= sensor_downsamples[SENSOR_MAG].target_window_n) {
                        struct sensor_mag sensor_mag = {
                            .timestamp = mag_input.timestamp,
                            .x = sensor_downsamples[SENSOR_MAG].out[0],
                            .y = sensor_downsamples[SENSOR_MAG].out[1],
                            .z = sensor_downsamples[SENSOR_MAG].out[2],
                        };

                        if (radio_telem->empty_buff->mag_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            inerr("Magnetometer buffer full, dropping data\n");
                            sensor_downsamples[SENSOR_MAG].dropped_n++;
                            sensor_downsamples[SENSOR_MAG].window_n = 0;
                            break;
                        }

                        radio_telem->empty_buff->mag[radio_telem->empty_buff->mag_n++] = sensor_mag;
                        sensor_downsamples[SENSOR_MAG].output_n++;
                        sensor_downsamples[SENSOR_MAG].out[0] = 0;
                        sensor_downsamples[SENSOR_MAG].out[1] = 0;
                        sensor_downsamples[SENSOR_MAG].out[2] = 0;
                        sensor_downsamples[SENSOR_MAG].window_n = 0;
                    }

                    break;
                }
                case SENSOR_GNSS: {
                    if (radio_telem->empty_buff->gnss_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                        sensor_downsamples[SENSOR_GNSS].dropped_n++;
                        sensor_downsamples[SENSOR_GNSS].window_n = 0;
                        break;
                    }

                    struct sensor_gnss gnss_input = ((struct sensor_gnss *)data_buf)[j];

                    /*
                    using the Welford formula to calculate the mean, one pass for each axis
                    mean = mean_(n-1) + (x - mean_(n-1)) / n
                    */
                    sensor_downsamples[SENSOR_GNSS].out[0] +=
                        (gnss_input.latitude - sensor_downsamples[SENSOR_GNSS].out[0]) /
                        sensor_downsamples[SENSOR_GNSS].window_n;
                    sensor_downsamples[SENSOR_GNSS].out[1] +=
                        (gnss_input.longitude - sensor_downsamples[SENSOR_GNSS].out[1]) /
                        sensor_downsamples[SENSOR_GNSS].window_n;

                    if (sensor_downsamples[SENSOR_GNSS].window_n >= sensor_downsamples[SENSOR_GNSS].target_window_n) {
                        struct sensor_gnss sensor_gnss = {
                            .timestamp = gnss_input.timestamp,
                            .latitude = sensor_downsamples[SENSOR_GNSS].out[0],
                            .longitude = sensor_downsamples[SENSOR_GNSS].out[1],
                        };

                        if (radio_telem->empty_buff->gnss_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            inerr("GNSS buffer full, dropping data\n");
                            sensor_downsamples[SENSOR_GNSS].dropped_n++;
                            sensor_downsamples[SENSOR_GNSS].window_n = 0;
                            break;
                        }

                        radio_telem->empty_buff->gnss[radio_telem->empty_buff->gnss_n++] = sensor_gnss;
                        sensor_downsamples[SENSOR_GNSS].output_n++;
                        sensor_downsamples[SENSOR_GNSS].out[0] = 0;
                        sensor_downsamples[SENSOR_GNSS].out[1] = 0;
                        sensor_downsamples[SENSOR_GNSS].window_n = 0;
                    }

                    break;
                }
                case SENSOR_ALT: {
                    if (radio_telem->empty_buff->alt_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                        sensor_downsamples[SENSOR_ALT].dropped_n++;
                        sensor_downsamples[SENSOR_ALT].window_n = 0;
                        break;
                    }

                    struct fusion_altitude alt_input = ((struct fusion_altitude *)data_buf)[j];

                    /*
                    using the Welford formula to calculate the mean, one pass for each axis
                    mean = mean_(n-1) + (x - mean_(n-1)) / n
                    */
                    sensor_downsamples[SENSOR_ALT].out[0] +=
                        (alt_input.altitude - sensor_downsamples[SENSOR_ALT].out[0]) /
                        sensor_downsamples[SENSOR_ALT].window_n;

                    if (sensor_downsamples[SENSOR_ALT].window_n >= sensor_downsamples[SENSOR_ALT].target_window_n) {
                        struct fusion_altitude sensor_alt = {
                            .timestamp = alt_input.timestamp,
                            .altitude = sensor_downsamples[SENSOR_ALT].out[0],
                        };

                        if (radio_telem->empty_buff->alt_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            inerr("Altitude buffer full, dropping data\n");
                            sensor_downsamples[SENSOR_ALT].dropped_n++;
                            sensor_downsamples[SENSOR_ALT].window_n = 0;
                            break;
                        }

                        radio_telem->empty_buff->alt[radio_telem->empty_buff->alt_n++] = sensor_alt;
                        sensor_downsamples[SENSOR_ALT].output_n++;
                        sensor_downsamples[SENSOR_ALT].out[0] = 0;
                        sensor_downsamples[SENSOR_ALT].window_n = 0;
                    }
                    break;
                }
                }
            }
        }
    }

    publish_error(PROC_ID_DOWNSAMPLE, ERROR_PROCESS_DEAD);
    pthread_exit(0);
}