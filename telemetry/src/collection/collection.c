/* The collection thread is responsible for collecting all the sensor measurements available and packaging them into the
 * correct packet format. Packets are then published to both the logging and transmission sinks so that they can be
 * output properly.
 */

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/analog/adc.h>
#include <nuttx/sensors/sensor.h>

#include <nuttx/analog/adc.h>
#include <nuttx/sensors/sensor.h>

#include "../fusion/fusion.h"
#include "../syslogging.h"
#include "collection.h"
#include "status-update.h"
#include "uORB/uORB.h"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* Minimum buffer size for copying multiple amounts of data at once */
#define DATA_BUF_MIN 10

enum uorb_sensors {
    SENSOR_ACCEL, /* Accelerometer */
    SENSOR_GYRO, /* Gyroscope */
    SENSOR_MAG, /* Magnetometer */
    SENSOR_GNSS, /* GNSS */
    SENSOR_ALT,    /* Altitude fusion */
};

/* A buffer that can hold any of the types of data created by the sensors in uorb_inputs */
union uorb_data {
    struct sensor_accel accel;
    struct sensor_gyro gyro;
    struct sensor_mag mag;
    struct sensor_gnss gnss;
    struct fusion_altitude alt;
};

/* uORB polling file descriptors */

static struct pollfd uorb_fds[] = {
    [SENSOR_ACCEL] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GYRO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
};

/* uORB sensor metadatas */

ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_gnss);
ORB_DECLARE(fusion_altitude);

static struct orb_metadata const *uorb_metas[] = {
    [SENSOR_ACCEL] = ORB_ID(sensor_accel),
    [SENSOR_GYRO] = ORB_ID(sensor_gyro),
    [SENSOR_MAG] = ORB_ID(sensor_mag),
    [SENSOR_GNSS] = ORB_ID(sensor_gnss),
    [SENSOR_ALT] = ORB_ID(fusion_altitude)
};

/* The default sampling rate for low-sample sensors or topics */

#define LOW_SAMPLE_RATE_DEFAULT 10

static const uint32_t sample_freqs[] = {
    [SENSOR_ACCEL] = CONFIG_INSPACE_TELEMETRY_ACCEL_SF,
    [SENSOR_GYRO] = CONFIG_INSPACE_TELEMETRY_GYRO_SF,
    [SENSOR_MAG] = CONFIG_INSPACE_TELEMETRY_MAG_SF,
    [SENSOR_GNSS] = CONFIG_INSPACE_TELEMETRY_GPS_SF
};

/* Data buffer for copying uORB data */

static uint8_t data_buf[sizeof(union uorb_data) * DATA_BUF_MIN];

/* The numbers of sensors that are available to be polled */

#define NUM_SENSORS (sizeof(uorb_fds) / sizeof(uorb_fds[0]))

struct sensor_downsampling_t {
    int64_t mean[3];
    uint16_t count;
    uint16_t rate;
};

static struct sensor_downsampling_t sensor_downsamples[] = {
    [SENSOR_ACCEL] = {.rate = CONFIG_INSPACE_TELEMETRY_ACCEL_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_GYRO] = {.rate = CONFIG_INSPACE_TELEMETRY_GYRO_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_MAG] = {.rate = CONFIG_INSPACE_TELEMETRY_MAG_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_GNSS] = {.rate = CONFIG_INSPACE_TELEMETRY_GPS_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
    [SENSOR_ALT] = {.rate = CONFIG_INSPACE_TELEMETRY_ALT_SF / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ},
};

/*
 * Collection thread.
 *
 * Runs to collect data from sensors and battery ADC, packaging all measurements into packets.
 */
void *collection_main(void *arg) {
    int err;
    struct collection_args *unpacked_args = (struct collection_args *)(arg);
    radio_telem_t *radio_telem = unpacked_args->radio_telem;

    ininfo("Collection thread started.\n");

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

    ininfo("Configuring sensors with their specific requirements.\n");

    if(uorb_fds[SENSOR_ACCEL].fd >= 0) {
        ininfo("Configuring accelerometer FSR to +/-32g.\n");
        err = orb_ioctl(uorb_fds[SENSOR_ACCEL].fd, SNIOC_SETFULLSCALE, 32);
        if (err < 0) {
            inerr("Couldn't set FSR of sensor_accel: %d\n", errno);
        }
    }

    if(uorb_fds[SENSOR_GYRO].fd >= 0) {
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

    for (int i = 0; i < NUM_SENSORS; i++) {

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

    /* Measure data forever */
    for (;;) {
        /* Wait for new data */

        poll(uorb_fds, NUM_SENSORS, -1);

        pthread_mutex_lock(&radio_telem->empty_mux);

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

            /* check if telem thead swapped the buffer */
            if(radio_telem->empty->accel_n == -1 || radio_telem->empty->gyro_n == -1 || radio_telem->empty->mag_n == -1 || radio_telem->empty->gnss_n == -1 || radio_telem->empty->alt_n == -1){

                /* adjust downsampling rates and reset internal state */
                for(int k = 0; k < sizeof(sensor_downsamples) / sizeof(sensor_downsamples[0]); k++){
                    int updated_rate = sensor_downsamples[k].count / CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ;
                    if(updated_rate > 0 && (sensor_downsamples[k].rate != updated_rate)) {
                        sensor_downsamples[k].rate = updated_rate;
                    }

                    sensor_downsamples[k].count = 0;
                    sensor_downsamples[k].mean[0] = 0;
                    sensor_downsamples[k].mean[1] = 0;
                    sensor_downsamples[k].mean[2] = 0;
                }

                radio_telem->empty->accel_n = 0;
                radio_telem->empty->gyro_n = 0;
                radio_telem->empty->mag_n = 0;
                radio_telem->empty->gnss_n = 0;
                radio_telem->empty->alt_n = 0;
            }

            for (int j = 0; j < (err / uorb_metas[i]->o_size); j++) {
                sensor_downsamples[i].count++;

                /* using the Welford formula to calculate the mean */
                switch (i) {
                    case SENSOR_ACCEL: {
                        if(radio_telem->empty->accel_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            break;
                        }

                        struct sensor_accel accel_input = ((struct sensor_accel *)data_buf)[j];

                        sensor_downsamples[SENSOR_ACCEL].mean[0] += (accel_input.x - sensor_downsamples[SENSOR_ACCEL].mean[0]) / sensor_downsamples[SENSOR_ACCEL].count;
                        sensor_downsamples[SENSOR_ACCEL].mean[1] += (accel_input.y - sensor_downsamples[SENSOR_ACCEL].mean[1]) / sensor_downsamples[SENSOR_ACCEL].count;
                        sensor_downsamples[SENSOR_ACCEL].mean[2] += (accel_input.z - sensor_downsamples[SENSOR_ACCEL].mean[2]) / sensor_downsamples[SENSOR_ACCEL].count;

                        if(sensor_downsamples[SENSOR_ACCEL].count % sensor_downsamples[SENSOR_ACCEL].rate == 0) {

                            struct sensor_accel sensor_accel = {
                                .timestamp = accel_input.timestamp,
                                .x = sensor_downsamples[SENSOR_ACCEL].mean[0],
                                .y = sensor_downsamples[SENSOR_ACCEL].mean[1],
                                .z = sensor_downsamples[SENSOR_ACCEL].mean[2],
                            };

                            radio_telem->empty->accel[radio_telem->empty->accel_n++] = sensor_accel;
                            sensor_downsamples[SENSOR_ACCEL].mean[0] = 0;
                            sensor_downsamples[SENSOR_ACCEL].mean[1] = 0;
                            sensor_downsamples[SENSOR_ACCEL].mean[2] = 0;
                        }

                        break;
                    }
                    case SENSOR_GYRO: {
                        if(radio_telem->empty->gyro_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            break;
                        }

                        struct sensor_gyro gyro_input = ((struct sensor_gyro *)data_buf)[j];

                        sensor_downsamples[SENSOR_GYRO].mean[0] += (gyro_input.x - sensor_downsamples[SENSOR_GYRO].mean[0]) / sensor_downsamples[SENSOR_GYRO].count;
                        sensor_downsamples[SENSOR_GYRO].mean[1] += (gyro_input.y - sensor_downsamples[SENSOR_GYRO].mean[1]) / sensor_downsamples[SENSOR_GYRO].count;
                        sensor_downsamples[SENSOR_GYRO].mean[2] += (gyro_input.z - sensor_downsamples[SENSOR_GYRO].mean[2]) / sensor_downsamples[SENSOR_GYRO].count;

                        if(sensor_downsamples[SENSOR_GYRO].count % sensor_downsamples[SENSOR_GYRO].rate == 0) {
                            struct sensor_gyro sensor_gyro = {
                                .timestamp = gyro_input.timestamp,
                                .x = sensor_downsamples[SENSOR_GYRO].mean[0],
                                .y = sensor_downsamples[SENSOR_GYRO].mean[1],
                                .z = sensor_downsamples[SENSOR_GYRO].mean[2],
                            };
                            radio_telem->empty->gyro[radio_telem->empty->gyro_n++] = sensor_gyro;
                            sensor_downsamples[SENSOR_GYRO].mean[0] = 0;
                            sensor_downsamples[SENSOR_GYRO].mean[1] = 0;
                            sensor_downsamples[SENSOR_GYRO].mean[2] = 0;
                        }

                        break;
                    }
                    case SENSOR_MAG: {
                        if(radio_telem->empty->mag_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            break;
                        }

                        struct sensor_mag mag_input = ((struct sensor_mag *)data_buf)[j];

                        sensor_downsamples[SENSOR_MAG].mean[0] += (mag_input.x - sensor_downsamples[SENSOR_MAG].mean[0]) / sensor_downsamples[SENSOR_MAG].count;
                        sensor_downsamples[SENSOR_MAG].mean[1] += (mag_input.y - sensor_downsamples[SENSOR_MAG].mean[1]) / sensor_downsamples[SENSOR_MAG].count;
                        sensor_downsamples[SENSOR_MAG].mean[2] += (mag_input.z - sensor_downsamples[SENSOR_MAG].mean[2]) / sensor_downsamples[SENSOR_MAG].count;

                        if(sensor_downsamples[SENSOR_MAG].count % sensor_downsamples[SENSOR_MAG].rate == 0) {
                            struct sensor_mag sensor_mag = {
                                .timestamp = mag_input.timestamp,
                                .x = sensor_downsamples[SENSOR_MAG].mean[0],
                                .y = sensor_downsamples[SENSOR_MAG].mean[1],
                                .z = sensor_downsamples[SENSOR_MAG].mean[2],
                            };

                            if(radio_telem->empty->mag_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                                inerr("Magnetometer buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->mag[radio_telem->empty->mag_n++] = sensor_mag;

                            sensor_downsamples[SENSOR_MAG].mean[0] = 0;
                            sensor_downsamples[SENSOR_MAG].mean[1] = 0;
                            sensor_downsamples[SENSOR_MAG].mean[2] = 0;
                        }

                        break;
                    }
                    case SENSOR_GNSS: {
                        if(radio_telem->empty->gnss_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            break;
                        }

                        struct sensor_gnss gnss_input = ((struct sensor_gnss *)data_buf)[j];

                        sensor_downsamples[SENSOR_GNSS].mean[0] += (gnss_input.latitude - sensor_downsamples[SENSOR_GNSS].mean[0]) / sensor_downsamples[SENSOR_GNSS].count;
                        sensor_downsamples[SENSOR_GNSS].mean[1] += (gnss_input.longitude - sensor_downsamples[SENSOR_GNSS].mean[1]) / sensor_downsamples[SENSOR_GNSS].count;

                        if(sensor_downsamples[SENSOR_GNSS].count % sensor_downsamples[SENSOR_GNSS].rate == 0) {
                            struct sensor_gnss sensor_gnss = {
                                .timestamp = gnss_input.timestamp,
                                .latitude = sensor_downsamples[SENSOR_GNSS].mean[0],
                                .longitude = sensor_downsamples[SENSOR_GNSS].mean[1],
                            };

                            if(radio_telem->empty->gnss_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                                inerr("GNSS buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->gnss[radio_telem->empty->gnss_n++] = sensor_gnss;

                            sensor_downsamples[SENSOR_GNSS].mean[0] = 0;
                            sensor_downsamples[SENSOR_GNSS].mean[1] = 0;
                        }

                        break;
                    }
                    case SENSOR_ALT: {
                        if(radio_telem->empty->alt_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                            break;
                        }

                        struct fusion_altitude alt_input = ((struct fusion_altitude *)data_buf)[j];

                        sensor_downsamples[SENSOR_ALT].mean[0] += (alt_input.altitude - sensor_downsamples[SENSOR_ALT].mean[0]) / sensor_downsamples[SENSOR_ALT].count;

                        if(sensor_downsamples[SENSOR_ALT].count % sensor_downsamples[SENSOR_ALT].rate == 0) {
                            struct fusion_altitude sensor_alt = {
                                .timestamp = alt_input.timestamp,
                                .altitude = sensor_downsamples[SENSOR_ALT].mean[0],
                            };

                            if(radio_telem->empty->alt_n == CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ) {
                                inerr("Altitude buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->alt[radio_telem->empty->alt_n++] = sensor_alt;

                            sensor_downsamples[SENSOR_ALT].mean[0] = 0;
                        }
                        break;
                    }
                }
            }
        }

        pthread_mutex_unlock(&radio_telem->empty_mux);
    }
    publish_error(PROC_ID_COLLECTION, ERROR_PROCESS_DEAD);
    pthread_exit(0);
}