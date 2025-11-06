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

/* Unit conversion helpers */

#define us_to_ms(us) (us / 1000)
#define pascals(millibar) (millibar * 100)
#define millimeters(meters) (meters * 1000)
#define point_one_microdegrees(degrees) (1E7f * degrees)
#define tenth_degree(radian) (radian * 18 / M_PI)
#define tenth_microtesla(microtesla) (microtesla * 1000)
#define cm_per_sec_squared(meters_per_sec_squared) (meters_per_sec_squared * 100)
#define millidegrees(celsius) (celsius * 1000)

/* Minimum buffer size for copying multiple amounts of data at once */
#define DATA_BUF_MIN 10

enum uorb_sensors {
    SENSOR_ACCEL, /* Accelerometer */
    SENSOR_GYRO, /* Gyroscope */
    SENSOR_BARO, /* Barometer */
    SENSOR_MAG, /* Magnetometer */
    SENSOR_GNSS, /* GNSS */
    SENSOR_ALT,    /* Altitude fusion */
    SENSOR_ERROR,  /* Error messages */
    SENSOR_STATUS, /* Status messages */
};

/* A buffer that can hold any of the types of data created by the sensors in uorb_inputs */
union uorb_data {
    struct sensor_accel accel;
    struct sensor_gyro gyro;
    struct sensor_baro baro;
    struct sensor_mag mag;
    struct sensor_gnss gnss;
    struct fusion_altitude alt;
    struct error_message error;
    struct status_message status;
};

/* uORB polling file descriptors */

static struct pollfd uorb_fds[] = {
    [SENSOR_ACCEL] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GYRO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_ERROR] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_STATUS] = {.fd = -1, .events = POLLIN, .revents = 0},
};

/* uORB sensor metadatas */

ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
ORB_DECLARE(sensor_baro);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_gnss);
ORB_DECLARE(fusion_altitude);

static struct orb_metadata const *uorb_metas[] = {
    [SENSOR_ACCEL] = ORB_ID(sensor_accel),
    [SENSOR_GYRO] = ORB_ID(sensor_gyro),
    [SENSOR_BARO] = ORB_ID(sensor_baro),
    [SENSOR_MAG] = ORB_ID(sensor_mag),
    [SENSOR_GNSS] = ORB_ID(sensor_gnss),
    [SENSOR_ALT] = ORB_ID(fusion_altitude),   [SENSOR_ERROR] = ORB_ID(error_message),
    [SENSOR_STATUS] = ORB_ID(status_message),
};

/* The default sampling rate for low-sample sensors or topics */

#define LOW_SAMPLE_RATE_DEFAULT 10

static const uint32_t sample_freqs[] = {
    [SENSOR_ACCEL] = CONFIG_INSPACE_TELEMETRY_ACCEL_SF,
    [SENSOR_GYRO] = CONFIG_INSPACE_TELEMETRY_GYRO_SF,
    [SENSOR_BARO] = CONFIG_INSPACE_TELEMETRY_BARO_SF,
    [SENSOR_MAG] = CONFIG_INSPACE_TELEMETRY_MAG_SF,
    [SENSOR_GNSS] = CONFIG_INSPACE_TELEMETRY_GPS_SF,
    [SENSOR_ALT] = CONFIG_INSPACE_TELEMETRY_ALT_SF,     
    [SENSOR_ERROR] = LOW_SAMPLE_RATE_DEFAULT,
    [SENSOR_STATUS] = LOW_SAMPLE_RATE_DEFAULT,
};

/* Data buffer for copying uORB data */

static uint8_t data_buf[sizeof(union uorb_data) * DATA_BUF_MIN];

/* The numbers of sensors that are available to be polled */

#define NUM_SENSORS (sizeof(uorb_fds) / sizeof(uorb_fds[0]))

struct sensor_downsampling_t {
    int64_t sum_buffer[3];
    uint16_t rate;
    uint16_t reads;
};

static struct sensor_downsampling_t sensor_downsamples[] = {
    [SENSOR_ACCEL] = {.rate = CONFIG_INSPACE_TELEMETRY_ACCEL_SF / CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ},
    [SENSOR_GYRO] = {.rate = CONFIG_INSPACE_TELEMETRY_GYRO_SF / CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ},
    [SENSOR_BARO] = {.rate = CONFIG_INSPACE_TELEMETRY_BARO_SF / CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ},
    [SENSOR_MAG] = {.rate = CONFIG_INSPACE_TELEMETRY_MAG_SF / CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ},
    [SENSOR_GNSS] = {.rate = CONFIG_INSPACE_TELEMETRY_GPS_SF / CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ},
    [SENSOR_ALT] = {.rate = CONFIG_INSPACE_TELEMETRY_ALT_SF / CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ},
    [SENSOR_ERROR] = {.rate = 1},
    [SENSOR_STATUS] = {.rate = 1}
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

            /* locking this early heavily simplifies the logic, i'm not locked in */
            pthread_mutex_lock(&radio_telem->empty_mux);

            /* check if telem thead swapped the buffer */
            if(radio_telem->empty->accel_n == -1 || radio_telem->empty->ang_vel_n == -1 || radio_telem->empty->mag_n == -1 || radio_telem->empty->gnss_n == -1 || radio_telem->empty->alt_n == -1){

                /* adjust downsampling rates and reset internal state */
                for(int k = 0; k < sizeof(sensor_downsamples) / sizeof(sensor_downsamples[0]); k++){
                    int updated_rate = sensor_downsamples[k].reads / CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ;
                    if(sensor_downsamples[k].rate != updated_rate) {
                        ininfo("Adjusted downsampling rate for %s from %d to %d\n", uorb_metas[k]->o_name, sensor_downsamples[k].rate, updated_rate);
                        sensor_downsamples[k].rate = updated_rate;
                    }

                    sensor_downsamples[k].reads = 0;
                    sensor_downsamples[k].sum_buffer[0] = 0;
                    sensor_downsamples[k].sum_buffer[1] = 0;
                    sensor_downsamples[k].sum_buffer[2] = 0;
                }

                radio_telem->empty->accel_n = 0;
                radio_telem->empty->ang_vel_n = 0;
                radio_telem->empty->mag_n = 0;
                radio_telem->empty->gnss_n = 0;
                radio_telem->empty->alt_n = 0;
            }

            for (int j = 0; j < (err / uorb_metas[i]->o_size); j++) {
                sensor_downsamples[i].reads++;

                /* skip readings if have more reads than the target frequency */
                if(sensor_downsamples[i].reads > sensor_downsamples[i].rate * CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ) {
                    continue;
                }

                switch (i) {
                    case SENSOR_ACCEL: {
                        struct sensor_accel accel = ((struct sensor_accel *)data_buf)[j];

                        sensor_downsamples[SENSOR_ACCEL].sum_buffer[0] += (int32_t)cm_per_sec_squared(accel.x);
                        sensor_downsamples[SENSOR_ACCEL].sum_buffer[1] += (int32_t)cm_per_sec_squared(accel.y);
                        sensor_downsamples[SENSOR_ACCEL].sum_buffer[2] += (int32_t)cm_per_sec_squared(accel.z);

                        /* we check whether the sum buffer is full, if it is we downsample and pass the output to the radio buffer */
                        if(sensor_downsamples[SENSOR_ACCEL].reads > 0 && sensor_downsamples[SENSOR_ACCEL].reads % sensor_downsamples[SENSOR_ACCEL].rate == 0) {
                            float x = sensor_downsamples[SENSOR_ACCEL].sum_buffer[0] / sensor_downsamples[SENSOR_ACCEL].rate;
                            float y = sensor_downsamples[SENSOR_ACCEL].sum_buffer[1] / sensor_downsamples[SENSOR_ACCEL].rate;
                            float z = sensor_downsamples[SENSOR_ACCEL].sum_buffer[2] / sensor_downsamples[SENSOR_ACCEL].rate;

                            struct accel_blk_t accel_blk = {
                                .time_offset = us_to_ms(accel.timestamp),
                                .x = x,
                                .y = y,
                                .z = z,
                            };

                            /* send the packet to the radio buffer */
                            if(radio_telem->empty->accel_n == CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ) {
                                inerr("Accelerometer buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->accel[radio_telem->empty->accel_n++] = accel_blk;

                            sensor_downsamples[SENSOR_ACCEL].sum_buffer[0] = 0;
                            sensor_downsamples[SENSOR_ACCEL].sum_buffer[1] = 0;
                            sensor_downsamples[SENSOR_ACCEL].sum_buffer[2] = 0;
                        }

                        break;
                    }
                    case SENSOR_GYRO: {
                        struct sensor_gyro gyro = ((struct sensor_gyro *)data_buf)[j];

                        sensor_downsamples[SENSOR_GYRO].sum_buffer[0] += (int32_t)tenth_degree(gyro.x);
                        sensor_downsamples[SENSOR_GYRO].sum_buffer[1] += (int32_t)tenth_degree(gyro.y);
                        sensor_downsamples[SENSOR_GYRO].sum_buffer[2] += (int32_t)tenth_degree(gyro.z);

                        /* we check whether the sum buffer is full, if it is we downsample and pass the output to the radio buffer */
                        if(sensor_downsamples[SENSOR_GYRO].reads > 0 && sensor_downsamples[SENSOR_GYRO].reads % sensor_downsamples[SENSOR_GYRO].rate == 0) {
                            float x = sensor_downsamples[SENSOR_GYRO].sum_buffer[0] / sensor_downsamples[SENSOR_GYRO].rate;
                            float y = sensor_downsamples[SENSOR_GYRO].sum_buffer[1] / sensor_downsamples[SENSOR_GYRO].rate;
                            float z = sensor_downsamples[SENSOR_GYRO].sum_buffer[2] / sensor_downsamples[SENSOR_GYRO].rate;

                            struct ang_vel_blk_t ang_vel_blk = {
                                .time_offset = us_to_ms(gyro.timestamp),
                                .x = x,
                                .y = y,
                                .z = z,
                            };

                            /* send the packet to the radio buffer */
                            if(radio_telem->empty->ang_vel_n == CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ) {
                                inerr("Gyroscope buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->ang_vel[radio_telem->empty->ang_vel_n++] = ang_vel_blk;

                            sensor_downsamples[SENSOR_GYRO].sum_buffer[0] = 0;
                            sensor_downsamples[SENSOR_GYRO].sum_buffer[1] = 0;
                            sensor_downsamples[SENSOR_GYRO].sum_buffer[2] = 0;
                        }

                        break;
                    }
                    case SENSOR_MAG: {
                        struct sensor_mag mag = ((struct sensor_mag *)data_buf)[j];

                        sensor_downsamples[SENSOR_MAG].sum_buffer[0] += (int32_t)tenth_microtesla(mag.x);
                        sensor_downsamples[SENSOR_MAG].sum_buffer[1] += (int32_t)tenth_microtesla(mag.y);
                        sensor_downsamples[SENSOR_MAG].sum_buffer[2] += (int32_t)tenth_microtesla(mag.z);

                        /* we check whether the sum buffer is full, if it is we downsample and pass the output to the radio buffer */
                        if(sensor_downsamples[SENSOR_MAG].reads > 0 && sensor_downsamples[SENSOR_MAG].reads % sensor_downsamples[SENSOR_MAG].rate == 0) {
                            float x = sensor_downsamples[SENSOR_MAG].sum_buffer[0] / sensor_downsamples[SENSOR_MAG].rate;
                            float y = sensor_downsamples[SENSOR_MAG].sum_buffer[1] / sensor_downsamples[SENSOR_MAG].rate;
                            float z = sensor_downsamples[SENSOR_MAG].sum_buffer[2] / sensor_downsamples[SENSOR_MAG].rate;

                            struct mag_blk_t mag_blk = {
                                .time_offset = us_to_ms(mag.timestamp),
                                .x = x,
                                .y = y,
                                .z = z,
                            };

                            /* send the packet to the radio buffer */
                            if(radio_telem->empty->mag_n == CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ) {
                                inerr("Magnetometer buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->mag[radio_telem->empty->mag_n++] = mag_blk;

                            sensor_downsamples[SENSOR_MAG].sum_buffer[0] = 0;
                            sensor_downsamples[SENSOR_MAG].sum_buffer[1] = 0;
                            sensor_downsamples[SENSOR_MAG].sum_buffer[2] = 0;
                        }

                        break;
                    }
                    case SENSOR_GNSS: {
                        struct sensor_gnss gnss = ((struct sensor_gnss *)data_buf)[j];

                        sensor_downsamples[SENSOR_GNSS].sum_buffer[0] += (int64_t)point_one_microdegrees(gnss.latitude);
                        sensor_downsamples[SENSOR_GNSS].sum_buffer[1] += (int64_t)point_one_microdegrees(gnss.longitude);

                        /* we check whether the sum buffer is full, if it is we downsample and pass the output to the radio buffer */
                        if(sensor_downsamples[SENSOR_GNSS].reads > 0 && sensor_downsamples[SENSOR_GNSS].reads % sensor_downsamples[SENSOR_GNSS].rate == 0) {
                            float lat = sensor_downsamples[SENSOR_GNSS].sum_buffer[0] / sensor_downsamples[SENSOR_GNSS].rate;
                            float lon = sensor_downsamples[SENSOR_GNSS].sum_buffer[1] / sensor_downsamples[SENSOR_GNSS].rate;

                            struct coord_blk_t coord_blk = {
                                .time_offset = us_to_ms(gnss.timestamp),
                                .latitude = lat,
                                .longitude = lon,
                            };

                            /* send the packet to the radio buffer */
                            if(radio_telem->empty->gnss_n == CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ) {
                                inerr("GNSS buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->gnss[radio_telem->empty->gnss_n++] = coord_blk;

                            sensor_downsamples[SENSOR_GNSS].sum_buffer[0] = 0;
                            sensor_downsamples[SENSOR_GNSS].sum_buffer[1] = 0;
                        }

                        break;
                    }
                    case SENSOR_ALT: {
                        struct fusion_altitude alt = ((struct fusion_altitude *)data_buf)[j];

                        sensor_downsamples[SENSOR_ALT].sum_buffer[0] += (int64_t)millimeters(alt.altitude);

                        /* we check whether the sum buffer is full, if it is we downsample and pass the output to the radio buffer */
                        if(sensor_downsamples[SENSOR_ALT].reads > 0 && sensor_downsamples[SENSOR_ALT].reads % sensor_downsamples[SENSOR_ALT].rate == 0) {
                            float altitude = sensor_downsamples[SENSOR_ALT].sum_buffer[0] / sensor_downsamples[SENSOR_ALT].rate;

                            struct alt_blk_t alt_blk = {
                                .time_offset = us_to_ms(alt.timestamp),
                                .altitude = altitude,
                            };

                            /* send the packet to the radio buffer */
                            if(radio_telem->empty->alt_n == CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ) {
                                inerr("Altitude buffer full, dropping data\n");
                                break;
                            }

                            radio_telem->empty->alt[radio_telem->empty->alt_n++] = alt_blk;

                            sensor_downsamples[SENSOR_ALT].sum_buffer[0] = 0;
                        }
                        break;
                    }
                }
            }

            pthread_mutex_unlock(&radio_telem->empty_mux);
        }
    }
    publish_error(PROC_ID_COLLECTION, ERROR_PROCESS_DEAD);
    pthread_exit(0);
}