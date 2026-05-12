#ifndef _RADIO_TELEM_H_
#define _RADIO_TELEM_H_

#include "fusion/fusion.h"
#include "packets/packets.h"
#include <pthread.h>

typedef struct {
    struct sensor_gnss gnss[CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ];
    int gnss_n;
    struct fusion_altitude alt[CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ];
    int alt_n;
    struct sensor_mag mag[CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ];
    int mag_n;
    struct sensor_accel accel[CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ];
    int accel_n;
    struct sensor_gyro gyro[CONFIG_INSPACE_DOWNSAMPLING_TARGET_FREQ];
    int gyro_n;
} radio_raw_data;

typedef struct {
    radio_raw_data *buff;
    pthread_mutex_t buff_mux;
    sem_t swapped;
    radio_raw_data *empty_buff; /* internal to the downsampler thread */
} radio_telem_t;

#endif
