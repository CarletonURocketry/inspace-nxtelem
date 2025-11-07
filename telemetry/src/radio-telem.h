#ifndef _RADIO_TELEM_H_
#define _RADIO_TELEM_H_

#include "packets/packets.h"
#include <pthread.h>
#include "fusion/fusion.h"

typedef struct{
    struct sensor_gnss gnss[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int gnss_n;
    struct fusion_altitude alt[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int alt_n;
    struct sensor_mag mag[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int mag_n;
    struct sensor_accel accel[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int accel_n;
    struct sensor_gyro gyro[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int gyro_n;
} radio_raw_data;

typedef struct {
    radio_raw_data *full;
    pthread_mutex_t full_mux;
    radio_raw_data *empty;
    pthread_mutex_t empty_mux;
} radio_telem_t;

#endif
