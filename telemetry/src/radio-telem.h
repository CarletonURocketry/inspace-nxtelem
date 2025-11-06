#ifndef _RADIO_TELEM_H_
#define _RADIO_TELEM_H_

#include "packets/packets.h"
#include <pthread.h>

typedef struct{
    struct coord_blk_t gnss[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int gnss_n;
    struct alt_blk_t alt[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int alt_n;
    struct mag_blk_t mag[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int mag_n;
    struct accel_blk_t accel[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int accel_n;
    struct ang_vel_blk_t ang_vel[CONFIG_INSPACE_TELEMETRY_TARGET_TRANSMIT_FREQ];
    int ang_vel_n;
} radio_raw_data;

typedef struct {
    radio_raw_data *full;
    pthread_mutex_t full_mux;
    radio_raw_data *empty;
    pthread_mutex_t empty_mux;
} radio_telem_t;

#endif
