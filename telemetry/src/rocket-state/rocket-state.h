#ifndef _ROCKET_STATE_H_
#define _ROCKET_STATE_H_

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#if defined(CONFIG_LPWAN_RN2XX3)
#include <nuttx/wireless/lpwan/rn2xx3.h>
#else
enum rn2xx3_cr_e { SIM_NO_CODE_RATE = 0x0 };
#endif

/* Enum representing the current flight state. */

enum flight_state_e {
    STATE_IDLE,     /* The rocket is idle on the pad. */
    STATE_AIRBORNE, /* The rocket is in the air. */
    STATE_LANDED,   /* The rocket is landed. */
};

/* Enum representing the substate of the current flight state, if applicable */

enum flight_substate_e {
    SUBSTATE_UNKNOWN, /* The flight substate is unknown. */
    SUBSTATE_ASCENT,  /* The rocket is ascending. */
    SUBSTATE_DESCENT, /* The rocket is descending. */
};

/* State information about the rocket */

typedef struct {
    atomic_int state;    /* Flight state of the rocket. */
    atomic_int substate; /* Flight substate of the rocket. */
} rocket_state_t;

/* A struct that defines the configuration parameters for the radio */

struct radio_options {
    uint64_t sync;       /* Sync word */
    uint32_t freq;       /* Frequency, Hz */
    int32_t txpwr;       /* Transmit power, dBm */
    uint32_t bw;         /* Bandwidth, kHz */
    uint16_t preamble;   /* Preamble length */
    uint8_t spread;      /* Spread factor */
    enum rn2xx3_cr_e cr; /* Coding rate */
    bool crc;            /* CRC enabled */
    bool iqi;            /* IQI enabled */
};

/* A struct that defines the configuration parameters for the flight computer */

struct config_options {
    struct radio_options radio;
};

int state_init(rocket_state_t *state);

int state_set_flightstate(rocket_state_t *state, enum flight_state_e flight_state);
int state_get_flightstate(rocket_state_t *state, enum flight_state_e *flight_state);

int state_set_flightsubstate(rocket_state_t *state, enum flight_substate_e flight_substate);
int state_get_flightsubstate(rocket_state_t *state, enum flight_substate_e *flight_substate);

int config_get(struct config_options *config);
int config_set(struct config_options *config);

#endif // _ROCKET_STATE_H_
