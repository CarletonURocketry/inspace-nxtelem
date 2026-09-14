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
    STATE_IDLE = 0,     /* The rocket is idle on the pad. */
    STATE_AIRBORNE = 1, /* The rocket is in the air. */
    STATE_LANDED = 2,   /* The rocket is landed. */
};

/* Enum representing the substate of the current flight state, if applicable */

enum flight_substate_e {
    SUBSTATE_UNKNOWN = 0, /* The flight substate is unknown. */
    SUBSTATE_ASCENT = 1,  /* The rocket is ascending. */
    SUBSTATE_DESCENT = 2, /* The rocket is descending. */
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

/* A struct that defines the flight state */

struct nv_flightstate {
    uint8_t flight_state;    /* The flight state of the rocket, of type enum flight_state_e (a uint8_t so only one byte
                                stored) */
    uint8_t flight_substate; /* The flight substate of the rocket, of type enum flight_substate_e (a uint8_t so only one
                                byte stored)*/
    uint8_t crc;             /* A 8 bit cyclic redundancy check to make sure data is valid before being used */
} __attribute__((packed, aligned(1)));

/* A struct that defines how the non-volatile storage medium will store information */

struct nv_storage {
    struct config_options config; /* Flight computer configuration */
    struct nv_flightstate fstate; /* The flight state with CRC */
};

int state_init(rocket_state_t *state);

int state_set_flightstate(rocket_state_t *state, enum flight_state_e flight_state);
int state_get_flightstate(rocket_state_t *state, enum flight_state_e *flight_state);

int state_set_flightsubstate(rocket_state_t *state, enum flight_substate_e flight_substate);
int state_get_flightsubstate(rocket_state_t *state, enum flight_substate_e *flight_substate);

int flightstate_read(struct nv_flightstate *contents);
int flightstate_write(struct nv_flightstate *contents);

int save_state(rocket_state_t *state);

int config_get(struct config_options *config);
int config_set(struct config_options *config);

#endif // _ROCKET_STATE_H_
