#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>

#include "../syslogging.h"
#include "rocket-state.h"

/* The polynomial to use in the 8 bit crc */

#define NV_STORAGE_CRC_POLYNOMIAL 0x31

/* The initial value when calculating the 8 bit crc */

#define NV_STORAGE_CRC_INITIAL 0xFF

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

#if defined(CONFIG_INSPACE_SYSLOG_OUTPUT)
static const char *FLIGHT_STATES[] = {
    [STATE_IDLE] = "STATE_IDLE",
    [STATE_LANDED] = "STATE_LANDED",
    [STATE_AIRBORNE] = "STATE_AIRBORNE",
};
#endif

#if defined(CONFIG_INSPACE_SYSLOG_OUTPUT)
static const char *FLIGHT_SUBSTATES[] = {
    [SUBSTATE_UNKNOWN] = "SUBSTATE_UNKNOWN",
    [SUBSTATE_ASCENT] = "SUBSTATE_ASCENT",
    [SUBSTATE_DESCENT] = "SUBSTATE_DESCENT",
};
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/* Calculates an 8 bit cyclic redundancy check for the provided data
 * @param buff A pointer to the data to have its CRC calculated
 * @param n_bytes The length of the data in bytes
 * @return uint8_t The calculated CRC
 */
uint8_t calculate_crc8_bitwise(const uint8_t *buf, size_t nbytes) {
    uint8_t crc = NV_STORAGE_CRC_INITIAL;
    for (size_t byte = 0; byte < nbytes; byte++) {
        crc ^= buf[byte];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                // Discard the highest bit (implicit XOR), then divide by the polynomial
                crc <<= 1;
                crc ^= NV_STORAGE_CRC_POLYNOMIAL;
            } else {
                // Continue until the highest bit is set
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* Get the contents of non-volatile storage and check the CRC
 * @param Where to put the loaded contents of the NV storage
 * @return 0 on success, negative error code on failure
 */
static int flightstate_read(struct nv_flightstate *contents) {
    int fd;
    ssize_t err;

    fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_RDONLY);
    if (fd < 0) {
        inerr("Error opening nv storage: %d\n", errno);
        return fd;
    }

    /* Seek to location of flight state */

    if (lseek(fd, offsetof(struct nv_storage, fstate), SEEK_SET) < 0) {
        inerr("Couldn't seek to flight state: %d\n", errno);
        err = errno;
        goto early_ret;
    }

    /* Read */

    err = read(fd, contents, sizeof(*contents));
    if (err < 0) {
        inerr("Error reading nv storage: %d\n", errno);
        return errno;
    } else if (err != sizeof(*contents)) {
        inerr("Didn't read the correct number of bytes from nv storage: %d\n", err);
        err = EIO;
    } else if (calculate_crc8_bitwise((uint8_t *)contents, sizeof(*contents)) != 0) {
        inerr("CRC check failed on nv storage data\n");
        err = EIO;
    } else {
        err = 0;
    }

early_ret:
    close(fd);
    return err;
}

/* Set the CRC and write the flight state to NV storage
 * @param contents The contents to write to NV storage
 * @return 0 on success, negative error code on failure
 */
static int flightstate_write(struct nv_flightstate *contents) {
    int fd;
    ssize_t err;

    fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_WRONLY | O_CREAT);
    if (fd < 0) {
        inerr("Error opening nv storage: %d\n", errno);
        return fd;
    }

    /* Seek to location of flight state */

    if (lseek(fd, offsetof(struct nv_storage, fstate), SEEK_SET) < 0) {
        inerr("Couldn't seek to flight state: %d\n", errno);
        err = errno;
        goto early_ret;
    }

    contents->crc = calculate_crc8_bitwise((uint8_t *)contents, sizeof(*contents) - 1);
    err = write(fd, contents, sizeof(*contents));
    if (err < 0) {
        inerr("Error writing nv storage: %d\n", errno);
        err = errno;
    } else if (err != sizeof(*contents)) {
        inerr("Didn't write the correct number of bytes to nv storage: %d\n", err);
        err = EIO;
    } else {
        err = 0;
    }

early_ret:
    close(fd);
    return err;
}

/* Initialize the rocket state monitor using NV storage, or sensible defaults if NV storage is unavailable
 * @param state The rocket state to initialize
 * @return 0 on success, or an error code on failure reading from NV storage
 */
int state_init(rocket_state_t *state) {
    struct nv_flightstate contents;
    int err;

    err = flightstate_read(&contents);
    if (err) {
        inerr("Couldn't read from nv storage, setting airborne flightstate: %d\n", err);
        atomic_store(&state->state, STATE_AIRBORNE);
        atomic_store(&state->substate, SUBSTATE_UNKNOWN);

        contents.flight_state = STATE_AIRBORNE;
        contents.flight_substate = SUBSTATE_UNKNOWN;
        int write_err = flightstate_write(&contents);
        if (write_err) {
            inerr("Couldn't write new flightstate to nv storage after read failure: %d\n", write_err);
        }
    } else {
        atomic_store(&state->state, contents.flight_state);
        atomic_store(&state->substate, contents.flight_substate);
    }

    return err;
}

/* Save the flight state in NV storage
 * @param state The state to save in NV storage
 * @return 0 on success, or an error code if writing to NV storage failed
 */
static int save_state(rocket_state_t *state) {
    struct nv_flightstate contents;
    int err;

    err = flightstate_read(&contents);
    if (err) {
        inerr("Couldn't load old EEPROM contents to overwrite state.\n");
    }

    contents.flight_state = atomic_load(&state->state);
    contents.flight_substate = atomic_load(&state->substate);

    err = flightstate_write(&contents);
    if (err) {
        inerr("Couldn't write flight state to nv storage, continuing anyways\n");
    }
    return err;
}

/* Gets the current configuration from the EEPROM.
 * @param config Where to store the read config
 * @return 0 on success, errno error code on failure
 */
int config_get(struct config_options *config) {
    int fd;
    ssize_t err;

    fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_RDONLY);
    if (fd < 0) {
        inerr("Error opening nv storage: %d\n", errno);
        return errno;
    }

    err = read(fd, config, sizeof(*config));
    if (err < 0) {
        inerr("Error reading configuration from EEPROM: %d\n", errno);
        err = errno;
    }

    close(fd);
    return 0;
}

/* Sets the configuration in the EEPROM.
 * @param config The configuration to store in EEPROM
 * @return 0 on success, errno error code on failure
 */
int config_set(struct config_options *config) {
    int fd;
    ssize_t err;

    fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_WRONLY | O_CREAT);
    if (fd < 0) {
        inerr("Error opening nv storage: %d\n", errno);
        return errno;
    }

    err = write(fd, config, sizeof(*config));
    if (err < 0) {
        inerr("Error writing configuration to EEPROM: %d\n", errno);
        err = errno;
    }

    close(fd);
    return 0;
}

/* Set the flight state in NV storage and state object (write-through)
 * @param state The rocket state to modify
 * @param flight_state The rocket's new flight state to set
 * @return 0 on success, or an error code if writing to NV storage failed
 */
int state_set_flightstate(rocket_state_t *state, enum flight_state_e flight_state) {
    atomic_store(&state->state, flight_state);
    ininfo("Flight state changed to %s\n", FLIGHT_STATES[flight_state]);
    int err = save_state(state);
    return err;
}

/* Get the current flight state
 * @param state The rocket state to get flight state from
 * @param flight_state A pointer in which to hold the flight state
 * @return 0 on success, error code on failure
 */
int state_get_flightstate(rocket_state_t *state, enum flight_state_e *flight_state) {
    *flight_state = atomic_load(&state->state);
    return 0;
}

/* Set the current flight substate
 * @param state The rocket state to modify
 * @param flight_substate The new substate to set
 * @return 0 on success, or an error code if writing to NV storage failed
 */
int state_set_flightsubstate(rocket_state_t *state, enum flight_substate_e flight_substate) {
    atomic_store(&state->substate, flight_substate);
    ininfo("Flight substate changed to %s\n", FLIGHT_SUBSTATES[flight_substate]);
    int err = save_state(state);
    return err;
}

/* Get the current flight substate
 * @param state The rocket state to get the substate from
 * @param flight_substate A pointer in which to hold the substate
 * @return 0 on success, error code on failure
 */
int state_get_flightsubstate(rocket_state_t *state, enum flight_substate_e *flight_substate) {
    *flight_substate = atomic_load(&state->substate);
    return 0;
}
