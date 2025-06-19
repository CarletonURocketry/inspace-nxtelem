#include <pthread.h>
#include <stdio.h>
#include <fcntl.h>

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "rocket-state.h"

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
static const char *FLIGHT_STATES[] = {
    [STATE_IDLE] = "STATE_IDLE",
    [STATE_LANDED] = "STATE_LANDED",
    [STATE_AIRBORNE] = "STATE_AIRBORNE",
};
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/* A struct that defines how the non-volatile storage medium will store information */

struct nv_storage {
  uint8_t flight_state;     /* The flight state of the rocket, of type enum flight_state_e (a uint8_t so only one byte stored) */
  uint8_t flight_substate;  /* The flight substate of the rocket, of type enum flight_substate_e (a uint8_t so only one byte stored)*/
  int32_t launch_elevation; /* The measured altitude above sea level when at at ground level, in millimeters (signed in case of a constant error) */
  uint8_t crc;              /* A 8 bit cyclic redundancy check to make sure data is valid before being used */
} __attribute__((packed, aligned(1)));

/* The polynomial to use in the 8 bit crc */

#define NV_STORAGE_CRC_POLYNOMIAL 0x31

/* The initial value when calculating the 8 bit crc */

#define NV_STORAGE_CRC_INITIAL 0xFF

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
static int nv_read(struct nv_storage *contents) {
  int fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_RDONLY);
  if (fd < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error opening nv storage: %d\n", fd);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    return fd;
  }
  int err = read(fd, contents, sizeof(struct nv_storage));
  if (err < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error reading nv storage: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  } else if (err != sizeof(struct nv_storage)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Didn't read the correct number of bytes from nv storage: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    err = -1;
  } else if (calculate_crc8_bitwise((uint8_t *)contents, sizeof(struct nv_storage)) != 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "CRC check failed on nv storage data\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    err = -1;
  } else {
    err = 0;
  } 
  close(fd);
  return err;
}

/* Set the CRC and write to NV storage
 * @param contents The contents to write to NV storage
 * @return 0 on success, negative error code on failure
 */
static int nv_write(struct nv_storage *contents) {
  int fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_WRONLY);
  if (fd < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error opening nv storage: %d\n", fd);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    return fd;
  }
  contents->crc = calculate_crc8_bitwise((uint8_t *)contents, sizeof(struct nv_storage) - 1);
  int err = write(fd, contents, sizeof(struct nv_storage));
  if (err < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error writing nv storage: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  } else if (err != sizeof(struct nv_storage)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Didn't write the correct number of bytes to nv storage: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    err = -1;
  } else {
    err = 0;
  }
  close(fd);
  return err;
}

/* Initialize the rocket state monitor using NV storage, or sensible defaults if NV storage is unavailable
 * @param state The rocket state to initialize
 * @return 0 on success, or an error code on failure reading from NV storage
 */
int state_init(rocket_state_t *state) {
  struct nv_storage contents;
  int err = nv_read(&contents);
  if (err < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Couldn't read from nv storage, setting idle flightstate: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    atomic_store(&state->state, STATE_AIRBORNE);
    atomic_store(&state->substate, SUBSTATE_UNKNOWN);
    atomic_store(&state->elevation, DEFAULT_ELEVATION_MM);
  } else {
    atomic_store(&state->state, contents.flight_state);
    atomic_store(&state->substate, contents.flight_substate);
    atomic_store(&state->elevation, contents.launch_elevation);
  }
  return err;
}

/* Save the flight state in NV storage
 * @param state The state to save in NV storage
 * @return 0 on success, or an error code if writing to NV storage failed
 */
static int save_state(rocket_state_t *state) {
  struct nv_storage contents;
  contents.flight_state = atomic_load(&state->state);
  contents.flight_substate = atomic_load(&state->substate);
  contents.launch_elevation = atomic_load(&state->elevation);
  int err = nv_write(&contents);
  if (err < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Couldn't write flight state to nv storage, continuing anyways\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Flight state changed to %s\n", FLIGHT_STATES[flight_state]);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  // Store the value last, so we won't begin using the new state unless we're sure of power-safety
  return err;
}

/* Set the flight state in NV storage and state object (write-through)
 * @param state The rocket state to modify
 * @param flight_state The rocket's new flight state to set
 * @return 0 on success, or an error code if writing to NV storage failed
 */
int state_set_flightstate(rocket_state_t *state,
                          enum flight_state_e flight_state) {
  atomic_store(&state->state, flight_state);
  int err = save_state(state);
  return err;
}

/* Get the current flight state
 * @param state The rocket state to get flight state from
 * @param flight_state A pointer in which to hold the flight state
 * @return 0 on success, error code on failure
 */
int state_get_flightstate(rocket_state_t *state,
                          enum flight_state_e *flight_state) {
  *flight_state = atomic_load(&state->state);
  return 0;
}

/* Set the current flight substate
 * @param state The rocket state to modify
 * @param flight_substate The new substate to set
 * @return 0 on success, or an error code if writing to NV storage failed
 */
int state_set_flightsubstate(rocket_state_t *state,
                             enum flight_substate_e flight_substate) {
  atomic_store(&state->substate, flight_substate);
  int err = save_state(state);
  return err;
}

/* Get the current flight substate
 * @param state The rocket state to get the substate from
 * @param flight_substate A pointer in which to hold the substate
 * @return 0 on success, error code on failure
 */
int state_get_flightsubstate(rocket_state_t *state,
                             enum flight_substate_e *flight_substate) {
  *flight_substate = atomic_load(&state->substate);
  return 0;
}

/* Set the elevation
 * @param state The rocket state to modify
 * @param elevation The new elevation to set
 * @return 0 on success, or an error code if writing to NV storage failed
 */
int state_set_elevation(rocket_state_t *state, int32_t elevation) {
  atomic_store(&state->elevation, elevation);
  int err = save_state(state);
  return err;
}

/* Get the current elevation
 * @param state The rocket state to get the elevation from
 * @param elevation A pointer in which to hold the elevation
 * @return 0 on success, error code on failure
 */
int state_get_elevation(rocket_state_t *state, int32_t *elevation) {
  *elevation = atomic_load(&state->elevation);
  return 0;
}
