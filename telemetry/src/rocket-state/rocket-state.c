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

/* A struct with the values of the eeprom's contents */

struct eeprom_contents {
  int flight_state;
};

/* Represents the different liens that exist in the eeprom contents */

enum eeprom_line_e {
  EEPROM_FLIGHT_STATE,
};

/* The maximum size of a line in the eeprom */
#define MAX_EEPROM_LINE_LEN 50

/* Matches eeprom lines to their format codes, for reading or writing */

static const char *EEPROM_CONTENTS_FMT[] = {
  [EEPROM_FLIGHT_STATE] = "Flight state: %d\n",
};

/**
 * Read the contents of the EEPROM
 * 
 * @param contents The read contents of the eeprom
 * @return 0 on a successful read, or a negative error code on failure
 */
static int eeprom_read(struct eeprom_contents* contents) {
  int err;

  int fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_RDONLY);
  if (fd < 0) {
    return fd;
  }
  char buffer[MAX_EEPROM_LINE_LEN];

  int err = read(fd, buffer, sizeof(buffer));
  if (err < 0) {
    goto close;
  }
  int err = sscanf(buffer, EEPROM_CONTENTS_FMT[EEPROM_FLIGHT_STATE], &contents->flight_state);
  if (err < 0) {
    goto close;
  }

  close:
    close(fd);
    return err; 
}

/**
 * Write the contents of the EEPROM
 * 
 * @param contents The data to write to the eerpom
 * @return 0 on a successful write, or a negative error code on failure
 */
static int eeprom_write(struct eeprom_contents* contents) {
  int err;

  int fd = open(CONFIG_INSPACE_TELEMETRY_EEPROM, O_WRONLY);
  if (fd < 0) {
    return fd;
  }
  char buffer[MAX_EEPROM_LINE_LEN];

  int err = snprintf(buffer, sizeof(buffer), EEPROM_CONTENTS_FMT[EEPROM_FLIGHT_STATE], contents->flight_state);
  if (err < 0) {
    goto close;
  }
  err = write(fd, buffer, sizeof(buffer));
  if (err < 0) {
    goto close;
  }

  close:
    close(fd);
    return err;
}

/*
 * Gets the current flight state stored in NV storage.
 */
static enum flight_state_e get_flight_state(void) {
  struct eeprom_contents contents;
  int err = eeprom_read(&contents);
  if (err < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error reading eeprom, going to idle flightstate: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    return STATE_IDLE;
  }
  return (enum flight_state_e)contents.flight_state;
}


/* Initialize the rocket state monitor
 * @param state The rocket state to initialize
 * @param flight_state The flight state that the rocket is currently in.
 * @return 0 on success, error code on failure
 */
int state_init(rocket_state_t *state) {
  atomic_store(&state->state, get_flight_state());
  return 0;
}

/*
 * Set the flight state in NV storage and state object (write-through)
 * @param state The rocket state to modify
 * @param flight_state The rocket's new flight state to set.
 * @return 0 on success, error code on failure
 */
int state_set_flightstate(rocket_state_t *state,
                          enum flight_state_e flight_state) {
  atomic_store(&state->state, flight_state);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Flight state changed to %s\n", FLIGHT_STATES[flight_state]);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  struct eeprom_contents contents;
  contents.flight_state = flight_state;
  return eeprom_write(&contents);
}

/*
 * Get the flight state atomically.
 * @param state The rocket state to get flight state from.
 * @param flight_state A pointer in which to hold the flight state.
 * @return 0 on success, error code on failure
 */
int state_get_flightstate(rocket_state_t *state,
                          enum flight_state_e *flight_state) {
  *flight_state = atomic_load(&state->state);
  return 0;
}
