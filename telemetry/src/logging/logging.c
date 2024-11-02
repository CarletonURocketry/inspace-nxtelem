#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "logging.h"

/* The maximum size of a log file's file name. */

#define MAX_FILENAME 32

/* The format for flight log file names */

#define FLIGHT_FNAME_FMT CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "/flog%d.bin"

/* The format for extraction log file names */

#define EXTR_FNAME_FMT CONFIG_INSPACE_TELEMETRY_LANDED_FS "/elog%d.bin"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/*
 * Logging thread which runs to log data to the SD card.
 */
void *logging_main(void *arg) {

  int err;
  size_t written;
  enum flight_state_e flight_state;
  rocket_state_t *state = ((rocket_state_t *)(arg));
  char flight_filename[sizeof(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS) +
                       MAX_FILENAME];
  char land_filename[sizeof(CONFIG_INSPACE_TELEMETRY_LANDED_FS) + MAX_FILENAME];

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Logging thread started.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  /* Generate flight log file name TODO: use sequence numbers */

  snprintf(flight_filename, sizeof(flight_filename), FLIGHT_FNAME_FMT, 0);

  /* Open storage location in append mode */

  FILE *storage = fopen(flight_filename, "ab");
  if (storage == NULL) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    err = errno;
    fprintf(stderr, "Error opening log file '%s': %d\n", flight_filename, err);
#endif                             /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    pthread_exit(err_to_ptr(err)); // TODO: fail more gracefully
  }

  /* Infinite loop to handle states */

  for (;;) {

    err = state_get_flightstate(state, &flight_state);

    switch (flight_state) {
    case STATE_IDLE:
      /* Purposeful fall-through */
    case STATE_AIRBORNE: {

      /* Infinite loop to log data */

      /* Wait for the data to have a change */

      err = state_wait_for_change(state); // TODO: handle error

      /* Log data */

      err = state_read_lock(state); // TODO: handle error

      written = fwrite(&state->data, sizeof(state->data), 1, storage);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      printf("Logged %u bytes\n", written * sizeof(state->data));
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      if (written == 0) {
        // TODO: Handle error (might happen if file got too large, start
        // another file)
      }

      err = state_unlock(state); // TODO: handle error

      /* If we are in the idle state, only write the latest n seconds of data
       */
      if (flight_state == STATE_IDLE) {
        // TODO: check file position
      }
    } break;

    case STATE_LANDED: {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      printf("Copying files to extraction file system.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

      /* Generate log file name for extraction file system */

      snprintf(land_filename, sizeof(land_filename), EXTR_FNAME_FMT,
               0); // TODO: use log seq number

      /* Open extraction log file */

      FILE *log = fopen(land_filename, "wb");
      if (log == NULL) {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr,
                "Couldn't open extraction log file '%s' with error: %d\n",
                land_filename, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
        pthread_exit(err_to_ptr(err));
      }

      /* Roll power-safe log file pointer back to beginning */

      fseek(storage, 0, SEEK_SET); // TODO: handle error

      /* Copy from one to the other using a buffer */

      uint8_t buf[BUFSIZ];
      size_t nbytes = 0;

      while (!feof(storage)) {
        nbytes = sizeof(buf) * fread(buf, sizeof(buf), 1, storage);
        if (nbytes == 0)
          break;
        nbytes = sizeof(buf) * fwrite(buf, nbytes, 1, storage);
      }

      /* Now that logs are copied to FAT partition, move back to the idle state
       * for another go.
       */

      if (fclose(log) != 0) {
        // TODO: handle error
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr,
                "Couldn't close extraction log file '%s' with error: %d\n",
                land_filename, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      state_set_flightstate(state, STATE_IDLE); // TODO: error handling
    }
    }
  }

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (fclose(storage) != 0) {
    err = errno;
    fprintf(stderr, "Failed to close flight logfile handle: %d\n", err);
  }
#else
  fclose(storage);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  pthread_exit(err_to_ptr(err));
}
