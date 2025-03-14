#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "logging.h"
#include "../fusion/fusion.h"
#include "../sensors/sensors.h"
#include "../packets/packets.h"

/* The maximum size of a log file's file name. */

#define MAX_FILENAME 32

/* The format for flight log file names */

#define FLIGHT_FNAME_FMT CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "/flog%d.bin"

/* The format for extraction log file names */

#define EXTR_FNAME_FMT CONFIG_INSPACE_TELEMETRY_LANDED_FS "/elog%d.bin"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

static size_t log_packet(FILE *storage, uint8_t *packet, size_t packet_size);
static uint8_t *create_block(FILE *storage, uint8_t *packet, uint8_t *block, uint32_t *seq_num, enum block_type_e type, uint32_t mission_time);
static uint32_t us_to_ms(uint64_t us) {
  return (uint32_t)(us / 1000);
}

/*
 * Logging thread which runs to log data to the SD card.
 */
void *logging_main(void *arg) {

  int err;
  enum flight_state_e flight_state;
  struct logging_args *unpacked_args = (struct logging_args *)(arg);
  rocket_state_t *state = unpacked_args->state;
  packet_buffer_t *buffer = unpacked_args->buffer;
  uint32_t seq_num = 0;

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
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    if (err) {
      fprintf(stderr, "Error getting flight state: %d\n", err);
    }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

    switch (flight_state) {
    case STATE_IDLE:
      /* Purposeful fall-through */
    case STATE_AIRBORNE: {
      packet_node_t *next_packet = packet_buffer_get_full(buffer);
      ((pkt_hdr_t *)next_packet->packet)->packet_num = seq_num++;
      log_packet(storage, next_packet->packet, next_packet->end - next_packet->packet);
      packet_buffer_put_empty(buffer, next_packet);

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
        nbytes = sizeof(buf) * fwrite(buf, nbytes, 1, log);
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

      err = state_set_flightstate(state, STATE_IDLE); // TODO: error handling
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

static size_t log_packet(FILE *storage, uint8_t *packet, size_t packet_size) {
  size_t written = fwrite(packet, 1, packet_size, storage);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Logged %lu bytes\n", written);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  if (written == 0) {
    // TODO: Handle error (might happen if file got too large, start
    // another file)
  }
  return written;
}
