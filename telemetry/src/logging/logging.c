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
  rocket_state_t *state = ((rocket_state_t *)(arg));

  char flight_filename[sizeof(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS) +
                       MAX_FILENAME];
  char land_filename[sizeof(CONFIG_INSPACE_TELEMETRY_LANDED_FS) + MAX_FILENAME];

  /* Use packets for the logging format to stay consistent */

  uint8_t packet[PACKET_MAX_SIZE];     /* Array of bytes for packet */
  uint8_t *current_block = packet;     /* Location in packet buffer */
  uint8_t *next_block;                 /* Next block in packet buffer */
  uint32_t seq_num = 0;                /* Packet numbering */

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

  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.accel, ORB_ID(fusion_accel));
  setup_sensor(&sensors.baro, ORB_ID(fusion_baro));
  struct sensor_accel accel_data[ACCEL_FUSION_BUFFER_SIZE];
  struct sensor_baro baro_data[BARO_FUSION_BUFFER_SIZE];

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

      /* Wait for new data */
      poll_sensors(&sensors);
      // Get data together, so we can block on transmit and not lose the data we're currently using
      // Could also ask for the minimum of the free space in the size of the buffer to save effort
      ssize_t accel_len = get_sensor_data(&sensors.accel, accel_data, sizeof(accel_data));
      ssize_t baro_len = get_sensor_data(&sensors.baro, baro_data, sizeof(baro_data));

      if (accel_len > 0) {
        for (int i = 0; i < (accel_len / sizeof(struct sensor_accel)); i++) {
          next_block = create_block(storage, packet, current_block, &seq_num, DATA_ACCEL_ABS, us_to_ms(accel_data[i].timestamp));
          accel_blk_init((struct accel_blk_t*)block_body(current_block), accel_data[i].x, accel_data[i].y, accel_data[i].z);
          current_block = next_block;
        }
      }
      if (baro_len > 0) {
        for (int i = 0; i < (baro_len / sizeof(struct sensor_baro)); i++) {
          next_block = create_block(storage, packet, current_block, &seq_num, DATA_PRESSURE, us_to_ms(baro_data[i].timestamp));
          pres_blk_init((struct pres_blk_t*)block_body(current_block), baro_data[i].pressure);
          current_block = next_block;

          next_block = create_block(storage, packet, current_block, &seq_num, DATA_TEMP, us_to_ms(baro_data[i].timestamp));
          temp_blk_init((struct temp_blk_t*)block_body(current_block), baro_data[i].temperature);
          current_block = next_block;
        }
      }
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

/**
 * Create a block and set it up, or transmit and make a new packet if a block can't be added
 *
 * @param radio The radio to write the packet to when full
 * @param packet The packet to write to
 * @param block The block to create and initialize the header and time for
 * @param seq_num The current sequence number, incremented if a packet is transmitted
 * @param type The type of block to add
 * @param mission_time If the type of block requires a time, the time to use
 * @return The start of the next block
 */
static uint8_t *create_block(FILE *storage, uint8_t *packet, uint8_t *block, uint32_t *seq_num, enum block_type_e type, uint32_t mission_time) {
  uint8_t *next_block = pkt_create_blk(packet, block, type, mission_time);
  if (next_block == NULL) {
    if (block != packet) {
      log_packet(storage, packet, block - packet);
      *seq_num += 1;
    }
    // We can delay setting up the header and just let the addition of the first block fail
    block = init_pkt(packet, *seq_num, mission_time);
    next_block = pkt_create_blk(packet, block, type, mission_time);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    if (next_block == NULL) {
      fprintf(stderr, "Error adding block to packet after logging, should not be possible\n");
    }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }
  return next_block;
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
