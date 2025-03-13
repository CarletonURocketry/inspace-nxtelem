#include <pthread.h>
#include <time.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "../rocket-state/rocket-state.h"
#include "../fusion/fusion.h"
#include "../sensors/sensors.h"
#include "collection.h"

/* Measurement interval in nanoseconds */

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
#define INTERVAL (1e9 / CONFIG_INSPACE_TELEMETRY_RATE)
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

static uint32_t ms_since(struct timespec *start);


#define UORB_BUFFER_SIZE 5

/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {

  int err;
  enum flight_state_e flight_state;
  struct timespec start_time;
  struct collection_args *unpacked_args = (struct collection_args *)(arg);
  rocket_state_t *state = unpacked_args->state;

  packet_buffer_t *logging_buffer = unpacked_args->logging_buffer;
  packet_node_t *logging_packet = packet_buffer_get_empty(logging_buffer);
  uint8_t logging_seq_num = 0;
  packet_buffer_t *transmit_buffer = unpacked_args->transmit_buffer;
  packet_node_t *transmit_packet = packet_buffer_get_empty(transmit_buffer);
  uint8_t transmit_seq_num = 0;

  struct uorb_inputs sensors;
  union uorb_data sensor_data[UORB_BUFFER_SIZE];

  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.accel, orb_get_meta("sensor_accel"));
  setup_sensor(&sensors.baro, orb_get_meta("sensor_baro"));
  setup_sensor(&sensors.mag, orb_get_meta("sensor_mag"));
  setup_sensor(&sensors.gyro, org_get_meta("sensor_gyro"));
  setup_sensor(&sensors.gnss, orb_get_meta("sensor_gnss"));

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
  struct timespec period_start;
  struct timespec next_interval;
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Collection thread started.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  /* Get startup time */

  clock_gettime(CLOCK_MONOTONIC, &start_time);

  /* Measure data forever */

  for (;;) {

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
    /* Get the start of this measurement period */

    clock_gettime(CLOCK_MONOTONIC, &period_start);
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

    /* Get the current flight state */

    err = state_get_flightstate(state, &flight_state); // TODO: error handling
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    if (err) {
      fprintf(stderr, "Could not get flight state: %d\n", err);
    }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

    /* Wait for new data */
    poll_sensors(&sensors);
    ssize_t data_len = get_sensor_data(&sensors.accel, sensor_data, sizeof(struct sensor_accel) * UORB_BUFFER_SIZE);
    if (data_len > 0) {
      struct sensor_accel *accel_data = (struct sensor_accel *)sensor_data;
      for (int i = 0; i < (data_len / sizeof(struct sensor_accel)); i++) {
        uint8_t *block = alloc_block(transmit_buffer, &transmit_packet, DATA_ACCEL_ABS, transmit_seq_num, us_to_ms(accel_data[i].timestamp));
        accel_blk_init((struct accel_blk_t*)block_body(block), accel_data[i].x, accel_data[i].y, accel_data[i].z);

        block = alloc_block(logging_buffer, &logging_packet, DATA_ACCEL_ABS, logging_seq_num, us_to_ms(accel_data[i].timestamp));
        accel_blk_init((struct accel_blk_t*)block_body(block), accel_data[i].x, accel_data[i].y, accel_data[i].z);
      }
    }
    data_len = get_sensor_data(&sensors.accel, sensor_data, sizeof(struct sensor_baro) * UORB_BUFFER_SIZE);
    if (data_len > 0) {
      struct sensor_baro *baro_data = (struct sensor_baro *)sensor_data;
      for (int i = 0; i < (data_len / sizeof(struct sensor_baro)); i++) {
        uint8_t *block = alloc_block(transmit_buffer, &transmit_packet, DATA_PRESSURE, transmit_seq_num, us_to_ms(baro_data[i].timestamp));
        pres_blk_init(block, baro_data[i].pressure);

        block = alloc_block(logging_buffer, &logging_packet, DATA_ACCEL_ABS, logging_seq_num, us_to_ms(baro_data[i].timestamp));
        pres_blk_init(block, baro_data[i].pressure);
      }
    }

    /* Decide whether to move to lift-off state. TODO: real logic */

    switch (flight_state) {
    case STATE_IDLE: {
      // Perform lift-off detection operations
      // If lift-off:
      // state_set_flightstate(state, STATE_AIRBORNE);
    } break;
    case STATE_AIRBORNE: {
      // Perform landing detection operations
      // If landed
      // state_set_flightstate(state, STATE_LANDED);
    } break;
    case STATE_LANDED:
      break; /* Do nothing, just continue collecting */
    }

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
    /* Once operations are complete, wait until the next measuring period */

    next_interval.tv_nsec = (INTERVAL - (ms_since(&period_start) * 1e6));

    clock_nanosleep(CLOCK_MONOTONIC, 0, &next_interval, NULL);
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */
  }

  pthread_exit(0);
}

/*
 * Get the time difference from start until now in milliseconds.
 * @param start The start time.
 * @return The time elapsed since start in milliseconds.
 */
static uint32_t ms_since(struct timespec *start) {

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now); // TODO: this can fail

  struct timespec diff = {
      .tv_sec = (now.tv_sec - start->tv_sec),
      .tv_nsec = now.tv_nsec - start->tv_nsec,
  };

  if (diff.tv_nsec < 0) {
    diff.tv_nsec += 1e9;
    diff.tv_nsec--;
  }

  return diff.tv_sec * 1000 + (diff.tv_nsec / 1e6);
}

/**
 * Allocates a block in the current node or a block in a new node if the current node is full
 * 
 * @param buffer The buffer to get a new, empty node from and place full nodes in
 * @param node The current node being worked on
 * @param type The type of block being allocated
 * @param seq_num The sequence number of the packet
 * @param mission_time The time of the measurement, if this block type has one
 * @return The location to write the requested type of block
 */
static uint8_t *alloc_block(packet_buffer_t *buffer, packet_node_t **node, enum block_type_e type, uint32_t *seq_num, uint32_t mission_time) {
  // The last byte of the packet will be where the block is allocated, but we need to know where it will end to update (*node)->end
  uint8_t *next_block = pkt_create_blk((*node)->packet, (*node)->end, type, mission_time);
  // Can't add to this packet, it's full or we can just assume its done being asssembled
  if (next_block == NULL) {
    packet_buffer_put_full(buffer, node);
    (*node) = packet_buffer_get_empty(buffer);
    (*node)->end = init_pkt((*node)->packet, ++(*seq_num), mission_time);
    next_block = pkt_create_blk((*node)->packet, (*node)->end, type, mission_time);
  }
  uint8_t *write_to = (*node)->end;
  (*node)->end = next_block;

  return write_to;
}
