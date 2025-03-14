#include <pthread.h>
#include <time.h>
#include <math.h>
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

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

typedef struct {
  packet_buffer_t *logging_buffer;
  packet_node_t *logging_packet;

  packet_buffer_t *transmit_buffer;
  packet_node_t *transmit_packet;
} processing_context_t;

static uint32_t ms_since(struct timespec *start);
static uint8_t *alloc_block(packet_buffer_t *buffer, packet_node_t **node, enum block_type_e type, uint32_t mission_time);
static void baro_handler(void *ctx, uint8_t *data);
static void accel_handler(void *ctx, uint8_t *data);
static void mag_handler(void *ctx, uint8_t *data);
static void gnss_handler(void *ctx, uint8_t *data);
static void gyro_handler(void *ctx, uint8_t *data);

#define us_to_ms(us) (us / 1000)

/* How many measurements to read from sensors at a time (match to size of internal buffers) */
#define ACCEL_READ_SIZE 5
#define BARO_READ_SIZE 5
#define MAG_READ_SIZE 5
#define GNSS_READ_SIZE 5
#define GYRO_READ_SIZE 5

/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {

  int err;
  enum flight_state_e flight_state;
  struct timespec start_time;
  struct collection_args *unpacked_args = (struct collection_args *)(arg);
  rocket_state_t *state = unpacked_args->state;

  processing_context_t context;
  context.logging_buffer = unpacked_args->logging_buffer;
  context.logging_packet = packet_buffer_get_empty(context.logging_buffer);
  context.transmit_buffer = unpacked_args->transmit_buffer;
  context.transmit_packet = packet_buffer_get_empty(context.transmit_buffer);
  if (!context.logging_packet || !context.transmit_packet) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Could not get an initial empty packet for collection\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    pthread_exit(err_to_ptr(-ENOMEM));
  }

  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.accel, orb_get_meta("sensor_accel"));
  setup_sensor(&sensors.baro, orb_get_meta("sensor_baro"));
  setup_sensor(&sensors.mag, orb_get_meta("sensor_mag"));
  setup_sensor(&sensors.gyro, orb_get_meta("sensor_gyro"));
  setup_sensor(&sensors.gnss, orb_get_meta("sensor_gnss"));

  /* Separate buffers use more memory but allow us to process in pieces while still reading only once */
  struct sensor_accel accel_data[ACCEL_READ_SIZE];
  struct sensor_baro baro_data[BARO_READ_SIZE];
  struct sensor_mag mag_data[MAG_READ_SIZE];
  struct sensor_gyro gyro_data[GYRO_READ_SIZE];
  struct sensor_gnss gnss_data[GNSS_READ_SIZE];

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

    /* Read data all at once to avoid as much locking as possible */
    uint8_t *accel_end = get_sensor_data_end(&sensors.accel, accel_data, sizeof(accel_data));
    uint8_t *baro_end = get_sensor_data_end(&sensors.baro, baro_data, sizeof(baro_data));
    uint8_t *mag_end = get_sensor_data_end(&sensors.mag, mag_data, sizeof(mag_data));
    uint8_t *gyro_end = get_sensor_data_end(&sensors.gyro, gyro_data, sizeof(gyro_data));
    uint8_t *gnss_end = get_sensor_data_end(&sensors.gnss, gnss_data, sizeof(gnss_data));

    uint8_t *accel_start = (uint8_t *)accel_data;
    uint8_t *baro_start = (uint8_t *)baro_data;
    uint8_t *mag_start = (uint8_t *)mag_data;
    uint8_t *gyro_start = (uint8_t *)gyro_data;
    uint8_t *gnss_start = (uint8_t *)gnss_data;

    /* Process one piece of data of each type at a time to get a more even mix of things in the packets */
    int processed;
    do {
      processed = 0;
      processed += process_one(accel_handler, &context, &accel_start, accel_end, sizeof(struct sensor_accel));
      processed += process_one(baro_handler, &context, &baro_start, baro_end, sizeof(struct sensor_baro));
      processed += process_one(mag_handler, &context, &mag_start, mag_end, sizeof(struct sensor_mag));
      processed += process_one(gyro_handler, &context, &gyro_start, gyro_end, sizeof(struct sensor_gyro));
      processed += process_one(gnss_handler, &context, &gnss_start, gnss_end, sizeof(struct sensor_gnss));
    } while (processed);

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
 * @param mission_time The time of the measurement, if this block type has one
 * @return The location to write the requested type of block
 */
static uint8_t *alloc_block(packet_buffer_t *buffer, packet_node_t **node, enum block_type_e type, uint32_t mission_time) {
  // The last byte of the packet will be where the block is allocated, but we need to know where it will end to update (*node)->end
  uint8_t *next_block = pkt_create_blk((*node)->packet, (*node)->end, type, mission_time);
  // Can't add to this packet, it's full or we can just assume its done being asssembled
  if (next_block == NULL) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    printf("Completed a packet length %ld\n", (*node)->end - (*node)->packet);
#endif
    packet_buffer_put_full(buffer, *node);
    (*node) = packet_buffer_get_empty(buffer);
    if (*node == NULL) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Couldn't get an empty packet or overwrite a full one - not enough packets in buffer\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      return NULL;
    }

    // Leave seq num up to the logger/transmitter (don't know if or in what order packets get transmitted)
    (*node)->end = init_pkt((*node)->packet, 0, mission_time);
    next_block = pkt_create_blk((*node)->packet, (*node)->end, type, mission_time);
    if (next_block == NULL) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Couldn't add a block to a new packet\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      return NULL;
    }
  }
  uint8_t *write_to = (*node)->end;
  (*node)->end = next_block;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  //printf("Allocated %ld bytes for type %d in packet %p, current size %ld\n", next_block - write_to, type, (*node)->packet, (*node)->end - (*node)->packet);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  return write_to;
}

/**
 * Convert millibar to pascals
 */
static float pascals(float millibar) {
  return millibar * 100;
}

/**
 * Add a pressure block to the packet being assembled
 * 
 * @param buffer A buffer of packet nodes, in case the current packet can't be added to
 * @param node The packet currently being assembled
 * @param baro_data The baro data to add
 */
static void add_pres_blk(packet_buffer_t *buffer, packet_node_t **node, struct sensor_baro *baro_data) {
  uint8_t *block = alloc_block(buffer, node, DATA_PRESSURE, us_to_ms(baro_data->timestamp));
  if (block) {
    pres_blk_init((struct pres_blk_t*)block_body(block), pascals(baro_data->pressure));
  }
}

/**
 * Convert a degrees celsius to millidegrees
 */
static float millidegrees(float celsius) {
  return celsius * 1000;
}

/**
 * Add a temperature block to the packet being assembled
 * 
 * @param buffer A buffer of packet nodes, in case the current packet can't be added to
 * @param node The packet currently being assembled
 * @param baro_data The baro data to add
 */
static void add_temp_blk(packet_buffer_t *buffer, packet_node_t **node, struct sensor_baro *baro_data) {
  uint8_t *block = alloc_block(buffer, node, DATA_TEMP, us_to_ms(baro_data->timestamp));
  if (block) {
    temp_blk_init((struct temp_blk_t*)block_body(block), millidegrees(baro_data->temperature));
  }
}

/**
 * A uorb_data_callback_t function - adds barometric data to the required packets
 * 
 * @param ctx Context information, type processing_context_t
 * @param data Barometric data to add, type struct sensor_baro
 */
static void baro_handler(void *ctx, uint8_t *data) {
  struct sensor_baro *baro_data = (struct sensor_baro*)data;
  processing_context_t *context = (processing_context_t *)ctx;
  add_pres_blk(context->logging_buffer, &context->logging_packet, baro_data);
  add_temp_blk(context->logging_buffer, &context->logging_packet, baro_data);

  add_pres_blk(context->transmit_buffer, &context->transmit_packet, baro_data);
  add_temp_blk(context->transmit_buffer, &context->transmit_packet, baro_data);
}

/**
 * Convert meters per second squared to centimeters per second squared
 */
static float cm_per_sec_squared(float meters_per_sec_squared) {
  return meters_per_sec_squared * 100;
}

/**
 * Add an acceleration block to the packet being assembled
 * 
 * @param buffer A buffer of packet nodes, in case the current packet can't be added to
 * @param node The packet currently being assembled
 * @param accel_data The accel data to add
 */
static void add_accel_blk(packet_buffer_t *buffer, packet_node_t **node, struct sensor_accel *accel_data) {
  uint8_t *block = alloc_block(buffer, node, DATA_ACCEL_ABS, us_to_ms(accel_data->timestamp));
  if (block) {
    accel_blk_init((struct accel_blk_t*)block_body(block), cm_per_sec_squared(accel_data->x), cm_per_sec_squared(accel_data->y), cm_per_sec_squared(accel_data->z));
  }
}

/**
 * A uorb_data_callback_t function - adds acceleration data to the required packets
 *  
 * @param ctx Context information, type processing_context_t
 * @param data Acceleration data to add, type struct sensor_accel
 */
static void accel_handler(void *ctx, uint8_t *data) {
  struct sensor_accel *accel_data = (struct sensor_accel*)data;
  processing_context_t *context = (processing_context_t *)ctx;
  add_accel_blk(context->logging_buffer, &context->logging_packet, accel_data);
  add_accel_blk(context->transmit_buffer, &context->transmit_packet, accel_data);
}

/**
 * A uorb_data_callback_t function - adds magnetometer data to the required packets
 *  
 * @param ctx Context information, type processing_context_t
 * @param data magnetometer data to add, type struct sensor_mag
 */
static void mag_handler(void *ctx, uint8_t *data) {
  // TODO - currently no definition for magnetometer data in packet spec
}


/**
 * Convert a radian to a tenth of a degree
 */
static float tenth_degree(float radian) {
  return radian * 18 / M_PI;
}

/**
 * Add an gyro block to the packet being assembled
 * 
 * @param buffer A buffer of packet nodes, in case the current packet can't be added to
 * @param node The packet currently being assembled
 * @param gyro_data The gyro data to add
 */
static void add_gyro_blk(packet_buffer_t *buffer, packet_node_t **node, struct sensor_gyro *gyro_data) {
  uint8_t *block = alloc_block(buffer, node, DATA_ANGULAR_VEL, us_to_ms(gyro_data->timestamp));
  if (block) {
    ang_vel_blk_init((struct ang_vel_blk_t*)block_body(block), tenth_degree(gyro_data->x), tenth_degree(gyro_data->y), tenth_degree(gyro_data->z));
  }
}

/**
 * A uorb_data_callback_t function - adds gyro data to the required packets
 *  
 * @param ctx Context information, type processing_context_t
 * @param data Acceleration data to add, type struct sensor_gyro
 */
static void gyro_handler(void *ctx, uint8_t *data) {
  struct sensor_gyro *gyro_data = (struct sensor_gyro*)data;
  processing_context_t *context = (processing_context_t *)ctx;
  add_gyro_blk(context->logging_buffer, &context->logging_packet, gyro_data);
  add_gyro_blk(context->transmit_buffer, &context->transmit_packet, gyro_data);
}

/**
 * Convert a degree to a tenth of a microdegree
 */
static float point_one_microdegrees(float degrees) {
  return 1E7f * degrees;
}

/**
 * Add an gnss block to the packet being assembled
 * 
 * @param buffer A buffer of packet nodes, in case the current packet can't be added to
 * @param node The packet currently being assembled
 * @param gnss_data The gnss data to add
 */
static void add_gnss_block(packet_buffer_t *buffer, packet_node_t **node, struct sensor_gnss *gnss_data) {
  uint8_t *block = alloc_block(buffer, node, DATA_LAT_LONG, us_to_ms(gnss_data->timestamp));
  if (block) {
    coord_blk_init((struct coord_blk_t*)block_body(block), point_one_microdegrees(gnss_data->latitude), point_one_microdegrees(gnss_data->longitude));
  }
}

/**
 * Convert meters to millimeters
 */
static float millimeters(float meters) {
  return meters * 1000;
}

/**
 * Add a mean sea level altitude block to the packet being assembled
 * 
 * @param buffer A buffer of packet nodes, in case the current packet can't be added to
 * @param node The packet currently being assembled
 * @param alt_data The altitude data to add
 */
static void add_msl_block(packet_buffer_t *buffer, packet_node_t **node, struct sensor_gnss *alt_data) {
  uint8_t *block = alloc_block(buffer, node, DATA_ALT_SEA, us_to_ms(alt_data->timestamp));
  if (block) {
    alt_blk_init((struct alt_blk_t*)block_body(block), millimeters(alt_data->altitude));
  }
}

static void gnss_handler(void *ctx, uint8_t *data) {
  struct sensor_gnss *gnss_data = (struct sensor_gnss*)data;
  processing_context_t *context = (processing_context_t *)ctx;
  add_gnss_block(context->logging_buffer, &context->logging_packet, gnss_data);
  add_msl_block(context->logging_buffer, &context->logging_packet, gnss_data);

  add_gnss_block(context->transmit_buffer, &context->transmit_packet, gnss_data);
  add_msl_block(context->transmit_buffer, &context->transmit_packet, gnss_data);
}
