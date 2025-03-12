#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "../packets/packets.h"
#include "../rocket-state/rocket-state.h"
#include "../sensors/sensors.h"
#include "../fusion/fusion.h"
#include "transmit.h"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

static ssize_t transmit(int radio, uint8_t *packet, size_t packet_size);
static uint8_t *create_block(int radio, uint8_t *packet, uint8_t *block, uint32_t *seq_num, enum block_type_e type, uint32_t mission_time);
static uint32_t us_to_ms(uint64_t us) {
  return (uint32_t)(us / 1000);
}

/* Main thread for data transmission over radio. */
void *transmit_main(void *arg) {

  int err;
  int radio; /* Radio device file descriptor */

  struct transmit_args *unpacked_args = (struct transmit_args *)arg;

  /* Packet variables. */

  uint8_t packet[PACKET_MAX_SIZE];     /* Array of bytes for packet */
  uint8_t *current_block = packet;     /* Location in packet buffer */
  uint8_t *next_block;                 /* Next block in packet buffer */
  uint32_t seq_num = 0;                /* Packet numbering */

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Transmit thread started.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  /* Get access to radio TODO: remove O_CREAT */

  radio = open(CONFIG_INSPACE_TELEMETRY_RADIO, O_WRONLY | O_CREAT);
  if (radio < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    err = errno;
    fprintf(stderr, "Error getting radio handle: %d\n", err);
#endif                             /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    pthread_exit(err_to_ptr(err)); // TODO: handle more gracefully
  }

  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.accel, ORB_ID(fusion_accel));
  setup_sensor(&sensors.baro, ORB_ID(fusion_baro));
  struct sensor_accel accel_data[ACCEL_FUSION_BUFFER_SIZE];
  struct sensor_baro baro_data[BARO_FUSION_BUFFER_SIZE];

  /* Transmit forever, regardless of rocket flight state. */

  for (;;) {
    /* Wait for new data */
    poll_sensors(&sensors);
    // Get data together, so we can block on transmit and not lose the data we're currently using
    // Could also ask for the minimum of the free space in the size of the buffer to save effort
    ssize_t accel_len = get_sensor_data(&sensors.accel, accel_data, sizeof(accel_data));
    ssize_t baro_len = get_sensor_data(&sensors.baro, baro_data, sizeof(baro_data));
    if (accel_len > 0) {
      for (int i = 0; i < (accel_len / sizeof(struct sensor_accel)); i++) {
        next_block = create_block(radio, packet, current_block, &seq_num, DATA_ACCEL_ABS, us_to_ms(accel_data[i].timestamp));
        accel_blk_init((struct accel_blk_t*)block_body(current_block), accel_data[i].x, accel_data[i].y, accel_data[i].z);
        current_block = next_block;
      }
    }
    if (baro_len > 0) {
      for (int i = 0; i < (baro_len / sizeof(struct sensor_baro)); i++) {
        next_block = create_block(radio, packet, current_block, &seq_num, DATA_PRESSURE, us_to_ms(baro_data[i].timestamp));
        pres_blk_init((struct pres_blk_t*)block_body(current_block), baro_data[i].pressure);
        current_block = next_block;

        next_block = create_block(radio, packet, current_block, &seq_num, DATA_TEMP, us_to_ms(baro_data[i].timestamp));
        temp_blk_init((struct temp_blk_t*)block_body(current_block), baro_data[i].temperature);
        current_block = next_block;
      }
    }
  }

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (close(radio) < 0) {
    fprintf(stderr, "Error closing radio handle: %d\n", err);
  }
#else
  close(radio);
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
static uint8_t *create_block(int radio, uint8_t *packet, uint8_t *block, uint32_t *seq_num, enum block_type_e type, uint32_t mission_time) {
  uint8_t *next_block = pkt_create_blk(packet, block, type, mission_time);
  if (next_block == NULL) {
    if (block != packet) {
      transmit(radio, packet, block - packet);
      *seq_num += 1;
    }
    // We can delay setting up the header and just let the addition of the first block fail
    block = init_pkt(packet, *seq_num, mission_time);
    next_block = pkt_create_blk(packet, block, type, mission_time);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    if (next_block == NULL) {
      fprintf(stderr, "Error adding block to packet after transmission, should not be possible\n");
    }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }
  return next_block;
}

/**
 * Transmits a packet over the radio with a fake delay
 *
 * @param radio The radio to transmit to
 * @param packet The completed packet to transmit
 * @param packet_size The size of the packet to transmit
 * @return The number of bytes written or a negative error code
 */
static ssize_t transmit(int radio, uint8_t *packet, size_t packet_size) {
  ssize_t written = write(radio, packet, packet_size);
  int err = 0;
  if (written == -1) {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error transmitting: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    // TODO: handle error in errno
    return -err;
  }
  usleep(500000);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Completed transmission of packet #%u of %ld bytes.\n", ((pkt_hdr_t *)packet)->packet_num,
          packet_size);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  return written;
}
