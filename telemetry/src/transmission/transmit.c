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

static uint32_t construct_packet(struct rocket_t *data, uint8_t *pkt,
                                 uint32_t seq_num);
static int transmit(int radio, uint8_t *packet, uint32_t packet_size);

/* Main thread for data transmission over radio. */
void *transmit_main(void *arg) {

  int err;
  int radio; /* Radio device file descriptor */
  uint32_t version = 0;
  rocket_state_t *state = (rocket_state_t *)(arg);

  /* Packet variables. */

  uint8_t packet[PACKET_MAX_SIZE]; /* Array of bytes for packet */
  uint32_t seq_num = 0;            /* Packet numbering */
  uint32_t packet_size;            /* The packet size after encoding */

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

  /* Transmit forever, regardless of rocket flight state. */

  for (;;) {

    
    struct uorb_inputs sensors;
    clear_uorb_inputs(&sensors);
    setup_sensor(&sensors.accel, ORB_ID(fusion_accel));
    setup_sensor(&sensors.baro, ORB_ID(fusion_baro));
    struct sensor_accel accel_data[ACCEL_FUSION_BUFFER_SIZE];
    struct sensor_baro baro_data[BARO_FUSION_BUFFER_SIZE];

    /* Wait for new data */
    poll_sensors(&sensors);
    int len = get_sensor_data(&sensors.accel, accel_data, sizeof(accel_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
        // Make and add a new block
        // TODO - make a new block here and add it 
      }
    } 
    len = get_sensor_data(&sensors.baro, baro_data, sizeof(baro_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
        // Make and add a new block
        // TODO - make a new block here and add it
      }
    }
    
    /* Transmit radio data */
    transmit(radio, packet, packet_size);

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

/** Construct a packet from the rocket state data.
 *
 * @param data The rocket data to encode
 * @param pkt The packet buffer to encode the data in
 * @param seq_num The sequence number of the packet
 * @return The packet's length in bytes
 */
static uint32_t construct_packet(struct rocket_t *data, uint8_t *pkt,
                                 uint32_t seq_num) {

  pkt_hdr_t *pkt_hdr = (pkt_hdr_t *)(pkt); /* Packet header */
  uint8_t *write_ptr = pkt;                /* Location in packet buffer */
  uint32_t size = 0;                       /* Packet size */

  pkt_hdr_init(pkt_hdr, seq_num, data->time); /* Fresh packet header */
  size += sizeof(pkt_hdr_t);
  write_ptr += sizeof(pkt_hdr_t);

  /* Encode temperature */

  blk_hdr_t *temp_blk_hdr = (blk_hdr_t *)(write_ptr);
  blk_hdr_init(temp_blk_hdr, DATA_TEMP);
  size += sizeof(blk_hdr_t);
  write_ptr += sizeof(blk_hdr_t);

  struct temp_blk_t *tempblk = (struct temp_blk_t *)(write_ptr);
  temp_blk_init(tempblk, data->temp);
  size += sizeof(struct temp_blk_t);
  write_ptr += sizeof(struct temp_blk_t);

  pkt_add_blk(pkt_hdr, temp_blk_hdr, tempblk, data->time);

  // TODO: encode rest of data

  /* Update packet header length with the size of all written bytes */

  return size;
}

/**
 * Transmits a packet over the radio with a fake delay
 * 
 * @param radio The radio to transmit to
 * @param packet The completed packet to transmit
 * @param packet_size The size of the packet to transmit
 * @return The number of bytes written or a negative error code
 */
static int transmit(int radio, uint8_t *packet, uint32_t packet_size) {
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
  printf("Completed transmission of packet #%lu of %ld bytes.\n", (pkt_hdr_t *)packet->packet_num, 
          packet_size);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  return written;
}
