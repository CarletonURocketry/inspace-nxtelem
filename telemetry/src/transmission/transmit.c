#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "../packets/packets.h"
#include "../rocket-state/rocket-state.h"
#include "transmit.h"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

static uint32_t construct_packet(struct rocket_t *data, uint8_t *pkt,
                                 uint32_t seq_num);

/* Main thread for data transmission over radio. */
void *transmit_main(void *arg) {

  int err;
  int radio; /* Radio device file descriptor */
  ssize_t written;
  rocket_state_t *state = ((struct transmit_args_t *)(arg))->state;
  const char *radio_dev = ((struct transmit_args_t *)(arg))->radio_dev;

  /* Packet variables. */

  uint8_t packet[PACKET_MAX_SIZE]; /* Array of bytes for packet */
  uint32_t seq_num = 0;            /* Packet numbering */
  uint32_t packet_size;            /* The packet size after encoding */

  /* Get access to radio TODO: remove O_CREAT */
  radio = open(radio_dev, O_WRONLY | O_CREAT);
  if (radio < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    err = errno;
    fprintf(stderr, "Error getting radio handle: %d\n", err);
#endif                             /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    pthread_exit(err_to_ptr(err)); // TODO: handle more gracefully
  }

  /* Transmit forever, regardless of rocket flight state. */

  for (;;) {
    err = state_wait_for_change(state); // TODO handle error

    err = state_read_lock(state); // TODO: handle error

    /* Encode radio data into packet. */

    packet_size = construct_packet(&state->data, packet, seq_num);
    err = state_unlock(state); // TODO: handle error

    seq_num++; /* Increment sequence numbering */
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    printf("Constructed packet #%lu of size %lu bytes\n", seq_num, packet_size);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

    /* Transmit radio data */

    written = write(radio, packet, packet_size);
    if (written == -1) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Error transmitting: %d\n", errno);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      // TODO: handle error in errno
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

/* Construct a packet from the rocket state data.
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

  pkt_hdr_init(pkt_hdr, seq_num); /* Fresh packet header */
  size += sizeof(pkt_hdr_t);
  write_ptr += sizeof(pkt_hdr_t);

  /* Encode temperature */

  blk_hdr_t *temp_blk_hdr = (blk_hdr_t *)(write_ptr);
  blk_hdr_init(temp_blk_hdr, TYPE_DATA, DATA_TEMP);
  size += sizeof(blk_hdr_t);
  write_ptr += sizeof(blk_hdr_t);

  struct temp_blk_t *tempblk = (struct temp_blk_t *)(write_ptr);
  temp_blk_init(tempblk, data->time, data->temp);
  size += sizeof(struct temp_blk_t);
  write_ptr += sizeof(struct temp_blk_t);

  // TODO: encode rest of data

  /* Update packet header length with the size of all written bytes */

  pkt_hdr_set_len(pkt_hdr, size - sizeof(pkt_hdr_t));
  return size;
}
