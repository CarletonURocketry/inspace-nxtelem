#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#include "../packets/packets.h"
#include "../rocket-state/rocket-state.h"
#include "transmit.h"

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

  /* Get access to radio */
  radio = open(radio_dev, O_WRONLY);
  if (radio < 0) {
    pthread_exit(EXIT_FAILURE); // TODO: handle more gracefully
  }

  /* Transmit forever, regardless of rocket flight state. */

  for (;;) {
    err = state_wait_for_change(state); // TODO handle error

    err = state_read_lock(state); // TODO: handle error

    /* Encode radio data into packet. */

    packet_size = construct_packet(&state->data, packet, seq_num);
    seq_num++; /* Increment sequence numbering */

    /* Transmit radio data */

    written = write(radio, packet, packet_size);
    if (written == -1) {
      // TODO: handle error in errno
    }

    err = state_unlock(state); // TODO: handle error
  }

  close(radio);
  pthread_exit(0);
}

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
}
