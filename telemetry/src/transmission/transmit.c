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

/* Main thread for data transmission over radio. */
void *transmit_main(void *arg) {

  int err;
  int radio; /* Radio device file descriptor */
  struct transmit_args *unpacked_args = (struct transmit_args *)arg;
  rocket_state_t *state = unpacked_args->state;
  packet_buffer_t *buffer = unpacked_args->buffer;
  uint32_t seq_num = 0;

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
      packet_node_t *next_packet = packet_buffer_get_full(buffer);
      ((pkt_hdr_t *)next_packet->packet)->packet_num = seq_num++;
      transmit(radio, next_packet->packet, next_packet->end - next_packet->packet);
      packet_buffer_put_empty(buffer, next_packet);
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
