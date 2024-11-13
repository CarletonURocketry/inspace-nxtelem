#ifndef PACKET_QUEUE_H
#define PACKET_QUEUE_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include "packets.h"

#define QUEUE_NUM_BUFFERS 4
#define QUEUE_NUM_CONSUMERS 2
#define QUEUE_MAX_PKT_SIZE PACKET_MAX_SIZE

/* A buffer for assembling packets piecewise and then reading all at once */

typedef struct {
  int consumer_flags[QUEUE_NUM_CONSUMERS];
  int packet_length;
  pthread_rwlock_t packet_lock;
  /* Header and blocks array should be contiguous */
  pkt_hdr_t header;
  uint8_t blocks[QUEUE_MAX_PKT_SIZE - sizeof(pkt_hdr_t)];
} packet_buffer_t;

/* A queue for writing to and reading from multiple packet buffers at once */

typedef struct {
  atomic_int num_consumers;
  int packet_number;                          /* Sequence number */
  pthread_rwlock_t queue_lock;                /* A lock on the read_queue */
  packet_buffer_t buffers[QUEUE_NUM_BUFFERS]; /* The actual storage */
  packet_buffer_t *write_buffer;              /* Where data is being written,
                                               * assumes only one producer
                                               */
  packet_buffer_t *read_queue[QUEUE_NUM_BUFFERS - 1]; /* A queue of packets */
} packet_queue_t;

/* Consumer id type */

typedef int cons_id_t;

/* Setup and writer */

int queue_init(packet_queue_t *queue);
int queue_write(packet_queue_t *queue, blk_hdr_t *blk_hdr,  uint8_t *blk,
                const size_t blk_len, const uint32_t mission_time);

/* Consumer functions*/

cons_id_t queue_add_consumer(packet_queue_t *queue);
packet_buffer_t *queue_get_buffer(packet_queue_t *queue, const cons_id_t id);
int queue_release_buffer(packet_buffer_t *read_buffer, const cons_id_t id);
uint8_t *queue_read_head(packet_buffer_t *read_buffer);
int queue_read_length(packet_buffer_t *read_buffer);

#endif /* PACKET_QUEUE_H */
