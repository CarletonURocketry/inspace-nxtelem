#ifndef PACKET_QUEUE_H
#define PACKET_QUEUE_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include "packets.h"

#define PACKET_QUEUE_NUM_BUFFERS 4
#define PACKET_QUEUE_NUM_CONSUMERS 2
#define PACKET_QUEUE_PACKET_MAX_SIZE 256

typedef struct {
  int consumer_flags[PACKET_QUEUE_NUM_CONSUMERS];
  int packet_length;
  pthread_rwlock_t packet_lock;

  /* Header and blocks array should be contiguous */
  pkt_hdr_t header;
  uint8_t blocks[PACKET_QUEUE_PACKET_MAX_SIZE - sizeof(pkt_hdr_t)];
} packet_buffer_t;

typedef struct {
  atomic_int num_consumers; /* */
  int packet_number; /* The current sequence number on packets */
  pthread_rwlock_t queue_lock; /* A lock on the read_queue */
  packet_buffer_t buffers[PACKET_QUEUE_NUM_BUFFERS]; /* The actual storage for this queue */
  packet_buffer_t *write_buffer; /* Where data is being written, no lock for writing, assumes only one producer*/
  packet_buffer_t *read_queue[PACKET_QUEUE_NUM_BUFFERS - 1]; /* A queue of packets ready to be read */
} packet_queue_t;

typedef int consumer_id_t;

int queue_init(packet_queue_t *queue);
int queue_write(packet_queue_t *queue, blk_hdr_t *blk_header, uint8_t *block,
                const size_t block_len, const uint32_t mission_time);
consumer_id_t queue_add_consumer(packet_queue_t *queue);
packet_buffer_t *queue_get_buffer(packet_queue_t *queue,
                                  const consumer_id_t id);
int queue_release_buffer(packet_buffer_t *read_buffer,
                         const consumer_id_t id);
uint8_t *queue_read_head(packet_buffer_t *read_buffer);
int queue_read_length(packet_buffer_t *read_buffer);

#endif /* PACKET_QUEUE_H */
