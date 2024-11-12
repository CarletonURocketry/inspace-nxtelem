#include "packet-queue.h"
#include <errno.h>
#include <string.h>

#define CONSUMED   1
#define UNCONSUMED 0
#define length(array) sizeof(array) / sizeof(array[0])

static packet_buffer_t *write_buff(packet_queue_t *queue) {
  return queue->write_buffer;
}

static void *init_consumed(packet_buffer_t *buff, int state) {
  for (int i = 0; i < length(buff->consumer_flags); i++) {
    buff->consumer_flags[i] = state;
  }
}

static int sum_consumed(packet_buffer_t *buff) {
  int total = 0;
  for (int i = 0; i < length(buff->consumer_flags); i++) {
    total += buff->consumer_flags[i] == CONSUMED;
  }
  return total;
}

static int was_consumed(packet_buffer_t *buff, consumer_id_t id) {
  return buff->consumer_flags[id] == CONSUMED;
}

static void set_consumed(packet_buffer_t *buff, consumer_id_t id) {
  return buff->consumer_flags[id] = CONSUMED;
}

/* Initializes a packet queue and must be called before any other operations
 * are performed on that queue
 * @param queue The packet queue to be initialized
 * @returns 0 upon success 
 */
int queue_init(packet_queue_t *queue) {
  int err;

  pthread_rwlock_init(&queue->queue_lock, NULL);
  queue->packet_number = 0;
  atomic_init(&queue->num_consumers, 0);

  for (int i = 0; i < length(queue->buffers); i++) {
    init_consumed(&queue->buffers[i], CONSUMED);
    pthread_rwlock_init(&queue->buffers[i].packet_lock, NULL);
    /* Want some way to tell the write function that the header isn't set up*/
    queue->buffers[i].packet_length = -1;
    if (i == 0) {
      queue->write_buffer = queue->buffers;
    } 
    else {
      queue->read_queue[i] = queue->buffers + i;
    }
  }
};

/* Writes initialized blocks to the queue */
int queue_write(packet_queue_t *queue, blk_hdr_t *blk_header, 
                uint8_t *block, const size_t block_len, const uint32_t mission_time) {
  int err;

  // Check if the write buffer is full or null
  if (write_buff(queue)->packet_length < 0) {
    write_buff(queue)->packet_length = 0;
    pkt_hdr_init(&queue->write_buffer->header, queue->packet_number, mission_time);
  }
  else if (write_buff(queue)->packet_length + block_len > PACKET_QUEUE_PACKET_MAX_SIZE) {
    // Get the write buffer ready to be read from
    init_consumed(write_buff(queue), UNCONSUMED);
    // Get the queue, block if someone else is using it
    pthread_rwlock_wrlock(queue->queue_lock);
    // TOTO - handle error

    // Get the next buffer
    int max_consumed = -1;
    int best_buffer = -1;
    for (int i = length(queue->read_queue) - 1; i >= 0 && max_consumed != atomic_load(&queue->num_consumers); i--) {
      int count = sum_consumed(queue->read_queue[i]);
      if (count > max_consumed) {
        err = pthread_rwlock_trywrlock(queue->read_queue[i]->packet_lock);
        if (!err) {
          max_consumed = count;
          best_buffer = i;
        } 
        // TODO - handle error
      }
    }
    // Should be impossible
    if (best_buffer == -1) {
      return ENOBUFS;
    }
    packet_buffer_t *buf = queue->read_queue[best_buffer];
    // Put the buffer on the queue, shuffle from the start of the queue, copy into the place the next guy is going - 
    for (int i = best_buffer; i > 0 ; i--) {
      queue->read_queue[i] = queue->read_queue[i - 1];
    }
    queue->read_queue[0] = queue->write_buffer;
    queue->write_buffer = buf;
    pthread_rwlock_unlock(queue->queue_lock);

    // Now that we have our new buffer, set it up
    write_buff(queue)->packet_length = 0;
    // Because only one producer accessing packet_number variable, no mutex
    // Conversion from int -> uint8_t should cause the wrap-around we want
    pkt_hdr_init(&write_buff(queue)->header, queue->packet_number++, mission_time);
  }
  if (!pkt_add_blk(&write_buff(queue)->header, blk_header, block, mission_time)) {
    /* 
       This packet is incompatible with this block, start a new packet. If 
       this happens frequently, we should add a check to here that gets the
       current time and sees if it's incompatible with this header. If it is,
       start a new block. Otherwise, discard the block instead of making a new
       packet
    */
    // Do what we did above
  }
  // Assume only one producer - so don't need a mutex here
  // Our thing has space, and has been initialized - write into it
  memcpy(write_buff(queue)->blocks + write_buff(queue)->packet_length, block, block_len);
  write_buff(queue)->packet_length += block_len;
}

/* Add a consumer to the packet queue (to keep track of which consumers have)
 * or have not consumed a particular packet
 * @param queue The packet queue to register the consumer with
 * @returns The id that should be used with functions for reading from the
 * queue
 */
consumer_id_t queue_add_consumer(packet_queue_t *queue) {
  consumer_id_t new_id = atomic_fetch_add(&queue->num_consumers, 1);
  if (new_id <= PACKET_QUEUE_NUM_CONSUMERS) {
    return new_id;
  }
  return -1;
}

/* Get a buffer to read out of from the queue, very important nothing is
 * modified here through
 * @param queue The packet queue to get a ready buffer from
 * @param id An id generated by queue_add_consumer
 * @returns A pointer to a read-locked buffer, which should be accessed
 * through the functions below, or NULL if no new data is available 
 */
packet_buffer_t *queue_get_buffer(packet_queue_t *queue, const consumer_id_t id) {
  /* Only use one return statement to ensure we unlock at the end */
  packet_buffer_t *output = NULL;
  pthread_rwlock_rdlock(&queue->queue_lock);
  for (int i = 0; (i < length(queue->read_queue)) && (output == NULL); i++) {
    if (!was_consumed(queue->read_queue[i], id)) {
      output = queue->read_queue[i];
      /* To be unlocked in queue_release_buffer */
      pthread_rwlock_rdlock(output->packet_lock);
    }
  }
  pthread_rwlock_unlock(&queue->queue_lock);
  return NULL;
  /*
     May actually be worth trading rwlock on queue for a mutex, then make this
     case a sleep on a condvar, which the producer can broadcast when a new
     packet is made available
  */
}

/* Release a buffer back to the queue. Ideally, only have one buffer in use at 
 * a time
 * @param read_buffer A buffer given by queue_get_buffer to release
 * @param id An id generated by queue_add_consumer
 * @returns A value of 0 when the queue is successfully released
 */
int queue_release_buffer(packet_buffer_t *read_buffer, const consumer_id_t id) {
  set_consumed(read_buffer, id);
  pthread_rwlock_unlock(read_buffer->packet_lock);
}

/* Get the start of the data to read from a buffer. A memcpy starting in this 
 * location will get all useful data from the buffer when combined with the 
 * length given by queue_read_length
 * @param read_buffer A buffer given by queue_get_buffer
 * @returns A pointer to the start of the useful data. Read from this point in
 * the buffer for the length given by queue_read_length
 */
uint8_t *queue_read_head(packet_buffer_t *read_buffer) {
  return &read_buffer->header;
}

/* Get the length of the data in this packet buffer, including the packet 
 * header
 * @param read_buffer The packet being read from 
 * @returns The length of useful data in the given buffer 
 */
int queue_read_length(packet_buffer_t *read_buffer) {
  return sizeof(read_buffer->header) + read_buffer->packet_length;
}
