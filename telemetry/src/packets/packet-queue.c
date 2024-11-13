#include "packet-queue.h"
#include <errno.h>
#include <string.h>

#define length(array) sizeof(array) / sizeof(array[0])

/* Flags for determining if a consumer has already seen a packet */
enum cons_flag_e {
  /* The consumer has used this packet */
  CONSUMED = 1,
  /* The consumer has not used this packet */
  UNCONSUMED = 0,
};

/* Short form for getting the write buffer
 * @param queue The queue to get a write buffer from
 * @return The write buffer
 */
static packet_buffer_t *wrbuf(packet_queue_t *queue) {
  return queue->write_buffer;
}

/* Set all the consumption flags to a certain state
 * @param buf The buffer to set consumption flags in
 * @param state The state to set them to
 */
static void *init_consumed(packet_buffer_t *buf, enum cons_flag_e state) {
  for (int i = 0; i < length(buf->consumer_flags); i++) {
    buf->consumer_flags[i] = state;
  }
}

/* Get the sum of all consumption flags, even those not in use
 * @param buf The buffer to sum consumption flags for
 * @return The number of consumers that have consumed this packet
 */
static int sum_consumed(packet_buffer_t *buf) {
  int total = 0;
  for (int i = 0; i < length(buf->consumer_flags); i++) {
    total += buf->consumer_flags[i] == CONSUMED;
  }
  return total;
}

/* Check if this consumer has already consumed this buffer
 * @param buf The buffer to check consumer flags in
 * @param id The id of the consumer who's flag is being checked
 * @return 1 if the packet was consumed, 0 otherwise
 */
static int was_consumed(packet_buffer_t *buf, consumer_id_t id) {
  return buf->consumer_flags[id] == CONSUMED;
}

/* Set this buffer as consumed
 * @param buf The buffer to set the consumer flag in
 * @param id The id of the consumer who's flag is being set
 */
static void set_consumed(packet_buffer_t *buf, consumer_id_t id) {
  return buf->consumer_flags[id] = CONSUMED;
}

/* Initializes a packet queue and must be called before any other operations
 * are performed on that queue
 * @param queue The packet queue to be initialized
 * @return 0 upon success, or an error
 */
int queue_init(packet_queue_t *queue) {
  int err;

  err = pthread_rwlock_init(&queue->queue_lock, NULL);
  if (!err) {
    return err;
  }
  queue->packet_number = 0;
  atomic_init(&queue->num_consumers, 0);

  for (int i = 0; i < length(queue->buffers); i++) {
    init_consumed(&queue->buffers[i], CONSUMED);
    err = pthread_rwlock_init(&queue->buffers[i].packet_lock, NULL);
    if (!err) {
      return err;
    }
    /* Want some way to tell the write function that the header isn't set up*/
    queue->buffers[i].packet_length = PACKET_QUEUE_PACKET_MAX_SIZE;
    if (i == 0) {
      queue->write_buffer = queue->buffers;
    }
    else {
      queue->read_queue[i] = &queue->buffers[i];
    }
  }
  return 0;
};

/* Put the current write buffer back in the read queue and get a new one
 * @param queue The queue to use
 * @return 0 upon success, or an error
 */
static int swap_write_buff(packet_queue_t *queue) {
  int err;
  err = pthread_rwlock_wrlock(queue->queue_lock);
  if (err) {
    return err;
  }
  /* Finding the best choice for the next write buffer */
  int max_consumed = -1;
  int best_buffer = -1;
  for (int i = length(queue->read_queue) - 1; i >= 0; i--) {
    int count = sum_consumed(queue->read_queue[i]);
    if (count > max_consumed) {
      err = pthread_rwlock_trywrlock(queue->read_queue[i]->packet_lock);
      if (!err) {
        max_consumed = count;
        best_buffer = i;
      }
      if (count >= atomic_load(&queue->num_consumers)) {
        break;
      }
    }
  }
  /* If num of buffs > num consumers + 2 will always be true */
  if (best_buffer != -1) {
    packet_buffer_t *buf = queue->read_queue[best_buffer];
    for (int i = best_buffer; i > 0 ; i--) {
      queue->read_queue[i] = queue->read_queue[i - 1];
    }
    pthread_rwlock_unlock(queue->write_buffer->packet_lock);
    queue->read_queue[0] = queue->write_buffer;
    queue->write_buffer = buf;
  }
  pthread_rwlock_unlock(queue->queue_lock);
  return 0;
}

/* Set up a write buffer for writing to
 * @param queue The queue with a write buffer to set up
 * @mission_time The time to use for the new packet header
 */
static int setup_write_buff(packet_queue_t *queue, const uint32_t mission_time) {
  /* Set these flags here instead when swapping in */
  init_consumed(wrbuf(queue), UNCONSUMED);
  wrbuf(queue)->packet_length = 0;
  /* Assume only one producer in access of packet_number */
  pkt_hdr_init(&wrbuf(queue)->header, queue->packet_number++, mission_time);
}

static int new_write_buff(packet_queue_t *queue, const uint32_t mission_time) {
  swap_write_buff(queue);
  setup_write_buff(queue, mission_time);
}

/* Write a block to the packet currently being written to, change which packet
 * is being written to if we run out of space or the packet is incompatible
 * @param queue The queue to write to
 * @param blk_hdr The header of the block to write
 * @param blk The block to write
 * @param blk_len The length of the block to write
 * @param mission_time The mission time associated with this block
 */
int queue_write(packet_queue_t *queue, blk_hdr_t *blk_hdr, uint8_t *blk,
                const size_t blk_len, const uint32_t mission_time) {
  if (wrbuf(queue)->packet_length + blk_len > length(wrbuf(queue)->blocks)) {
    new_write_buff(queue, mission_time);
  }
  if (!pkt_add_blk(&wrbuf(queue)->header, blk_hdr, blk, mission_time)) {
    /*
       This packet is incompatible with this block, start a new packet. If
       this happens frequently, we should add a check to here that gets the
       current time and sees if it's incompatible with this header. If it is,
       start a new block. Otherwise, discard the block instead of making a new
       packet
    */
    new_write_buff(queue, mission_time);
    /* Hopefully that work was worth it */
    if (!pkt_add_blk(&wrbuf(queue)->header, blk_hdr, blk, mission_time)) {
      return -1;
    }
  }
  /* No mutex here, since we assume only one writer */
  memcpy(wrbuf(queue)->blocks + wrbuf(queue)->packet_length, blk, blk_len);
  wrbuf(queue)->packet_length += blk_len;
  return blk_len;
}

/* Add a consumer to the packet queue (to keep track of which consumers have)
 * or have not consumed a particular packet
 * @param queue The packet queue to register the consumer with
 * @return The id that should be used with functions for reading from the
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
 * @return A pointer to a read-locked buffer, which should be accessed
 * through the functions below, or NULL if no new data is available
 */
packet_buffer_t *queue_get_buffer(packet_queue_t *queue,
                                  const consumer_id_t id) {
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
 * @return A value of 0 when the queue is successfully released
 */
int queue_release_buffer(packet_buffer_t *read_buffer,
                         const consumer_id_t id) {
  set_consumed(read_buffer, id);
  pthread_rwlock_unlock(read_buffer->packet_lock);
}

/* Get the start of the data to read from a buffer. A memcpy starting in this
 * location will get all useful data from the buffer when combined with the
 * length given by queue_read_length
 * @param read_buffer A buffer given by queue_get_buffer
 * @return A pointer to the start of the useful data. Read from this point in
 * the buffer for the length given by queue_read_length
 */
uint8_t *queue_read_head(packet_buffer_t *read_buffer) {
  return &read_buffer->header;
}

/* Get the length of the data in this packet buffer, including the packet
 * header
 * @param read_buffer The packet being read from
 * @return The length of useful data in the given buffer
 */
int queue_read_length(packet_buffer_t *read_buffer) {
  return sizeof(read_buffer->header) + read_buffer->packet_length;
}
