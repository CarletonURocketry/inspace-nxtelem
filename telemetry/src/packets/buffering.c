#include "buffering.h"
#include "../syslogging.h"

/**
 * Initialize a packet queue. Must be performed before using the queue
 * 
 * @param queue Pointer to the queue
 * @return 0 on success, non-zero error code on mutex initialization failure
 */
static int packet_queue_init(struct packet_queue *queue) {
    int err = pthread_mutex_init(&queue->lock, NULL);
    if (err) {
        return err;
    }
    err = pthread_cond_init(&queue->not_empty, NULL);
    if (err) {
        pthread_mutex_destroy(&queue->lock);
        return err;
    }
    queue->head = NULL;
    queue->tail = NULL;
    return err;
}

/**
 * Remove and return the first node from the queue
 * 
 * @param queue Pointer to the queue
 * @return Pointer to the removed node, or NULL if queue is empty
 */
static packet_node_t* packet_queue_lpop(struct packet_queue *queue) {
    packet_node_t *node = NULL;
    pthread_mutex_lock(&queue->lock);
    if (queue->head) {
        node = queue->head;
        queue->head = node->next;
        if (queue->head) {
            queue->head->prev = NULL;
        } else {
            queue->tail = NULL;
        }
    }
    pthread_mutex_unlock(&queue->lock);
    return node;
}

/**
 * Remove and return the first node from the queue, block if there are no nodes
 * 
 * @param queue Pointer to the queue
 * @return Pointer to the removed node
 */
static packet_node_t* packet_queue_wait_lpop(struct packet_queue *queue) {
    packet_node_t *node = NULL;
    pthread_mutex_lock(&queue->lock);

    while (queue->head == NULL) {
        indebug("Waiting for a full packet\n");
        pthread_cond_wait(&queue->not_empty, &queue->lock);
    }

    node = queue->head;
    queue->head = node->next;
    if (queue->head) {
        queue->head->prev = NULL;
    } else {
        queue->tail = NULL;
    }
    pthread_mutex_unlock(&queue->lock);
    return node;
}

/**
 * Remove and return the last node from the queue
 * 
 * @param queue Pointer to the queue
 * @return Pointer to the removed node, or NULL if queue is empty
 */
static packet_node_t* packet_queue_rpop(struct packet_queue *queue) {
    packet_node_t *node = NULL;
    pthread_mutex_lock(&queue->lock);
    if (queue->tail) {
        node = queue->tail;
        queue->tail = node->prev;
        if (queue->tail) {
            queue->tail->next = NULL;
        } else {
            queue->head = NULL;
        }
        node->next = node->prev = NULL;
    }
    pthread_mutex_unlock(&queue->lock);
    return node;
}

/**
 * Add a node to the front of the queue.
 * 
 * @param queue Pointer to the queue structure
 * @param node Pointer to the node to add
 */
static void packet_queue_push(struct packet_queue *queue, packet_node_t *node) {
    pthread_mutex_lock(&queue->lock);
    node->next = queue->head;
    node->prev = NULL;
    if (queue->head) {
        queue->head->prev = node;
    } else {
        queue->tail = node;
    }
    queue->head = node;
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->lock);
}

/**
 * Initialize a packet buffer. Needs to be called before using the buffer.
 * 
 * @param buffer Pointer to the packet buffer to initialize
 * @return 0 or a negative error code on failure
 */
int packet_buffer_init(packet_buffer_t *buffer) {
    int err = packet_queue_init(&buffer->full_queue);
    if (err) {
        return -err;
    }
    err = packet_queue_init(&buffer->empty_queue);
    if (err) {
        return -err;
    }
    for (int i = 0; i < PACKET_QUEUE_NUM_BUFFERS; i++) {
        packet_queue_push(&buffer->empty_queue, &buffer->buffers[i]);
    }
    return 0;
}

/**
 * Takes an empty packet or the oldest full packet from the buffer
 * 
 * @param buffer The buffer to get the packet from
 * @return A packet or NULL if there are no packets in the buffer
 */
packet_node_t *packet_buffer_get_empty(packet_buffer_t *buffer) {
    packet_node_t *packet;
    packet = packet_queue_lpop(&buffer->empty_queue);
    if (!packet) {
        indebug("No empty packets to write into, getting a full packet to overwrite\n");
        packet = packet_queue_rpop(&buffer->full_queue);
    }
    if (packet) {
        packet->end = packet->packet;
    }
    indebug("Got an packet from the buffer at address %p to write into\n", packet); 
    return packet;
}

/**
 * Takes a full packet from the buffer, or blocks until there is one
 * 
 * @param buffer The buffer to get the packet from
 * @return A packet that has 0 or more bytes in it
 */
packet_node_t *packet_buffer_get_full(packet_buffer_t *buffer) {
    indebug("Getting a full packet from the buffer\n");
    return packet_queue_wait_lpop(&buffer->full_queue);
}

/**
 * Puts a empty or used packet back into the buffer
 * 
 * @param buffer The buffer to put the packet into
 * @param node The empty or used packet which can now be overwritten
 */
void packet_buffer_put_empty(packet_buffer_t *buffer, packet_node_t *node) {
    packet_queue_push(&buffer->empty_queue, node);
}

/**
 * Puts a full packet back into the buffer
 * 
 * @param buffer The buffer to put the packet into
 * @param node The full packet that is ready to be used by other parts of the system
 */
void packet_buffer_put_full(packet_buffer_t *buffer, packet_node_t *node) {
    indebug("Putting a full packet back into the buffer: ");
    if (node != NULL) {
        indebug("packet of length %d\n", node->end - node->packet);
    }
    else {
        indebug("node is NULL\n");
    }
    packet_queue_push(&buffer->full_queue, node);
}
