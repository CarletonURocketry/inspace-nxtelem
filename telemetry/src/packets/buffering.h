#ifndef _INSPACE_PACKET_QUEUE_H_
#define _INSPACE_PACKET_QUEUE_H_

#include <pthread.h>
#include <stdbool.h>
#include "packets.h"

typedef struct packet_node packet_node_t;

struct packet_node {
    uint8_t packet[PACKET_MAX_SIZE];
    packet_node_t *next;
    packet_node_t *prev;
};

struct packet_queue {
    pthread_mutex_t lock;
    packet_node_t *head;
    packet_node_t *tail;
};

#define PACKET_QUEUE_NUM_BUFFERS 3

typedef struct {
    packet_node_t buffers[PACKET_QUEUE_NUM_BUFFERS];
    struct packet_queue full_queue;
    struct packet_queue empty_queue;
} packet_buffer_t;

void packet_buffer_init(packet_buffer_t *buffer); 
packet_node_t *packet_buffer_get_empty(packet_buffer_t *buffer);
packet_node_t *packet_buffer_get_full(packet_buffer_t *buffer);
void packet_buffer_put_empty(packet_buffer_t *buffer, packet_node_t *node);
void packet_buffer_put_full(packet_buffer_t *buffer, packet_node_t *node);
#endif // _INSPACE_PACKET_QUEUE_H_
