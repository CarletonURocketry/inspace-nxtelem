#ifndef _INSPACE_PACKET_QUEUE_H_
#define _INSPACE_PACKET_QUEUE_H_

#include <pthread.h>
#include <stdbool.h>
#include "packets.h"

typedef struct packet_node packet_node_t;

/* The data type exchanged by the buffer */
struct packet_node {
    /* A packet that can be written directly to the radio or storage medium */
    uint8_t packet[PACKET_MAX_SIZE];
    /* Points to the end of a packet, where the blocks stop */
    uint8_t *end;
    /* The next node in the packet queue */
    packet_node_t *next;
    /* The previous node in the packet queue */
    packet_node_t *prev;
};

/* A doubly-linked list used to store packets that can be used concurrently */
struct packet_queue {
    /* A lock for concurrency */
    pthread_mutex_t lock;
    /* The first node in the queue */
    packet_node_t *head;
    /* The last node in the queue */
    packet_node_t *tail;
};

/* The number of buffers statically allocated to the packet buffer, to save allocation time */
#define PACKET_QUEUE_NUM_BUFFERS 3

/* Uses doubly-linked lists to offer a queue of packets for a writer and single reader */
typedef struct {
    /* Statically allocated buffers, added upon initialization */
    packet_node_t buffers[PACKET_QUEUE_NUM_BUFFERS];
    /* A queue of full packets, to be logged or transmitted */
    struct packet_queue full_queue;
    /* A queue of empty packets to be written into */
    struct packet_queue empty_queue;
} packet_buffer_t;

void packet_buffer_init(packet_buffer_t *buffer); 
packet_node_t *packet_buffer_get_empty(packet_buffer_t *buffer);
packet_node_t *packet_buffer_get_full(packet_buffer_t *buffer);
void packet_buffer_put_empty(packet_buffer_t *buffer, packet_node_t *node);
void packet_buffer_put_full(packet_buffer_t *buffer, packet_node_t *node);

#endif // _INSPACE_PACKET_QUEUE_H_
