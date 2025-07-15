#ifndef _FUSION_H_
#define _FUSION_H_

#include <uORB/uORB.h>

#include "../rocket-state/rocket-state.h"

/* UORB declarations for fused sensor data */
ORB_DECLARE(fusion_altitude);

/*
 * Size of the internal queues for the fusioned data, which other threads should
 * match the size of their buffers to
 */
#define ALT_FUSION_BUFFER 5

/* A fusioned altitude sample */
struct fusion_altitude {
    uint64_t timestamp; /* Timestamp in microseconds */
    float altitude;     /* Altitude in meters */
};

/* The arguments required by the fusion thread */
struct fusion_args {
    rocket_state_t *state; /* A pointer to the rocket state shared between all threads */
};

void *fusion_main(void *arg);

#endif // _FUSION_H_
