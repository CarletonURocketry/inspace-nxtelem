#ifndef _FUSION_H_
#define _FUSION_H_

#include <uORB/uORB.h>

/* UORB declarations for fused sensor data */
ORB_DECLARE(fusion_accel);
ORB_DECLARE(fusion_baro);

/* 
 * Size of the internal queues for the fusioned data, which other threads should 
 * match the size of their buffers to 
 */
#define ACCEL_FUSION_BUFFER_SIZE 100
#define BARO_FUSION_BUFFER_SIZE 100

#endif // _FUSION_H_

void *fusion_main(void *arg);
