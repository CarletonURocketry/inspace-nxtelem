#ifndef _FUSION_H_
#define _FUSION_H_

#include <uORB/uORB.h>

/* UORB declarations for fused sensor data */
ORB_DECLARE(fusion_accel);
ORB_DECLARE(fusion_baro);

#endif // _FUSION_H_

void *fusion_main(void *arg);
