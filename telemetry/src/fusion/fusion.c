#include <pthread.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <uORB/uORB.h>

#include "fusion.h"

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/* UORB debugging help */
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
static void print_orb_state(struct orb_state *state) {
  if (state == NULL) {
    printf("State was NULL\n");
  }
  else {
    printf("Maximum Frequency: %d\n", state->max_frequency);
    printf("Min Batch Interval: %d\n", state->min_batch_interval);
    printf("Internal Queue Size: %d\n", state->queue_size);
    printf("Subscribers: %d\n", state->nsubscribers);
    printf("Generation: %d\n", state->generation);
  }
}

/* Format strings for fusioned data, useful for printing debug data using orb_info() */
static const char fusion_accel_format[] =
  "fusioned accel - timestamp:%" PRIu64 ",x:%hf,y:%hf,z:%hf,temperature:%hf";
static const char fusion_baro_format[] =
  "fusioned baro - timestamp:%" PRIu64 ",pressure:%hf,temperature:%hf";

#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/* UORB declarations for fused sensor data */
ORB_DEFINE(fusion_accel, struct sensor_accel, fusion_accel_format);
ORB_DEFINE(fusion_baro, struct sensor_baro, fusion_baro_format);

/* Input sensors */

#define ACCEL_MULTI_BUFFER_SIZE 100
#define BARO_MULTI_BUFFER_SIZE 100
#define NUM_SENSORS 2
#define SENSOR_ACCEL 0
#define SENSOR_BARO 1

void *fusion_main(void *arg) {
  int err;

  /* Input sensors, may want to directly read instead */
  orb_id_t accel_meta = orb_get_meta("sensor_accel");
  orb_id_t baro_meta = orb_get_meta("sensor_baro");
  struct sensor_accel accel_data[ACCEL_MULTI_BUFFER_SIZE];
  struct sensor_baro baro_data[BARO_MULTI_BUFFER_SIZE];

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (accel_meta == NULL) {
    fprintf(stderr, "Could not get accel metadata");
  }
  if (baro_meta == NULL) {
    fprintf(stderr, "Could not get baro metadata");
  }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  struct pollfd input_sensors[NUM_SENSORS];
  input_sensors[SENSOR_ACCEL].fd = orb_subscribe_multi(accel_meta, 0);
  input_sensors[SENSOR_ACCEL].events = POLLIN;
  input_sensors[SENSOR_BARO].fd = orb_subscribe_multi(baro_meta, 0);
  input_sensors[SENSOR_BARO].events = POLLIN;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (input_sensors[SENSOR_ACCEL].fd < 0) {
    fprintf(stderr, "Accel sensor was not opened successfully");
  }
  else if (input_sensors[SENSOR_BARO].fd < 0) {
    fprintf(stderr, "Baro sensor was not opened successfully");
  }
  else {
    struct orb_state orbstate;
    orb_get_state(input_sensors[SENSOR_ACCEL].fd, &orbstate);
    printf("Accel sensor state\n");
    print_orb_state(&orbstate);

    orb_get_state(input_sensors[SENSOR_BARO].fd, &orbstate);
    printf("Baro sensor state\n");
    print_orb_state(&orbstate);
  }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  /* Output sensors */ 

  /* Currently publishing blank data to start, might be better to try and advertise only on first fusioned data */
  struct sensor_accel output_accel = {.x = 0, .y = 0, .z = 0, .temperature = 0, .timestamp = orb_absolute_time()};
  struct sensor_baro output_baro = {.pressure = 0, .temperature = 0, .timestamp = orb_absolute_time()};
  int accel_out = orb_advertise_multi_queue(ORB_ID(fusion_accel), &output_accel, NULL, 1);
  if (accel_out < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Fusion could not advertise accel topic: %d\n", accel_out);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }
  int baro_out = orb_advertise_multi_queue(ORB_ID(fusion_baro), &output_baro, NULL, 1);
  if (baro_out < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Fusion could not advertise baro topic: %d\n", baro_out);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  /* Perform fusion on sensor data endlessly */

  for(;;) {
    /* Wait for new data */
    poll(input_sensors, NUM_SENSORS, -1);
    if (input_sensors[SENSOR_ACCEL].revents == POLLIN) {
      int len = orb_copy_multi(input_sensors[SENSOR_ACCEL].fd, accel_data, sizeof(accel_data));
      if (len < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error reading from uORB data: %d\n", len);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
      else {
        for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
          /* Simply publish the same data we got for now*/
          orb_publish(ORB_ID(fusion_accel), accel_out, &accel_data[i]);
        }
      }
    }
    if (input_sensors[SENSOR_BARO].revents == POLLIN) {
      int len = orb_copy_multi(input_sensors[SENSOR_BARO].fd, baro_data, sizeof(baro_data));
      if (len < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error reading from uORB data: %d\n", len);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
      else {
        for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
          orb_publish(ORB_ID(fusion_baro), baro_out, &baro_data[i]);
          /* Do some processing or fusion on this data */
        }
      }
    }
  }
}
