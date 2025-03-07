#include <pthread.h>
#include <time.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <uORB/uORB.h>

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "../rocket-state/rocket-state.h"
#include "../fusion/fusion.h"
#include "collection.h"

/* Measurement interval in nanoseconds */

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
#define INTERVAL (1e9 / CONFIG_INSPACE_TELEMETRY_RATE)
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

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

/* Sizes of buffers for getting new data from uORB */
#define ACCEL_MULTI_BUFFER_SIZE 1
#define BARO_MULTI_BUFFER_SIZE 1

struct uorb_inputs {
  struct pollfd accel;
  struct pollfd baro;
};
#define NUM_SENSORS sizeof(struct uorb_inputs) / sizeof(struct pollfd)

static uint32_t ms_since(struct timespec *start);
static void update_state(rocket_state_t *state, struct sensor_baro *baro_data);

/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {

  int err;
  enum flight_state_e flight_state;
  struct timespec start_time;
  rocket_state_t *state = (rocket_state_t *)(arg);

  /* We want collection to get data for the logging & transmission threads 
   * because sensors (and uorb) only queue data up to the size of the 
   * sensor's internal buffer. If we set these large could have the same
   * effect as using buffers but may be less understandable
   */
  orb_id_t accel_meta = ORB_ID(fusion_accel);
  orb_id_t baro_meta = ORB_ID(fusion_baro);
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

  struct uorb_inputs sensors;
  sensors.accel.fd = orb_subscribe_multi(accel_meta, 0);
  sensors.accel.events = POLLIN;
  sensors.baro.fd = orb_subscribe_multi(baro_meta, 0);
  sensors.baro.events = POLLIN;

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (sensors.accel.fd < 0) {
    fprintf(stderr, "Accel sensor was not opened successfully");
  }
  else if (sensors.baro.fd < 0) {
    fprintf(stderr, "Baro sensor was not opened successfully");
  }
  else {
    struct orb_state orbstate;
    orb_get_state(sensors.accel.fd, &orbstate);
    printf("\nAccel sensor state\n");
    print_orb_state(&orbstate);

    orb_get_state(sensors.baro.fd, &orbstate);
    printf("\nBaro sensor state\n");
    print_orb_state(&orbstate);
  }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
  struct timespec period_start;
  struct timespec next_interval;
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Collection thread started.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  /* Get startup time */

  clock_gettime(CLOCK_MONOTONIC, &start_time);

  /* Measure data forever */

  for (;;) {

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
    /* Get the start of this measurement period */

    clock_gettime(CLOCK_MONOTONIC, &period_start);
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

    /* Get the current flight state */

    err = state_get_flightstate(state, &flight_state); // TODO: error handling
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    if (err) {
      fprintf(stderr, "Could not get flight state: %d\n", err);
    }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */


#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    printf("Collecting data...\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

    /* Wait for new data */
    poll((struct pollfd*)&sensors, NUM_SENSORS, -1);
    if (sensors.accel.revents == POLLIN) {
      int len = orb_copy_multi(sensors.accel.fd, accel_data, sizeof(accel_data));
      if (len < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Collection: Error reading from uORB data: %d\n", len);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
      else {
        for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG) && defined(CONFIG_DEBUG_UORB)
          orb_info(accel_meta->o_format, accel_meta->o_name, &accel_data[i]);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
        }
      }
    }
    if (sensors.baro.revents == POLLIN) {
      int len = orb_copy_multi(sensors.baro.fd, baro_data, sizeof(baro_data));
      if (len < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Collection: Error reading from uORB data: %d\n", len);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
      else {
        for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG) && defined(CONFIG_DEBUG_UORB)
          orb_info(baro_meta->o_format, baro_meta->o_name, &baro_data[i]);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
          update_state(state, &baro_data[i]);
        }
      }
    }

    /* Decide whether to move to lift-off state. TODO: real logic */

    switch (flight_state) {
    case STATE_IDLE: {
      // Perform lift-off detection operations
      // If lift-off:
      // state_set_flightstate(state, STATE_AIRBORNE);
    } break;
    case STATE_AIRBORNE: {
      // Perform landing detection operations
      // If landed
      // state_set_flightstate(state, STATE_LANDED);
    } break;
    case STATE_LANDED:
      break; /* Do nothing, just continue collecting */
    }

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
    /* Once operations are complete, wait until the next measuring period */

    next_interval.tv_nsec = (INTERVAL - (ms_since(&period_start) * 1e6));

    clock_nanosleep(CLOCK_MONOTONIC, 0, &next_interval, NULL);
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */
  }

  pthread_exit(0);
}

/*
 * Get the time difference from start until now in milliseconds.
 * @param start The start time.
 * @return The time elapsed since start in milliseconds.
 */
static uint32_t ms_since(struct timespec *start) {

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now); // TODO: this can fail

  struct timespec diff = {
      .tv_sec = (now.tv_sec - start->tv_sec),
      .tv_nsec = now.tv_nsec - start->tv_nsec,
  };

  if (diff.tv_nsec < 0) {
    diff.tv_nsec += 1e9;
    diff.tv_nsec--;
  }

  return diff.tv_sec * 1000 + (diff.tv_nsec / 1e6);
}


static void update_state(rocket_state_t *state, struct sensor_baro *baro_data) {
    /* Put data in the state structure */
    int err = state_write_lock(state);
    if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Could not acquire state write lock: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      return;
    }

    state->data.temp = baro_data->temperature * 1000; /* Convert to millidegrees */
    state->data.time = baro_data->timestamp; /* Measurement time */

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    printf("Measurement time: %lu ms\n", state->data.time);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

    err = state_unlock(state);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    if (err) {
      fprintf(stderr, "Could not release state write lock: %d\n", err);
    }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

    err = state_signal_change(state);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    if (err) {
      fprintf(stderr, "Could not signal state change: %d\n", err);
    }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
}
