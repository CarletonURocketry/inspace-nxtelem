#include <pthread.h>
#include <time.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "../rocket-state/rocket-state.h"
#include "../fusion/fusion.h"
#include "../sensors/sensors.h"
#include "collection.h"

/* Measurement interval in nanoseconds */

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
#define INTERVAL (1e9 / CONFIG_INSPACE_TELEMETRY_RATE)
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

static uint32_t ms_since(struct timespec *start);

/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {

  int err;
  enum flight_state_e flight_state;
  struct timespec start_time;
  struct collection_args *unpacked_args = (struct collection_args *)(arg);
  rocket_state_t *state = unpacked_args->state;
  packet_buffer_t *logging_buffer = unpacked_args->logging_buffer;
  packet_buffer_t *transmit_buffer = unpacked_args->transmit_buffer;

  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.accel, orb_get_meta("sensor_accel"));
  setup_sensor(&sensors.baro, orb_get_meta("sensor_baro"));
  setup_sensor(&sensors.mag, orb_get_meta("sensor_mag"));
  setup_sensor(&sensors.gyro, org_get_meta("sensor_gyro"));
  setup_sensor(&sensors.gnss, orb_get_meta("sensor_gnss"));

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

    /* Wait for new data */
    poll_sensors(&sensors);
    int len = get_sensor_data(&sensors.accel, accel_data, sizeof(accel_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG) && defined(CONFIG_DEBUG_UORB)
        /*const struct orb_metadata *meta = ORB_ID(fusion_accel);
        orb_info(meta->o_format, meta->o_name, &accel_data[i]);*/
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
    } 
    len = get_sensor_data(&sensors.baro, baro_data, sizeof(baro_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG) && defined(CONFIG_DEBUG_UORB)
        /*const struct orb_metadata *meta = ORB_ID(fusion_baro);
        orb_info(meta->o_format, meta->o_name, &baro_data[i]);*/
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
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
