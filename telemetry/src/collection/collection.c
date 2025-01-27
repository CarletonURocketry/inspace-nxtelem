#include <pthread.h>
#include <time.h>
#include <nuttx/sensors/sensor.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>


#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

#include "../rocket-state/rocket-state.h"
#include "collection.h"

/* Measurement interval in nanoseconds */

#if CONFIG_INSPACE_TELEMETRY_RATE != 0
#define INTERVAL (1e9 / CONFIG_INSPACE_TELEMETRY_RATE)
#endif /* CONFIG_INSPACE_TELEMETRY_RATE != 0 */

static uint32_t ms_since(struct timespec *start);

void print_accel(const struct sensor_accel *accel) {
  printf("Data recieved from accel: X: %lf Y: %lf Z: %lf, %lu\n", accel->x, accel->y, accel->z, accel->timestamp);
}
void print_baro(const struct sensor_baro *baro) {
  printf("Data recieved from baro: temp: %lf pressure: %lf, %lu\n", baro->pressure, baro->temperature, baro->timestamp);
}
/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {

  int err;
  enum flight_state_e flight_state;
  struct timespec start_time;
  rocket_state_t *state = (rocket_state_t *)(arg);

  struct pollfd fds[2];
  struct sensor_accel accel;
  struct sensor_baro baro;
  
  union {
    struct sensor_accel accel[10];
    struct sensor_baro baro[10];
  } data;

  int accel_fd = open("/dev/uorb/sensor_accel0", O_RDONLY);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (accel_fd < 0) {
    fprintf(stderr, "Couldn't open accel device\n");
  } 
#endif
  fds[0].fd = accel_fd;
  fds[0].events = POLLIN;
  int baro_fd = open("/dev/uorb/sensor_baro0", O_RDONLY);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (baro_fd < 0) {
    fprintf(stderr, "Couldn't open baro device\n");
  }
#endif
  fds[1].fd = baro_fd;
  fds[1].events = POLLIN;

  // Configure to have a higher latency, allowing for batching to occur. Currently set latency for all

  unsigned int latency_us = 50 * 1000;
  for (int i = 0; i < sizeof(fds) / sizeof(struct pollfd); i++) {
    err = ioctl(fds[i].fd, SNIOC_BATCH, latency_us); // A latency of 50ms -> (50ms / (100Hz sample rate)) = 5 measurements per read, hopefully
    if (err < 0) {
      fprintf(stderr, "Couldn't set batch interval of %ud", latency_us);
    }
  }

  // Test reading, forever
  for (;;) {
    printf("calling poll()...\n");
    if (poll(fds, sizeof(fds) / sizeof(struct pollfd), -1) > 0) {
      printf("woken up\n");
      if (fds[0].revents == POLLIN) {
        int len = read(fds[0].fd, &data, sizeof(data));
        if (len > 0) {
          if (len % sizeof(struct sensor_accel) != 0) {
            printf("Recieved odd number of bytes: %d", len);
          } else {
            for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
              print_accel(&data.accel[i]);
            }
          }
        }
      }
      if (fds[1].revents == POLLIN) {
        int len = read(fds[1].fd, &data, sizeof(data));
        if (len > 0) {
          if (len % sizeof(struct sensor_baro) != 0) {
            printf("Recieved odd number of bytes: %d", len);
          } else {
            for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
              print_baro(&data.baro[i]);
            }
          }
        }
      }
    }
    usleep(latency_us);
  }
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

    /* Collect data; TODO */

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    printf("Collecting data...\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

    /* Put data in the state structure */

    err = state_write_lock(state);
    if (err) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Could not acquire state write lock: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      continue;
    }

    state->data.temp++; // TODO: remove and replace with real data
    
    state->data.time = ms_since(&start_time); /* Measurement time */

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
