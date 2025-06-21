#include <pthread.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>
#include <math.h>

#include "../sensors/sensors.h"
#include "../rocket-state/rocket-state.h"
#include "fusion.h"
#include "detector.h"


#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/* The pressure at sea level in millibar*/
#define SEA_PRESSURE 1013.25

/* The universal gas constant */
#define GAS_CONSTANT 8.31432

/* Constant for acceleration due to gravity */
#define GRAVITY 9.80665

/* Constant for the mean molar mass of atmospheric gases */
#define MOLAR_MASS 0.0289644

/* Constant for the conversion from Celsius to Kelvin */
#define KELVIN 273

// TODO - remove this, but potentially keep the time lockout
/* The amount of time to not perform detection for (if the idle state is loaded)
 * so that the detector can have a decent amount of data to work with. Otherwise,
 * the first few data points will have nothing to compare against and could be very
 * noisy. After this time, also set the default elevation. If the rocket turns out to
 * actually be flying this whole time, we'll likely detect it immediately after the new
 * launch elevation is set. In loaded states other than idle, noise shouldn't be a large concern
 */
#define DETECTION_INIT_SETTLE_TIME 100000

/* UORB declarations */
#if defined(CONFIG_DEBUG_UORB)
static const char fusion_alt_format[] = "fusioned altitude - timestamp:%" PRIu64 ",altitude:%hf";
ORB_DEFINE(fusion_altitude, struct fusion_altitude, fusion_alt_format);
#else
ORB_DEFINE(fusion_altitude, struct fusion_altitude, 0);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

/* Buffers for inputs, best to match to the size of the internal sensor buffers */
#define BARO_INPUT_BUFFER_SIZE 5

/* Buffers for inputs, best to match to the size of the internal sensor buffers */
#define ACCEL_INPUT_BUFFER_SIZE 5

static struct fusion_altitude calculate_altitude(struct sensor_baro *baro_data);
static struct accel_sample calculate_accel_magnitude(struct sensor_accel *accel_data);

void *fusion_main(void *arg) {
  rocket_state_t *state = ((struct fusion_args *)arg)->state;
  enum flight_state_e flight_state;
  enum flight_substate_e flight_substate;
  int32_t init_elevation;

  state_get_flightstate(state, &flight_state);
  state_get_flightsubstate(state, &flight_substate);
  state_get_elevation(state, &init_elevation);

  /* Input sensors, may want to directly read instead */
  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.baro, orb_get_meta("sensor_baro"));
  setup_sensor(&sensors.accel, orb_get_meta("sensor_accel"));
  struct sensor_baro baro_data[BARO_INPUT_BUFFER_SIZE];
  struct sensor_accel accel_data[ACCEL_INPUT_BUFFER_SIZE];

  struct detector detector;
  detector_init(&detector, orb_absolute_time());
  detector_set_state(&detector, flight_state, flight_substate);

  /* Set the elevation, which will either be a remembered value or a sensible default (and convert to meters) */
  // detector_set_elevation(&detector, init_elevation / 1000);

  /* Currently publishing blank data to start, might be better to try and advertise only on first fusioned data */
  struct fusion_altitude calculated_altitude = {.altitude = 0, .timestamp = orb_absolute_time()};
  struct accel_sample calculated_accel_mag = {0};
  int altitude_fd = orb_advertise_multi_queue(ORB_ID(fusion_altitude), &calculated_altitude, NULL, ALT_FUSION_BUFFER);
  if (altitude_fd < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Fusion could not advertise altitude topic: %d\n", altitude_fd);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  /* Perform fusion on sensor data endlessly */

  /* The time included with data is based on the uORB time so use that */
  int time_lockout = 0;
  uint64_t enable_time = orb_absolute_time() + DETECTION_INIT_SETTLE_TIME;
  if (flight_state == STATE_IDLE) {
    time_lockout = 1;
  }

  for(;;) {
    /* Wait for new data */
    poll_sensors(&sensors);
    int len = get_sensor_data(&sensors.baro, baro_data, sizeof(baro_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
        calculated_altitude = calculate_altitude(&baro_data[i]);
        detector_add_alt(&detector, (struct altitude_sample *)&calculate_altitude);
        orb_publish(ORB_ID(fusion_altitude), altitude_fd, &calculated_altitude);
        /* Do some processing or fusion on this data */
      }
    }
    len = get_sensor_data(&sensors.accel, accel_data, sizeof(accel_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
        calculated_accel_mag = calculate_accel_magnitude(&accel_data[i]);
        detector_add_accel(&detector, (struct accel_sample *)&calculated_accel_mag);
      }
    }

    /* Check if we should skip detection, so that the detector can get some data to work with first */
    if (time_lockout) {
      if (orb_absolute_time() > enable_time) {
        float elevation = detector_get_alt(&detector);
        // Allow the detector to set its own elevation
        // detector_set_elevation(&detector, elevation);
        state_set_elevation(state, elevation * 1000); // Convert to millimeters
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        printf("Time lockout on detection is up, setting new landed elevation to %fm above sea level\n", elevation);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
        time_lockout = 0;
      } else {
        continue;
      }
    }

    /* Run detection. Potentially run periodically instead of every update */
    switch(detector_detect(&detector)) {
      case DETECTOR_AIRBORNE_EVENT:
        /* Make sure we're in the idle state when going to airborne */
        state_get_flightstate(state, &flight_state);
        if (flight_state == STATE_IDLE) {
          state_set_flightstate(state, STATE_AIRBORNE);
          state_set_flightsubstate(state, SUBSTATE_ASCENT);
          detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_ASCENT);
        }
        break;
      case DETECTOR_APOGEE_EVENT:
        /* Make sure we're airborne already before setting to descent */
        state_get_flightstate(state, &flight_state);
        if (flight_state == STATE_AIRBORNE) {
          state_set_flightsubstate(state, SUBSTATE_DESCENT);
          detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_DESCENT);
        } else {
          detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_ASCENT);
        }
        break;
      case DETECTOR_LANDING_EVENT:
        /* We can set to landing from anywhere */
        state_set_flightstate(state, STATE_LANDED);
        detector_set_state(&detector, STATE_LANDED, SUBSTATE_UNKNOWN);
        /* Update our elevation so that we don't get false readings in IDLE and detect liftoff again */
        float elevation = detector_get_alt(&detector);
        detector_set_elevation(&detector, elevation);
        /* Also set the state in case we lose power, so we know where the ground level is */
        state_set_elevation(state, elevation * 1000); // Convert to millimeters
        break;
      default:
        /* Includes DETECTOR_NO_EVENT
         * The landed to idle state transition is done by the thread responsible for copying out the flight data
         */
        break;
    }
  }
}

/**
 * Calculates the current altitude above sea level using temperature adjusted barometer readings
 * @param baro_data The barometer data to use for the calculation
 * @return The calculated altitude and the timestamp from baro_data
 */
static struct fusion_altitude calculate_altitude(struct sensor_baro *baro_data) {
  struct fusion_altitude output;
  output.timestamp = baro_data->timestamp;

  /* Assume barometric reading is temperature adjusted */
  output.altitude = -(GAS_CONSTANT * KELVIN) / (MOLAR_MASS * GRAVITY) * log(baro_data->pressure / SEA_PRESSURE);
  return output;
}

/**
 * Calculates the magnitude of an acceleration reading
 * @param accel_data The accelerometer data to use in the calculation
 * @return The calculated magnitude and the timestamp from accel_data
 */
static struct accel_sample calculate_accel_magnitude(struct sensor_accel *accel_data) {
  struct accel_sample output;
  output.time = accel_data->timestamp;

  output.acceleration = sqrtf(powf(accel_data->x, 2) + powf(accel_data->y, 2) + powf(accel_data->z, 2));
  return output;
}
