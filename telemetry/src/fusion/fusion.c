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

/* The universal gas constant. */

#define GAS_CONSTANT 8.31432

/* Constant for acceleration due to gravity. */

#define GRAVITY 9.80665

/* Constant for the mean molar mass of atmospheric gases. */

#define MOLAR_MASS 0.0289644

/** Defines constant for the absolute temperature in Kelvins. */
#define KELVIN 273

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
  int32_t elevation;

  state_get_flightstate(state, &flight_state);
  state_get_flightsubstate(state, &flight_substate)
  state_get_elevation(state, &elevation);

  /* Input sensors, may want to directly read instead */
  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.baro, orb_get_meta("sensor_baro"));
  setup_sensor(&sensors.accel, orb_get_meta("sensor_accel"));
  struct sensor_baro baro_data[BARO_INPUT_BUFFER_SIZE];
  struct sensor_accel accel_data[ACCEL_INPUT_BUFFER_SIZE];

  struct detector detector;
  detector_init(&detector);
  detector_set_state(flight_state, flight_substate);
  detector_set_elevation(&detector, elevation);

  /* Currently publishing blank data to start, might be better to try and advertise only on first fusioned data */
  struct fusion_altitude calculated_altitude = {.altitude = 0, .timestamp = orb_absolute_time()};
  int altitude_fd = orb_advertise_multi_queue(ORB_ID(fusion_altitude), &calculated_altitude, NULL, ALT_FUSION_BUFFER);
  if (altitude_fd < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Fusion could not advertise altitude topic: %d\n", altitude_fd);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  /* Perform fusion on sensor data endlessly */

  for(;;) {
    /* Wait for new data */
    poll_sensors(&sensors);
    int len = get_sensor_data(&sensors.baro, baro_data, sizeof(baro_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
        calculated_altitude = calculate_altitude(&baro_data[i]);
        detector_add_alt(detector, (struct altitude_sample *)&calculate_altitude);
        orb_publish(ORB_ID(fusion_altitude), altitude_fd, &calculated_altitude);
        /* Do some processing or fusion on this data */
      }
    }
    len = get_sensor_data(&sensors.accel, accel_data, sizeof(accel_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
        calculated_altitude = calculate_accel_magnitude(&accel_data[i]);
        detector_add_accel(detector, (struct accel_sample *)&accel_data);
      }
    }
    /* Run detection. Could choose to only run after a certain amount of time has passed */
    switch(detector_detect(detector)) {
      case DETECTOR_AIRBORNE_EVENT:
        /* Make sure we're in the idle state when going to airborne */
        state_get_flightstate(state, &flight_state);
        if (flight_state == STATE_IDLE) {
          state_set_flightstate(state, STATE_AIRBORNE);
          state_set_flightsubstate(state, SUBSTATE_ASCENT);
          detector_set_state(STATE_AIRBORNE, SUBSTATE_ASCENT);
        }
        break;
      case DETECTOR_APOGEE_EVENT:
        /* Make sure we're airborne already before setting to descent */
        state_get_flightstate(state, &flight_state);
        if (flight_state == STATE_AIRBORNE) {
          state_set_flightsubstate(state, SUBSTATE_DESCENT);
          detector_set_state(STATE_AIRBORNE, SUBSTATE_DESCENT);
        } else {
          detector_set_state(STATE_AIRBORNE, SUBSTATE_ASCENT);
        }
        break;
      case DETECTOR_LANDING_EVENT:
        /* We can set to landing from anywhere */
        state_set_flightstate(state, STATE_LANDED);
        break;
      default:
        // Includes DETECTOR_NO_EVENT
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
