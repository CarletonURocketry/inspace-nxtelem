#include <pthread.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>
#include <math.h>

#include "../sensors/sensors.h"
#include "fusion.h"

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

struct fusion_altitude calculate_altitude(struct sensor_baro *baro_data);

void *fusion_main(void *arg) {
  /* Input sensors, may want to directly read instead */
  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.baro, orb_get_meta("sensor_baro"));
  struct sensor_baro baro_data[BARO_INPUT_BUFFER_SIZE];

  /* Output sensors */ 

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
        orb_publish(ORB_ID(fusion_altitude), altitude_fd, &calculated_altitude);
        /* Do some processing or fusion on this data */
      }
    }
  }
}

/**
 * Calculates the current altitude above sea level using temperature adjusted barometer readings
 * @param baro_data The barometer data to use for the calculation
 */
struct fusion_altitude calculate_altitude(struct sensor_baro *baro_data) {
  struct fusion_altitude output;
  output.timestamp = baro_data->timestamp;

  /* Assume barometric reading is temperature adjusted */
  output.altitude = -(GAS_CONSTANT * KELVIN) / (MOLAR_MASS * GRAVITY) * log(baro_data->pressure / SEA_PRESSURE);
  return output;
}
