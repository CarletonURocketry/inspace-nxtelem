#include <pthread.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>

#include "../sensors/sensors.h"
#include "fusion.h"

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */


/* UORB declarations */
#if defined(CONFIG_DEBUG_UORB)
static const char fusion_alt_format[] = "fusioned altitude - timestamp:%" PRIu64 ",altitude:%hf";
ORB_DEFINE(fusion_altitude, struct fusion_altitude, fusion_alt_format);
#else
ORB_DEFINE(fusion_altitude, struct fusion_altitude);
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
  int altitude_fd = orb_advertise_multi_queue(ORB_ID(fusion_altitude), &calculated_altitude, NULL, ACCEL_FUSION_BUFFER_SIZE);
  if (altitude_fd < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Fusion could not advertise accel topic: %d\n", altitude_fd);
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

struct fusion_altitude calculate_altitude(struct sensor_baro *baro_data) {
  struct fusion_altitude output;
  output.timestamp = baro_data->timestamp;
  output.altitude = 0;
  return output;
}
