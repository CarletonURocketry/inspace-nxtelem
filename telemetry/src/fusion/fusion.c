#include <pthread.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>

#include "../sensors/sensors.h"
#include "fusion.h"

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
#include <stdio.h>
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */


#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
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

#define ACCEL_MULTI_BUFFER_SIZE 5
#define BARO_MULTI_BUFFER_SIZE 5

void *fusion_main(void *arg) {
  int err;

  /* Input sensors, may want to directly read instead */
  struct uorb_inputs sensors;
  clear_uorb_inputs(&sensors);
  setup_sensor(&sensors.accel, orb_get_meta("sensor_accel"));
  setup_sensor(&sensors.baro, orb_get_meta("sensor_baro"));
  struct sensor_accel accel_data[ACCEL_MULTI_BUFFER_SIZE];
  struct sensor_baro baro_data[BARO_MULTI_BUFFER_SIZE];

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
    poll_sensors(&sensors);
    int len = get_sensor_data(&sensors.accel, accel_data, sizeof(accel_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
        /* Simply publish the same data we got for now*/
        orb_publish(ORB_ID(fusion_accel), accel_out, &accel_data[i]);
      }
    }
    len = get_sensor_data(&sensors.baro, baro_data, sizeof(baro_data));
    if (len > 0) {
      for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
        orb_publish(ORB_ID(fusion_baro), baro_out, &baro_data[i]);
        /* Do some processing or fusion on this data */
      }
    }
  }
}
