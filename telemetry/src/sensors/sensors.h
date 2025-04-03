#ifndef _INSPACE_SENSORS_H_
#define _INSPACE_SENSORS_H_

#include <uORB/uORB.h>
#include <poll.h>

#include "../fusion/fusion.h"

/* Used for polling on multiple sensors at once. Don't set up the ones you don't want to use */
struct uorb_inputs {
  struct pollfd accel;
  struct pollfd baro;
  struct pollfd mag;
  struct pollfd gyro;
  struct pollfd gnss;
  struct pollfd alt;
};

/* A buffer that can hold any of the types of data created by the sensors in uorb_inputs */
union uorb_data {
  struct sensor_accel accel;
  struct sensor_baro baro;
  struct sensor_mag mag;
  struct sensor_gyro gyro;
  struct sensor_gnss gnss;
  struct fusion_altitude alt;
};

/* The numbers of sensors defined in uorb_inputs */
#define NUM_SENSORS sizeof(struct uorb_inputs) / sizeof(struct pollfd)

/**
 * A function pointer to a function that will perform operations on single pieces of uORB data
 *
 * @param context Context given to the callback by the program using this interface
 * @param element The element to perform processing on, where the length is implied by knowing the type of element
 */
typedef void (*uorb_data_callback_t)(void* context, uint8_t* element);

int setup_sensor(struct pollfd *sensor, orb_id_t meta);
ssize_t get_sensor_data(struct pollfd *sensor, void *data, size_t size);
void *get_sensor_data_end(struct pollfd *sensor, void* data, size_t size);
void clear_uorb_inputs(struct uorb_inputs *sensors);
void poll_sensors(struct uorb_inputs *sensors);
void foreach_measurement(uorb_data_callback_t handler, void* handler_context, void *data, size_t size, size_t elem_size);
int process_one(uorb_data_callback_t handler, void* handler_context, void **data_start, void *data_end, size_t elem_size);

#endif // _INSPACE_SENSORS_H_
