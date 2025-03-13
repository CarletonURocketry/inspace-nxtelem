#ifndef _INSPACE_SENSORS_H_
#define _INSPACE_SENSORS_H_

#include <uORB/uORB.h>
#include <poll.h>

/* Used for polling on multiple sensors at once. Don't set up the ones you don't want to use */
struct uorb_inputs {
  struct pollfd accel;
  struct pollfd baro;
  struct pollfd mag;
  struct pollfd gyro;
  struct pollfd gnss;
};

/* A buffer that can hold any of the types of data created by the sensors in uorb_inputs */
union uorb_data {
  struct sensor_accel accel;
  struct sensor_baro baro;
  struct sensor_mag mag;
  struct sensor_gyro gyro;
  struct sensor_gnss gnss;
};

/* The numbers of sensors defined in uorb_inputs */
#define NUM_SENSORS sizeof(struct uorb_inputs) / sizeof(struct pollfd)

/**
 * A function pointer to a function that will perform operations on single reads of uORB data
 *
 * @param context Context given to the callback from the user
 * @param sensor_type The type of the sensor the data is from
 * @param buf A buffer at least the size of the struct that the type of sensor uses
 * @param bufsize The size of the buffer in bytes
 */
typedef void (*uorb_data_callback_t)(void* context, uint8_t* buf, size_t bufsize);

int setup_sensor(struct pollfd *sensor, orb_id_t meta);
ssize_t get_sensor_data(struct pollfd *sensor, void *data, size_t size);
void clear_uorb_inputs(struct uorb_inputs *sensors);
void poll_sensors(struct uorb_inputs *sensors);
void for_all_data(uorb_data_callback_t handler, void* handler_context, struct pollfd *sensor, uint8_t *buf, size_t size);

#endif // _INSPACE_SENSORS_H_

