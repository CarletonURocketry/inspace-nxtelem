#ifndef _INSPACE_SENSORS_H_
#define _INSPACE_SENSORS_H_

#include <uORB/uORB.h>
#include <poll.h>

/* Used for polling on multiple sensors at once. Don't set up the ones you don't want to use */
struct uorb_inputs {
  struct pollfd accel;
  struct pollfd baro;
};

/* The numbers of sensors defined in uorb_inputs */
#define NUM_SENSORS sizeof(struct uorb_inputs) / sizeof(struct pollfd)

int setup_sensor(struct pollfd *sensor, orb_id_t meta);
ssize_t get_sensor_data(struct pollfd *sensor, void *data, size_t size);
void clear_uorb_inputs(struct uorb_inputs *sensors);
void poll_sensors(struct uorb_inputs *sensors);

#endif // _INSPACE_SENSORS_H_

