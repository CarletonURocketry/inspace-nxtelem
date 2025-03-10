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
int get_sensor_data(struct pollfd *sensor, void *data, size_t size);

/**
 * Clears the uorb_inputs struct to make no sensors get polled on accidentally. After this,
 * should be fine to only set up a the desired sensors and poll everything
 * 
 * @param sensors The uorb_inputs struct to clear
 */
static void clear_uorb_inputs(struct uorb_inputs *sensors) {
  struct pollfd *sensor_array = (struct pollfd *)sensors;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor_array[i].fd = -1;
    sensor_array[i].events = 0;
  }
}

/**
 * Polls on all sensors in the uorb_inputs struct
 * 
 * @param sensors The uorb_inputs struct to poll on
 */
static void poll_sensors(struct uorb_inputs *sensors) {
    poll((struct pollfd *)sensors, NUM_SENSORS, -1);
}

#endif // _INSPACE_SENSORS_H_

