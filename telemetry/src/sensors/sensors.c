#include <poll.h>
#include "sensors.h"

/**
 * Set up a pollfd structure for a uORB sensor
 *
 * @param sensor A pollfd struct that will be initialized with a uORB fd and for POLLIN events
 * @param meta The metadata for the topic the sensor will be subscribed to
 * @return 0 on success or a negative error code
 */
int setup_sensor(struct pollfd *sensor, orb_id_t meta) {
  if (meta == NULL) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Could not set up sensor, missing metadata\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    return -1;
  }
  sensor->fd = orb_subscribe(meta);
  sensor->events = POLLIN;
  if (sensor->fd < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Sensor %s was not opened successfully", meta->o_name);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    sensor->events = POLLIN;
    return sensor->fd;
  }
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  else {
    struct orb_state state;
    orb_get_state(sensor->fd, &state);
    printf("Setup successful for sensor %s\n", meta->o_name);
    /* Not always useful
    printf("Maximum Frequency: %d\n", state.max_frequency);
    printf("Min Batch Interval: %d\n", state.min_batch_interval);
    printf("Internal Queue Size: %d\n", state.queue_size);
    printf("Subscribers: %d\n", state.nsubscribers);
    printf("Generation: %d\n", state.generation);
    */
  }
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  return 0;
}

/**
 * Gets data from the sensor if the POLLIN event has occured
 *
 * @param sensor A pollfd struct with a valid or invalid file descriptor
 * @param data An array of uORB data structs of the type this sensor uses
 * @param size The size of the data parameter in bytes
 * @return The number of bytes read from the sensor or 0 if there was none to read
 */
ssize_t get_sensor_data(struct pollfd *sensor, void *data, size_t size) {
    /*
     * If the sensor wasn't set up right, POLLIN won't get set, meaning there's no need to avoid using
     * the sensor if its metadata or fd weren't set up properly
     */
    if (sensor->revents == POLLIN) {
      ssize_t len = orb_copy_multi(sensor->fd, data, size);
      if (len < 0) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Collection: Error reading from uORB data: %ld\n", len);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
      return len;
    }
    return 0;
}

/**
 * Clears the uorb_inputs struct to make no sensors get polled on accidentally. After this,
 * should be fine to only set up a the desired sensors and poll everything
 *
 * @param sensors The uorb_inputs struct to clear
 */
void clear_uorb_inputs(struct uorb_inputs *sensors) {
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
void poll_sensors(struct uorb_inputs *sensors) {
    poll((struct pollfd *)sensors, NUM_SENSORS, -1);
}
