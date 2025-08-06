#ifndef _INSPACE_SENSORS_H_
#define _INSPACE_SENSORS_H_

#include <poll.h>
#include <uORB/uORB.h>

#include "../fusion/fusion.h"

/* Enumeration of the sensors in the uORB network. Includes fusion 'sensors' */

enum uorb_sensors {
#ifdef CONFIG_SENSORS_LSM6DSO32
    SENSOR_ACCEL, /* Accelerometer */
    SENSOR_GYRO,  /* Gyroscope */
#endif
#ifdef CONFIG_SENSORS_MS56XX
    SENSOR_BARO, /* Barometer */
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    SENSOR_MAG, /* Magnetometer */
#endif
#ifdef CONFIG_SENSORS_L86XXX
    SENSOR_GNSS, /* GNSS */
#endif
    SENSOR_ALT, /* Altitude fusion */
};

/* A buffer that can hold any of the types of data created by the sensors in uorb_inputs */

union uorb_data {
#ifdef CONFIG_SENSORS_LSM6DSO32
    struct sensor_accel accel;
    struct sensor_gyro gyro;
#endif
#ifdef CONFIG_SENSORS_MS56XX
    struct sensor_baro baro;
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    struct sensor_mag mag;
#endif
#ifdef CONFIG_SENSORS_L86XXX
    struct sensor_gnss gnss;
#endif
    struct fusion_altitude alt;
};

/**
 * A function pointer to a function that will perform operations on single pieces of uORB data
 *
 * @param context Context given to the callback by the program using this interface
 * @param element The element to perform processing on, where the length is implied by knowing the type of element
 */
typedef void (*uorb_data_callback_t)(void *context, uint8_t *element);

ssize_t get_sensor_data(struct pollfd *sensor, void *data, size_t size);
void *get_sensor_data_end(struct pollfd *sensor, void *data, size_t size);
void foreach_measurement(uorb_data_callback_t handler, void *handler_context, void *data, size_t size,
                         size_t elem_size);
int process_one(uorb_data_callback_t handler, void *handler_context, void **data_start, void *data_end,
                size_t elem_size);

#endif // _INSPACE_SENSORS_H_
