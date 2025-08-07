#include <math.h>
#include <nuttx/sensors/sensor.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include "../rocket-state/rocket-state.h"
#include "../sensors/sensors.h"
#include "../syslogging.h"
#include "detector.h"
#include "fusion.h"
#include "uORB/uORB.h"

/* The pressure at sea level in millibar*/
#define SEA_PRESSURE (1013.25)

/* The universal gas constant */
#define GAS_CONSTANT (8.31432)

/* Constant for acceleration due to gravity */
#define GRAVITY (9.80665)

/* Constant for the mean molar mass of atmospheric gases */
#define MOLAR_MASS (0.0289644)

/* Constant for the conversion from Celsius to Kelvin */
#define KELVIN (273)

/* UORB declarations */

#if defined(CONFIG_DEBUG_UORB)
static const char fusion_alt_format[] = "fusioned altitude - timestamp:%" PRIu64 ",altitude:%hf";
ORB_DEFINE(fusion_altitude, struct fusion_altitude, fusion_alt_format);
#else
ORB_DEFINE(fusion_altitude, struct fusion_altitude, 0);
#endif

/* Buffers for inputs, best to match to the size of the internal sensor buffers */
#define BARO_INPUT_BUFFER_SIZE 5

/* Buffers for inputs, best to match to the size of the internal sensor buffers */
#define ACCEL_INPUT_BUFFER_SIZE 5

static struct fusion_altitude calculate_altitude(struct sensor_baro *baro_data);
static struct accel_sample calculate_accel_magnitude(struct sensor_accel *accel_data);
static ssize_t get_sensor_data(struct pollfd *sensor, void *data, size_t size);

void *fusion_main(void *arg) {
    rocket_state_t *state = ((struct fusion_args *)arg)->state;
    enum flight_state_e flight_state;
    enum flight_substate_e flight_substate;
    struct sensor_baro baro_data[BARO_INPUT_BUFFER_SIZE];
    struct sensor_accel accel_data[ACCEL_INPUT_BUFFER_SIZE];
    struct detector detector;
    struct fusion_altitude calculated_altitude;
    struct accel_sample calculated_accel_mag;
    struct pollfd fds[2] = {0};
    const struct orb_metadata *barometa;
    const struct orb_metadata *accelmeta;

    state_get_flightstate(state, &flight_state);
    state_get_flightsubstate(state, &flight_substate);

    /* Input sensors, may want to directly read instead */

    // TODO make this way cleaner and handle errors
    barometa = orb_get_meta("sensor_baro");
    accelmeta = orb_get_meta("sensor_accel");

    fds[0].fd = orb_subscribe(barometa);
    fds[1].fd = orb_subscribe(accelmeta);

    detector_init(&detector, orb_absolute_time());
    detector_set_state(&detector, flight_state, flight_substate);

    /* Set the elevation, which will either be a remembered value or a sensible default (and convert to meters) */
    // detector_set_elevation(&detector, init_elevation / 1000);

    /* Currently publishing blank data to start, might be better to try and advertise only on first fusioned data */

    int altitude_fd = orb_advertise_multi_queue(ORB_ID(fusion_altitude), NULL, NULL, ALT_FUSION_BUFFER);
    if (altitude_fd < 0) {
        inerr("Fusion could not advertise altitude topic: %d\n", altitude_fd);
    }

    /* Output sensors */

    for (;;) {

        /* Wait for new data */

        poll(fds, sizeof(fds) / sizeof(fds[0]), -1);

        int len = get_sensor_data(&fds[0], baro_data, sizeof(baro_data));

        if (len > 0) {
            for (int i = 0; i < (len / sizeof(struct sensor_baro)); i++) {
                calculated_altitude = calculate_altitude(&baro_data[i]);
                detector_add_alt(&detector, (struct altitude_sample *)&calculated_altitude);
                orb_publish(ORB_ID(fusion_altitude), altitude_fd, &calculated_altitude);
            }
        }

        len = get_sensor_data(&fds[1], accel_data, sizeof(accel_data));

        if (len > 0) {
            for (int i = 0; i < (len / sizeof(struct sensor_accel)); i++) {
                calculated_accel_mag = calculate_accel_magnitude(&accel_data[i]);
                detector_add_accel(&detector, (struct accel_sample *)&calculated_accel_mag);
            }
        }

        /* Run detection. Potentially run periodically instead of every update */

        switch (detector_detect(&detector)) {
        case DETECTOR_AIRBORNE_EVENT: {
            /* Make sure we're in the idle state when going to airborne */
            state_get_flightstate(state, &flight_state);
            if (flight_state == STATE_IDLE) {
                ininfo("Changing the flight state, altitude is %f and acceleration is %f\n",
                       detector_get_alt(&detector), detector_get_accel(&detector));
                state_set_flightstate(state, STATE_AIRBORNE);
                state_set_flightsubstate(state, SUBSTATE_ASCENT);
                detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_ASCENT);
            }
        } break;

        case DETECTOR_APOGEE_EVENT: {
            /* Make sure we're airborne already before setting to descent */
            state_get_flightstate(state, &flight_state);
            if (flight_state == STATE_AIRBORNE) {
                ininfo("Changing the flight state, altitude is %f and acceleration is %f\n",
                       detector_get_alt(&detector), detector_get_accel(&detector));
                state_set_flightsubstate(state, SUBSTATE_DESCENT);
                detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_DESCENT);
            } else {
                detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_ASCENT);
            }
        } break;

        case DETECTOR_LANDING_EVENT: {
            /* We can set to landing from anywhere */
            ininfo("Changing the flight state, altitude is %f and acceleration is %f\n", detector_get_alt(&detector),
                   detector_get_accel(&detector));
            state_set_flightstate(state, STATE_LANDED);
            // Set the detector back to the landed state right away - airborne events will only cause a state transition
            // once the system is back in the idle state
            detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);
        } break;

        default:
            /* Includes DETECTOR_NO_EVENT
             * The landed to idle state transition is done by the thread responsible for copying out the flight data
             */
            break;
        }
    }
}

/**
 * Calculates the current altitude above sea level using temperature adjusted barometer readings
 * @param baro_data The barometer data to use for the calculation
 * @return The calculated altitude and the timestamp from baro_data
 */
struct fusion_altitude calculate_altitude(struct sensor_baro *baro_data) {
    struct fusion_altitude output;
    output.timestamp = baro_data->timestamp;

    /* Assume barometric reading is temperature adjusted */
    output.altitude = -(GAS_CONSTANT * (KELVIN + baro_data->temperature)) / (MOLAR_MASS * GRAVITY) *
                      log(baro_data->pressure / SEA_PRESSURE);
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

/* Gets data from the sensor if the POLLIN event has occured
 *
 * @param sensor A pollfd struct with a valid or invalid file descriptor
 * @param data An array of uORB data structs of the type this sensor uses
 * @param size The size of the data parameter in bytes
 * @return The number of bytes read from the sensor or 0 if there was none to read
 */
static ssize_t get_sensor_data(struct pollfd *sensor, void *data, size_t size) {
    ssize_t len;

    /* If the sensor wasn't set up right, POLLIN won't get set, meaning there's no need to avoid using
     * the sensor if its metadata or fd weren't set up properly
     */

    if (sensor->revents == POLLIN) {
        len = orb_copy_multi(sensor->fd, data, size);

        if (len < 0) {
            /* If there's no data to read or there's no fetch function, but that shouldn't happen if we get POLLIN */

            if (errno != ENODATA) {
                inerr("Error reading from uORB data: %d\n", errno);
            }
            return 0;
        }
        return len;
    }
    return 0;
}
