#include <uORB/uORB.h>

#include "syslogging.h"

/* Millibar pressure limits */

#define PRESSURE_MIN 990.0
#define PRESSURE_MAX 1100.0

/* Celsius temperature limits */

#define TEMP_MIN 16.0
#define TEMP_MAX 28.0

int selftest_baro(void) {

    const struct orb_metadata *meta;
    int fd;
    int err = 0;
    unsigned int frequency = 20;
    struct sensor_baro data;
    bool data_ready;
    bool have_data;

    meta = orb_get_meta("sensor_baro");
    if (meta == NULL) {
        serr("Couldn't get sensor_baro metadata.\n");
        return ENOENT;
    }

    fd = orb_subscribe(meta);
    if (fd < 0) {
        err = errno;
        serr("Couldn't subscribe to %s: %d\n", meta->o_name, err);
        return err;
    }

    if (orb_set_frequency(fd, frequency) < 0) {
        err = errno;
        serr("Couldn't set frequency of %s to %u Hz: %d", meta->o_name, frequency, err);
        goto early_ret;
    }

    /* Try to get data */

    have_data = false;
    data_ready = false;

    for (int i = 0; !data_ready && i < 5; i++) {

        usleep(100);

        err = orb_check(fd, &data_ready);
        if (err < 0) {
            err = errno;
            serr("Error checking for ready data: %d\n", err);
            continue;
        }

        if (data_ready) {
            err = orb_copy(meta, fd, &data);
            if (err < 0) {
                err = errno;
                serr("Error copying uORB data: %d\n", errno);
                continue;
            }

            /* We got data, we're done */
            have_data = true;
            break;
        }
    }

    if (!have_data) {
        serr("Couldn't read any data.\n");
        err = EIO;
        goto early_ret;
    }

    /* We have data, check if pressure is within ground pressure ranges. */

    if (!(PRESSURE_MIN <= data.pressure && data.pressure <= PRESSURE_MAX)) {
        serr("Pressure %.3f outside of expected range (%.2f, %.2f) in mbar\n", data.pressure, PRESSURE_MIN,
             PRESSURE_MAX);
        err = EINVAL;
        goto early_ret;
    } else {
        printf("Pressure read %.2f mbar\n", data.pressure);
    }

    /* Check if temperature is around ambient (might be hot/cold with AC or
     * outside)
     */

    if (!(TEMP_MIN <= data.temperature && data.temperature <= TEMP_MAX)) {
        serr("Temperature %.3f outside of expected range (%.2f, %.2f) in Celsius\n", data.temperature, TEMP_MIN,
             TEMP_MAX);
        err = EINVAL;
        goto early_ret;
    } else {
        printf("Temperature read %.2f Celsius\n", data.temperature);
    }

    err = 0; /* Success */

early_ret:
    orb_unsubscribe(fd);
    return err;
}
