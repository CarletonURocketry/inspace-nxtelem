#include <uORB/uORB.h>

#include "syslogging.h"

/* Acceleration limits in m/s^2 */

#define ACCEL_MIN -9.91
#define ACCEL_MAX 9.91

/* Gyro limits in rad/s */

#define GYRO_MIN -0.06
#define GYRO_MAX 0.06

/* Celsius temperature limits */

#define TEMP_MIN 16.0
#define TEMP_MAX 28.0

static int selftest_accel(void) {

    const struct orb_metadata *meta;
    int fd;
    int err = 0;
    unsigned int frequency = 50;
    struct sensor_accel data;
    bool data_ready;
    bool have_data;

    meta = orb_get_meta("sensor_accel");
    if (meta == NULL) {
        serr("Couldn't get sensor_accel metadata.\n");
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

    /* We have data, check if acceleration is fine. There should be only one
     * gravity vector.
     */

    // TODO

    /* Check the temperature ranges */

    if (!(TEMP_MIN <= data.temperature && data.temperature <= TEMP_MAX)) {
        serr("Temperature %.3f outside of expected range (%.2f, %.2f) in Celsius\n", data.temperature, TEMP_MIN,
             TEMP_MAX);
        err = EINVAL;
        goto early_ret;
    } else {
        printf("Temperature read %.2f Celsius\n", data.temperature);
    }

    /* Perform the chip self-test */

    err = orb_ioctl(fd, SNIOC_SELFTEST, 0);
    if (err < 0) {
        err = errno;
        serr("Accelerometer chip self-test failed: %d\n", errno);
    } else {
        printf("Accelerometer chip self-test passed.\n");
    }

early_ret:
    orb_unsubscribe(fd);
    return err;
}

static int selftest_gyro(void) {

    const struct orb_metadata *meta;
    int fd;
    int err = 0;
    unsigned int frequency = 50;
    struct sensor_gyro data;
    bool data_ready;
    bool have_data;

    meta = orb_get_meta("sensor_gyro");
    if (meta == NULL) {
        serr("Couldn't get sensor_gyro metadata.\n");
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

    /* We have data, check if angular velocity is close to 0. */

    if (!(GYRO_MIN <= data.x && data.x <= GYRO_MAX)) {
        serr("Angular velocity (X) %.3f outside of expected range (%.2f, %.2f) rad/s\n", data.x, GYRO_MIN, GYRO_MAX);
        err = EINVAL;
        goto early_ret;
    } else {
        printf("Angular velocity X read %.2f rad/s\n", data.x);
    }

    if (!(GYRO_MIN <= data.y && data.y <= GYRO_MAX)) {
        serr("Angular velocity (Y) %.3f outside of expected range (%.2f, %.2f) rad/s\n", data.y, GYRO_MIN, GYRO_MAX);
        err = EINVAL;
        goto early_ret;
    } else {
        printf("Angular velocity Y read %.2f rad/s\n", data.y);
    }

    if (!(GYRO_MIN <= data.z && data.z <= GYRO_MAX)) {
        serr("Angular velocity (Z) %.3f outside of expected range (%.2f, %.2f) rad/s\n", data.z, GYRO_MIN, GYRO_MAX);
        err = EINVAL;
        goto early_ret;
    } else {
        printf("Angular velocity Z read %.2f rad/s\n", data.z);
    }

    /* Check temperature ranges */

    if (!(TEMP_MIN <= data.temperature && data.temperature <= TEMP_MAX)) {
        serr("Temperature %.3f outside of expected range (%.2f, %.2f) in Celsius\n", data.temperature, TEMP_MIN,
             TEMP_MAX);
        err = EINVAL;
        goto early_ret;
    } else {
        printf("Temperature read %.2f Celsius\n", data.temperature);
    }

    /* Perform the chip self-test */

    err = orb_ioctl(fd, SNIOC_SELFTEST, 0);
    if (err < 0) {
        err = errno;
        serr("Gyro chip self-test failed: %d\n", errno);
    } else {
        printf("Gyro chip self-test passed.\n");
    }

early_ret:
    orb_unsubscribe(fd);
    return err;
}

int selftest_imu(void) {
    int err;

    err = selftest_accel();
    if (err < 0) {
        return err;
    }

    return selftest_gyro();
}
