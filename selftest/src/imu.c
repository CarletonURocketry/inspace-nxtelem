#include <uORB/uORB.h>

#include <testing/unity.h>

/* Acceleration limits in m/s^2 */

#define ACCEL_MIN -9.91
#define ACCEL_MAX 9.91

/* Gyro limits in rad/s */

#define GYRO_DELTA 0.06
#define GYRO_STATIONARY 0.0

/* Celsius temperature limits */

#define GROUND_TEMPERATURE 23.0
#define TEMPERATURE_DELTA 6.0

void selftest_accel(void) {
    const struct orb_metadata *meta;
    int fd;
    int err = 0;
    unsigned int frequency = 50;
    struct sensor_accel data;
    bool data_ready;
    bool have_data;

    meta = orb_get_meta("sensor_accel");
    TEST_ASSERT_MESSAGE(meta != NULL, "Couldn't get sensor_accel metadata.\n");

    fd = orb_subscribe(meta);
    TEST_ASSERT_MESSAGE(fd > 0, "Couldn't subscribe.\n");

    err = orb_set_frequency(fd, frequency);
    TEST_ASSERT_EQUAL_MESSAGE(0, err, "Couldn't set frequency.\n");

    /* Try to get data */

    have_data = false;
    data_ready = false;

    for (int i = 0; !data_ready && i < 5; i++) {

        usleep(100);

        err = orb_check(fd, &data_ready);
        TEST_ASSERT_EQUAL_MESSAGE(0, err, "Failed to check if data was ready.\n");

        if (data_ready) {
            err = orb_copy(meta, fd, &data);
            TEST_ASSERT_EQUAL_MESSAGE(0, err, "Failed to copy data.\n");

            /* We got data, we're done */

            have_data = true;
            break;
        }
    }

    TEST_ASSERT_MESSAGE(have_data, "Didn't get any data.\n");

    /* We have data, check if acceleration is fine. There should be only one
     * gravity vector.
     */

    // TODO

    /* Check the temperature ranges */

    TEST_ASSERT_FLOAT_WITHIN(TEMPERATURE_DELTA, GROUND_TEMPERATURE, data.temperature);
    printf("Temperature read %.2f Celsius\n", data.temperature);

    /* Perform the chip self-test */

    err = orb_ioctl(fd, SNIOC_SELFTEST, 0);
    TEST_ASSERT_EQUAL_MESSAGE(0, err, "Accelerometer self-test failed.\n");
    TEST_ASSERT_EQUAL_MESSAGE(0, orb_unsubscribe(fd), "Failed to unsubscribe.");
}

void selftest_gyro(void) {
    const struct orb_metadata *meta;
    int fd;
    int err = 0;
    unsigned int frequency = 50;
    struct sensor_gyro data;
    bool data_ready;
    bool have_data;

    meta = orb_get_meta("sensor_gyro");
    TEST_ASSERT_MESSAGE(meta != NULL, "Couldn't get sensor_gyro metadata.\n");

    fd = orb_subscribe(meta);
    TEST_ASSERT_MESSAGE(fd > 0, "Couldn't subscribe.\n");

    err = orb_set_frequency(fd, frequency);
    TEST_ASSERT_EQUAL_MESSAGE(0, err, "Couldn't set frequency.\n");

    /* Try to get data */

    have_data = false;
    data_ready = false;

    for (int i = 0; !data_ready && i < 5; i++) {

        usleep(100);

        err = orb_check(fd, &data_ready);
        TEST_ASSERT_EQUAL_MESSAGE(0, err, "Failed to check if data was ready.\n");

        if (data_ready) {
            err = orb_copy(meta, fd, &data);
            TEST_ASSERT_EQUAL_MESSAGE(0, err, "Failed to copy data.\n");

            /* We got data, we're done */

            have_data = true;
            break;
        }
    }

    TEST_ASSERT_MESSAGE(have_data, "Couldn't get any data.\n");

    /* We have data, check if angular velocity is close to 0. */

    TEST_ASSERT_FLOAT_WITHIN(GYRO_DELTA, GYRO_STATIONARY, data.x);
    printf("Angular velocity X read %.2f rad/s\n", data.x);

    TEST_ASSERT_FLOAT_WITHIN(GYRO_DELTA, GYRO_STATIONARY, data.y);
    printf("Angular velocity Y read %.2f rad/s\n", data.y);

    TEST_ASSERT_FLOAT_WITHIN(GYRO_DELTA, GYRO_STATIONARY, data.z);
    printf("Angular velocity Z read %.2f rad/s\n", data.z);

    /* Check temperature ranges */

    TEST_ASSERT_FLOAT_WITHIN(TEMPERATURE_DELTA, GROUND_TEMPERATURE, data.temperature);
    printf("Temperature read %.2f Celsius\n", data.temperature);

    /* Perform the chip self-test */

    err = orb_ioctl(fd, SNIOC_SELFTEST, 0);
    TEST_ASSERT_EQUAL_MESSAGE(0, err, "Accelerometer self-test failed.\n");
    TEST_ASSERT_EQUAL_MESSAGE(0, orb_unsubscribe(fd), "Failed to unsubscribe.");
}
