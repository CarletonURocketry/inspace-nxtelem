#include <uORB/uORB.h>

#include <testing/unity.h>

/* Millibar pressure limits */

#define GROUND_PRESSURE 1010.0
#define PRESSURE_DELTA 100.0

/* Celsius temperature limits */

#define GROUND_TEMPERATURE 23.0
#define TEMPERATURE_DELTA 6.0

void selftest_baro(void) {
    const struct orb_metadata *meta;
    int fd;
    int err = 0;
    unsigned int frequency = 20;
    struct sensor_baro data;
    bool data_ready;
    bool have_data;

    meta = orb_get_meta("sensor_baro");
    TEST_ASSERT_MESSAGE(meta != NULL, "Couldn't get sensor_baro metadata.\n");

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

    /* We have data, check if pressure is within ground pressure ranges. */

    TEST_ASSERT_FLOAT_WITHIN(PRESSURE_DELTA, GROUND_PRESSURE, data.pressure);
    printf("Pressure read %.2f mbar\n", data.pressure);

    /* Check if temperature is around ambient (might be hot/cold with AC or
     * outside)
     */

    TEST_ASSERT_FLOAT_WITHIN(TEMPERATURE_DELTA, GROUND_TEMPERATURE, data.temperature);
    printf("Temperature read %.2f Celsius\n", data.temperature);

    TEST_ASSERT_EQUAL_MESSAGE(0, orb_unsubscribe(fd), "Failed to unsubscribe.");
}
