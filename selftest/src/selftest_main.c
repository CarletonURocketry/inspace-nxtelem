#include <stdio.h>
#include <stdlib.h>

#include "syslogging.h"
#include "testfuncs.h"

/* Helper macros */

#define STR(x) #x
#define XSTR(x) STR(x)

#define run_test(errvar, errctr, func)                                                                                 \
    do {                                                                                                               \
        errvar = func();                                                                                               \
        if (errvar) {                                                                                                  \
            errctr++;                                                                                                  \
            serr("Test '%s' failed: %d", XSTR(func), errvar);                                                          \
        } else {                                                                                                       \
            ssuc("Test '%s' passed!", XSTR(func));                                                                     \
        }                                                                                                              \
    } while (0)

int main(int argc, char **argv) {

    int err;
    unsigned int tests_skipped = 0;
    unsigned int tests_failed = 0;

    printf("This is the self-test for the hardware present on Josh REV B.\n");
    printf("You will need to monitor the console output of this test to make sure that");
    printf("LEDs other non-verifiable actuators do what they're supposed to at the right time.\n");

    printf("Test beginning.");

    sleep(3);

#ifdef CONFIG_SENSORS_LIS2MDL
    run_test(err, tests_failed, selftest_mag);
#else
    swarn("Magnetometer test skipped, enable CONFIG_SENSORS_LIS2MDL to test this.");
    tests_skipped++;
#endif

#ifdef CONFIG_SENSORS_MS56XX
    run_test(err, tests_failed, selftest_baro);
#else
    swarn("Barometer test skipped, enable CONFIG_SENSORS_MS56XX to test this.");
    tests_skipped++;
#endif

#ifdef CONFIG_SENSORS_LSM6DSO32
    run_test(err, tests_failed, selftest_imu);
#else
    swarn("IMU test skipped, enable CONFIG_SENSORS_LSM6DSO32 to test this.");
    tests_skipped++;
#endif

#ifdef CONFIG_LPWAN_RN2XX3
    run_test(err, tests_failed, selftest_radio);
#else
    swarn("Radio test skipped, enable CONFIG_WIRELESS_RN2XX3 to test this.");
    tests_skipped++;
#endif

    printf("TEST RESULTS:\n");

    if (tests_failed > 0) {
        serr("%u tests failed.", tests_failed);
    }

    swarn("%u tests skipped.", tests_skipped);

    return 0;
}
